/*
 * Bumblebee acro application for the Crazyflie 2.1+.
 *
 * Copyright (C) 2026 Tobias Aschenbrenner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "commander.h"
#include "log.h"
#include "motors.h"
#include "param.h"
#include "supervisor.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "BUMBLEBEE"
#include "debug.h"

#define APP_VERSION 9
#define FLIP_REQUEST_MAGIC 0xB33F
#define LIFT_TEST_REQUEST_MAGIC 0xB11F
#define LOOP_PERIOD_MS 5
#define ACRO_PERMIT_MS 1450

#define MIN_START_ALTITUDE_M 0.75f
#define MAX_START_ALTITUDE_M 1.25f
#define MAX_MANEUVER_ALTITUDE_M 1.45f
#define MIN_START_VOLTAGE_V 3.50f
#define MAX_START_TILT_DEG 10.0f
#define MAX_BOOST_TILT_DEG 15.0f
#define MIN_HOVER_THRUST 20000.0f
#define MAX_HOVER_THRUST 55000.0f
#define MAX_THRUST 60000.0f

#define SPINUP_TIMEOUT_MS 180
#define ROTATION_TIMEOUT_MS 900
#define BRAKE_TIMEOUT_MS 500
#define RECOVERY_OBSERVATION_MS 400
#define SPINUP_READY_RATE_DEG_S 700.0f
#define ROTATION_RATE_DEG_S 950.0f
#define FINISH_RATE_DEG_S 250.0f
#define BRAKE_READY_RATE_DEG_S 180.0f
#define TARGET_ROTATION_DEG 335.0f
#define MAX_ROTATION_DEG 410.0f
#define UPRIGHT_TOLERANCE_DEG 25.0f
#define RECOVERY_LEVEL_TOLERANCE_DEG 20.0f
#define MIN_RECOVERY_ALTITUDE_M 0.55f

#define MIN_BOOST_HEIGHT_MM 100
#define MAX_BOOST_HEIGHT_MM 300
#define MIN_BOOST_DURATION_MS 150
#define MAX_BOOST_DURATION_MS 350
#define MIN_BRAKE_START_DEG 80
#define MAX_BRAKE_START_DEG 220
#define MIN_COUNTER_RATE_DEG_S 600
#define MAX_COUNTER_RATE_DEG_S 1200
#define MIN_SPIN_THRUST_PERCENT 25
#define MAX_SPIN_THRUST_PERCENT 55
#define MIN_COAST_THRUST_PERCENT 15
#define MAX_COAST_THRUST_PERCENT 35
#define MIN_BRAKE_THRUST_PERCENT 30
#define MAX_BRAKE_THRUST_PERCENT 60
#define MIN_RECOVERY_DURATION_MS 250
#define MAX_RECOVERY_DURATION_MS 500

typedef enum {
  flipStateIdle = 0,
  flipStateBoost = 1,
  flipStateSpinup = 2,
  flipStateCoast = 3,
  flipStateBrake = 4,
  flipStateRecover = 5,
  flipStateVerify = 6,
  flipStateComplete = 7,
  flipStateAborted = 8,
} FlipState;

typedef enum {
  flipResultNone = 0,
  flipResultSuccess = 1,
  flipResultNotFlying = 2,
  flipResultLowBattery = 3,
  flipResultNotLevel = 4,
  flipResultAltitude = 5,
  flipResultNoHoverThrust = 6,
  flipResultRotationTimeout = 7,
  flipResultSupervisorStopped = 8,
  flipResultPermitDenied = 9,
  flipResultNotUpright = 10,
  flipResultRecoveryFailed = 11,
  flipResultProfileDisabled = 12,
  flipResultProfileInvalid = 13,
  flipResultBrakingFailed = 14,
} FlipResult;

typedef enum {
  rotationCompleted = 0,
  rotationTimedOut = 1,
  rotationSupervisorStopped = 2,
  rotationNotUpright = 3,
  rotationBrakingFailed = 4,
} RotationResult;

static const uint8_t firmwareVersion = APP_VERSION;
static uint16_t flipRequest;
static int8_t direction = 1;

// The Mac writes every profile value before triggering. Values are volatile
// and return to these guarded defaults after a power cycle.
static uint16_t profileBoostMm = 100;
static uint16_t profileBoostMs = 150;
static uint16_t profileBrakeDeg = 100;
static uint16_t profileCounterRate = 1200;
static uint8_t profileSpinPct = 40;
static uint8_t profileCoastPct = 20;
static uint8_t profileBrakePct = 60;
static uint16_t profileRecoverMs = 500;

static uint8_t flipState;
static uint8_t flipResult;
static float rotationDeg;
static float rawRotationDeg;
static float peakPitchRate;
static float rotationEndRoll;
static float rotationEndPitch;
static float learnedHoverThrust;
static float startAltitude;
static float minimumAltitude;
static float minimumVoltage;
static float boostGain;
static float brakeStartRotation;
static float brakeEndRate;
static float recoveryStartAltitude;
static uint8_t acroPermitActive;
static uint8_t immediateStopReady;

static logVarId_t altitudeId;
static logVarId_t batteryId;
static logVarId_t pitchId;
static logVarId_t rollId;
static logVarId_t pitchRateId;
static logVarId_t thrustId;

static float clampedThrust(const float thrust) {
  return fminf(MAX_THRUST, fmaxf(0.0f, thrust));
}

static bool supervisorAllowsManeuver(void) {
  return supervisorIsFlying() && !supervisorIsLocked() &&
         !supervisorIsCrashed();
}

static void updateManeuverExtrema(void) {
  minimumAltitude = fminf(minimumAltitude, logGetFloat(altitudeId));
  minimumVoltage = fminf(minimumVoltage, logGetFloat(batteryId));
}

static void sendRateSetpoint(const float pitchRate, const float thrust) {
  setpoint_t setpoint;
  memset(&setpoint, 0, sizeof(setpoint));

  setpoint.mode.x = modeDisable;
  setpoint.mode.y = modeDisable;
  setpoint.mode.z = modeDisable;
  setpoint.mode.roll = modeVelocity;
  setpoint.mode.pitch = modeVelocity;
  setpoint.mode.yaw = modeVelocity;
  setpoint.attitudeRate.pitch = pitchRate;
  setpoint.thrust = clampedThrust(thrust);

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

static void sendPositionSetpoint(const float targetAltitude) {
  setpoint_t setpoint;
  memset(&setpoint, 0, sizeof(setpoint));

  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.mode.z = modeAbs;
  setpoint.mode.roll = modeDisable;
  setpoint.mode.pitch = modeDisable;
  setpoint.mode.yaw = modeVelocity;
  setpoint.position.z = targetAltitude;
  setpoint.velocity_body = true;

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

static bool isUprightWithin(const float toleranceDeg) {
  return fabsf(logGetFloat(rollId)) <= toleranceDeg &&
         fabsf(logGetFloat(pitchId)) <= toleranceDeg;
}

static bool profileIsValid(void) {
  return profileBoostMm >= MIN_BOOST_HEIGHT_MM &&
         profileBoostMm <= MAX_BOOST_HEIGHT_MM &&
         profileBoostMs >= MIN_BOOST_DURATION_MS &&
         profileBoostMs <= MAX_BOOST_DURATION_MS &&
         profileBrakeDeg >= MIN_BRAKE_START_DEG &&
         profileBrakeDeg <= MAX_BRAKE_START_DEG &&
         profileCounterRate >= MIN_COUNTER_RATE_DEG_S &&
         profileCounterRate <= MAX_COUNTER_RATE_DEG_S &&
         profileSpinPct >= MIN_SPIN_THRUST_PERCENT &&
         profileSpinPct <= MAX_SPIN_THRUST_PERCENT &&
         profileCoastPct >= MIN_COAST_THRUST_PERCENT &&
         profileCoastPct <= MAX_COAST_THRUST_PERCENT &&
         profileBrakePct >= MIN_BRAKE_THRUST_PERCENT &&
         profileBrakePct <= MAX_BRAKE_THRUST_PERCENT &&
         profileRecoverMs >= MIN_RECOVERY_DURATION_MS &&
         profileRecoverMs <= MAX_RECOVERY_DURATION_MS;
}

static FlipResult validateStart(void) {
  if (!profileIsValid()) {
    return flipResultProfileInvalid;
  }
  if (!supervisorAllowsManeuver()) {
    return flipResultNotFlying;
  }
  if (logGetFloat(batteryId) < MIN_START_VOLTAGE_V) {
    return flipResultLowBattery;
  }
  if (!isUprightWithin(MAX_START_TILT_DEG)) {
    return flipResultNotLevel;
  }

  const float altitude = logGetFloat(altitudeId);
  const float boostTarget = altitude + (float)profileBoostMm / 1000.0f;
  if (altitude < MIN_START_ALTITUDE_M ||
      altitude > MAX_START_ALTITUDE_M ||
      boostTarget > MAX_MANEUVER_ALTITUDE_M) {
    return flipResultAltitude;
  }
  if (learnedHoverThrust < MIN_HOVER_THRUST ||
      learnedHoverThrust > MAX_HOVER_THRUST) {
    return flipResultNoHoverThrust;
  }
  return flipResultNone;
}

static FlipResult runBoost(void) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = profileBoostMs / LOOP_PERIOD_MS;
  const float targetAltitude =
    startAltitude + (float)profileBoostMm / 1000.0f;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return flipResultSupervisorStopped;
    }
    if (!isUprightWithin(MAX_BOOST_TILT_DEG)) {
      return flipResultNotLevel;
    }
    if (logGetFloat(altitudeId) >= MAX_MANEUVER_ALTITUDE_M) {
      return flipResultAltitude;
    }

    sendPositionSetpoint(targetAltitude);
    updateManeuverExtrema();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  boostGain = logGetFloat(altitudeId) - startAltitude;
  return flipResultNone;
}

static float runRotationStep(
  const float pitchRate,
  const float thrust,
  const float directionSign
) {
  sendRateSetpoint(pitchRate, thrust);
  const float effectivePitchRate = -logGetFloat(pitchRateId);
  const float directedPitchRate = directionSign * effectivePitchRate;
  const float stepSeconds = (float)LOOP_PERIOD_MS / 1000.0f;

  rotationDeg += directedPitchRate * stepSeconds;
  rawRotationDeg += fabsf(effectivePitchRate) * stepSeconds;
  peakPitchRate = fmaxf(peakPitchRate, directedPitchRate);
  updateManeuverExtrema();
  return directedPitchRate;
}

static RotationResult rotateWithBraking(const float hoverThrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const float directionSign = direction >= 0 ? 1.0f : -1.0f;
  const float rateCommand = directionSign * ROTATION_RATE_DEG_S;
  const float finishRateCommand = directionSign * FINISH_RATE_DEG_S;
  const float counterRateCommand =
    -directionSign * (float)profileCounterRate;
  const float spinThrust =
    hoverThrust * (float)profileSpinPct / 100.0f;
  const float coastThrust =
    hoverThrust * (float)profileCoastPct / 100.0f;
  const float brakeThrust =
    hoverThrust * (float)profileBrakePct / 100.0f;
  uint32_t rotationElapsedMs = 0;
  float directedRate = 0.0f;

  rotationDeg = 0.0f;
  rawRotationDeg = 0.0f;
  peakPitchRate = 0.0f;
  brakeStartRotation = 0.0f;
  brakeEndRate = 0.0f;

  flipState = flipStateSpinup;
  while (rotationElapsedMs < SPINUP_TIMEOUT_MS) {
    if (!supervisorAllowsManeuver()) {
      return rotationSupervisorStopped;
    }
    directedRate = runRotationStep(
      rateCommand,
      spinThrust,
      directionSign
    );
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    rotationElapsedMs += LOOP_PERIOD_MS;
    if (directedRate >= SPINUP_READY_RATE_DEG_S && rotationDeg >= 30.0f) {
      break;
    }
  }

  flipState = flipStateCoast;
  while (rotationDeg < (float)profileBrakeDeg) {
    if (!supervisorAllowsManeuver()) {
      return rotationSupervisorStopped;
    }
    if (rotationElapsedMs >= ROTATION_TIMEOUT_MS) {
      return rotationTimedOut;
    }
    directedRate = runRotationStep(
      rateCommand,
      coastThrust,
      directionSign
    );
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    rotationElapsedMs += LOOP_PERIOD_MS;
  }

  flipState = flipStateBrake;
  brakeStartRotation = rotationDeg;
  const uint32_t brakeStartedMs = rotationElapsedMs;
  while (rotationElapsedMs - brakeStartedMs < BRAKE_TIMEOUT_MS) {
    if (!supervisorAllowsManeuver()) {
      return rotationSupervisorStopped;
    }

    float commandedRate = 0.0f;
    if (directedRate > BRAKE_READY_RATE_DEG_S) {
      commandedRate = counterRateCommand;
    } else if (rotationDeg < TARGET_ROTATION_DEG) {
      commandedRate = finishRateCommand;
    }
    directedRate = runRotationStep(
      commandedRate,
      brakeThrust,
      directionSign
    );
    brakeEndRate = directedRate;

    if (rotationDeg >= TARGET_ROTATION_DEG &&
        fabsf(directedRate) <= BRAKE_READY_RATE_DEG_S &&
        isUprightWithin(UPRIGHT_TOLERANCE_DEG)) {
      rotationEndRoll = logGetFloat(rollId);
      rotationEndPitch = logGetFloat(pitchId);
      return rotationCompleted;
    }
    if (rotationDeg > MAX_ROTATION_DEG) {
      rotationEndRoll = logGetFloat(rollId);
      rotationEndPitch = logGetFloat(pitchId);
      return rotationNotUpright;
    }
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    rotationElapsedMs += LOOP_PERIOD_MS;
  }

  rotationEndRoll = logGetFloat(rollId);
  rotationEndPitch = logGetFloat(pitchId);
  return rotationBrakingFailed;
}

static bool runPositionRecovery(void) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = profileRecoverMs / LOOP_PERIOD_MS;
  recoveryStartAltitude = logGetFloat(altitudeId);

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return false;
    }
    sendPositionSetpoint(startAltitude);
    updateManeuverExtrema();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  return isUprightWithin(RECOVERY_LEVEL_TOLERANCE_DEG);
}

static void releaseControl(void) {
  commanderRelaxPriority();
  acroPermitActive = supervisorIsAcroPermitActive() ? 1 : 0;
}

static bool observeRecovery(void) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = RECOVERY_OBSERVATION_MS / LOOP_PERIOD_MS;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return false;
    }
    updateManeuverExtrema();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  return isUprightWithin(UPRIGHT_TOLERANCE_DEG) &&
         logGetFloat(altitudeId) >= MIN_RECOVERY_ALTITUDE_M;
}

static void disarmAndReleaseControl(void) {
  supervisorRequestArming(false);
  motorsStop();
  releaseControl();

  // A stabilizer iteration that started before disarming may still finish.
  // Stop once more after that iteration has had time to leave the motor path.
  vTaskDelay(M2T(2));
  motorsStop();
}

static void failRotationSafely(const FlipResult result) {
  flipResult = result;
  flipState = flipStateAborted;
  disarmAndReleaseControl();
}

static void performFlip(void) {
  flipResult = validateStart();
  if (flipResult != flipResultNone) {
    flipState = flipStateAborted;
    return;
  }

  const float hoverThrust = learnedHoverThrust;
  startAltitude = logGetFloat(altitudeId);
  minimumAltitude = startAltitude;
  minimumVoltage = logGetFloat(batteryId);
  boostGain = 0.0f;
  recoveryStartAltitude = startAltitude;
  rotationEndRoll = 0.0f;
  rotationEndPitch = 0.0f;

  flipState = flipStateBoost;
  flipResult = runBoost();
  if (flipResult != flipResultNone) {
    flipState = flipStateAborted;
    if (flipResult == flipResultNotLevel ||
        flipResult == flipResultSupervisorStopped) {
      disarmAndReleaseControl();
    } else {
      releaseControl();
    }
    return;
  }

  if (!supervisorRequestAcroPermit(ACRO_PERMIT_MS)) {
    flipResult = flipResultPermitDenied;
    flipState = flipStateAborted;
    releaseControl();
    return;
  }
  acroPermitActive = 1;

  const RotationResult rotationResult = rotateWithBraking(hoverThrust);
  if (rotationResult != rotationCompleted) {
    if (rotationResult == rotationSupervisorStopped) {
      failRotationSafely(flipResultSupervisorStopped);
    } else if (rotationResult == rotationNotUpright) {
      failRotationSafely(flipResultNotUpright);
    } else if (rotationResult == rotationBrakingFailed) {
      failRotationSafely(flipResultBrakingFailed);
    } else {
      failRotationSafely(flipResultRotationTimeout);
    }
    return;
  }

  flipState = flipStateRecover;
  const bool recovered = runPositionRecovery();
  if (!recovered) {
    flipResult = supervisorAllowsManeuver() ?
      flipResultRecoveryFailed : flipResultSupervisorStopped;
    flipState = flipStateAborted;
    disarmAndReleaseControl();
    return;
  }
  releaseControl();

  flipState = flipStateVerify;
  if (observeRecovery()) {
    flipResult = flipResultSuccess;
    flipState = flipStateComplete;
  } else {
    flipResult = flipResultRecoveryFailed;
    flipState = flipStateAborted;
    disarmAndReleaseControl();
  }
}

static void verifyImmediateStopPath(void) {
  motorsStop();
  vTaskDelay(M2T(2));
  immediateStopReady =
    motorsGetRatio(MOTOR_M1) == 0 &&
    motorsGetRatio(MOTOR_M2) == 0 &&
    motorsGetRatio(MOTOR_M3) == 0 &&
    motorsGetRatio(MOTOR_M4) == 0;
}

static bool findLogVariables(void) {
  altitudeId = logGetVarId("stateEstimate", "z");
  batteryId = logGetVarId("pm", "vbat");
  pitchId = logGetVarId("stateEstimate", "pitch");
  rollId = logGetVarId("stateEstimate", "roll");
  pitchRateId = logGetVarId("gyro", "y");
  thrustId = logGetVarId("stabilizer", "thrust");

  return logVarIdIsValid(altitudeId) && logVarIdIsValid(batteryId) &&
         logVarIdIsValid(pitchId) && logVarIdIsValid(rollId) &&
         logVarIdIsValid(pitchRateId) && logVarIdIsValid(thrustId);
}

static void updateHoverThrustEstimate(void) {
  if (!supervisorIsFlying() || flipState != flipStateIdle) {
    return;
  }

  const float thrust = logGetFloat(thrustId);
  if (!isUprightWithin(MAX_START_TILT_DEG) ||
      thrust < MIN_HOVER_THRUST || thrust > MAX_HOVER_THRUST) {
    return;
  }

  if (learnedHoverThrust == 0.0f) {
    learnedHoverThrust = thrust;
  } else {
    learnedHoverThrust = 0.98f * learnedHoverThrust + 0.02f * thrust;
  }
}

void appMain(void) {
  vTaskDelay(M2T(3000));
  verifyImmediateStopPath();
  if (!findLogVariables()) {
    DEBUG_PRINT("Required log variables are unavailable\n");
    flipState = flipStateAborted;
    flipResult = flipResultNoHoverThrust;
    while (true) {
      vTaskDelay(M2T(1000));
    }
  }

  DEBUG_PRINT("Acro app version %u ready\n", APP_VERSION);
  while (true) {
    updateHoverThrustEstimate();
    acroPermitActive = supervisorIsAcroPermitActive() ? 1 : 0;

    if (flipRequest != 0 && flipState == flipStateIdle) {
      const uint16_t request = flipRequest;
      flipRequest = 0;
      flipResult = flipResultNone;
      if (request == FLIP_REQUEST_MAGIC) {
        performFlip();
      } else if (request == LIFT_TEST_REQUEST_MAGIC) {
        flipResult = flipResultProfileDisabled;
        flipState = flipStateAborted;
      }
    } else if (flipRequest != 0) {
      flipRequest = 0;
    }

    vTaskDelay(M2T(LOOP_PERIOD_MS));
  }
}

PARAM_GROUP_START(bumblebee)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, version, &firmwareVersion)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, stopReady, &immediateStopReady)
PARAM_ADD(PARAM_UINT16, flipRequest, &flipRequest)
PARAM_ADD(PARAM_INT8, direction, &direction)
PARAM_ADD(PARAM_UINT16, boostMm, &profileBoostMm)
PARAM_ADD(PARAM_UINT16, boostMs, &profileBoostMs)
PARAM_ADD(PARAM_UINT16, brakeDeg, &profileBrakeDeg)
PARAM_ADD(PARAM_UINT16, brakeRate, &profileCounterRate)
PARAM_ADD(PARAM_UINT8, spinPct, &profileSpinPct)
PARAM_ADD(PARAM_UINT8, coastPct, &profileCoastPct)
PARAM_ADD(PARAM_UINT8, brakePct, &profileBrakePct)
PARAM_ADD(PARAM_UINT16, recoverMs, &profileRecoverMs)
PARAM_GROUP_STOP(bumblebee)

LOG_GROUP_START(bumblebee)
LOG_ADD(LOG_UINT8, state, &flipState)
LOG_ADD(LOG_UINT8, result, &flipResult)
LOG_ADD(LOG_FLOAT, rotation, &rotationDeg)
LOG_ADD(LOG_FLOAT, rawRotation, &rawRotationDeg)
LOG_ADD(LOG_FLOAT, peakRate, &peakPitchRate)
LOG_ADD(LOG_FLOAT, endRoll, &rotationEndRoll)
LOG_ADD(LOG_FLOAT, endPitch, &rotationEndPitch)
LOG_ADD(LOG_FLOAT, hoverThrust, &learnedHoverThrust)
LOG_ADD(LOG_FLOAT, startAltitude, &startAltitude)
LOG_ADD(LOG_FLOAT, minAltitude, &minimumAltitude)
LOG_ADD(LOG_FLOAT, minVoltage, &minimumVoltage)
LOG_ADD(LOG_FLOAT, boostGain, &boostGain)
LOG_ADD(LOG_FLOAT, brakeStart, &brakeStartRotation)
LOG_ADD(LOG_FLOAT, brakeEndRate, &brakeEndRate)
LOG_ADD(LOG_FLOAT, recoveryStart, &recoveryStartAltitude)
LOG_ADD(LOG_UINT8, acroPermit, &acroPermitActive)
LOG_GROUP_STOP(bumblebee)
