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
#include "param.h"
#include "supervisor.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "BUMBLEBEE"
#include "debug.h"

#define APP_VERSION 4
#define FLIP_REQUEST_MAGIC 0xB33F
#define LOOP_PERIOD_MS 5
#define ACRO_PERMIT_MS 1450

#define MIN_START_ALTITUDE_M 0.75f
#define MAX_START_ALTITUDE_M 1.25f
#define MIN_START_VOLTAGE_V 3.50f
#define MAX_START_TILT_DEG 10.0f
#define MIN_HOVER_THRUST 20000.0f
#define MAX_HOVER_THRUST 55000.0f
#define MAX_THRUST 60000.0f

#define BOOST_DURATION_MS 280
#define ROTATION_TIMEOUT_MS 850
#define LEVEL_RECOVERY_TIMEOUT_MS 300
#define RECOVERY_OBSERVATION_MS 400
#define TARGET_ROTATION_DEG 340.0f
#define UPRIGHT_TOLERANCE_DEG 35.0f
#define LEVEL_RELEASE_TOLERANCE_DEG 15.0f
#define MIN_MANEUVER_ALTITUDE_M 0.25f
#define MIN_RECOVERY_ALTITUDE_M 0.55f
#define MIN_RECOVERY_VERTICAL_SPEED_M_S -0.50f
#define PITCH_RATE_DEG_S 900.0f
#define ROTATION_THRUST_FACTOR 0.22f

typedef enum {
  flipStateIdle = 0,
  flipStateBoost = 1,
  flipStateRotate = 2,
  flipStateRecover = 3,
  flipStateLevel = 4,
  flipStateComplete = 5,
  flipStateAborted = 6,
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
} FlipResult;

typedef enum {
  rotationCompleted = 0,
  rotationTimedOut = 1,
  rotationSupervisorStopped = 2,
  rotationNotUpright = 3,
} RotationResult;

static const uint8_t firmwareVersion = APP_VERSION;
static uint16_t flipRequest;
static int8_t direction = 1;

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
static float recoveryAltitude;
static float recoveryVerticalSpeed;
static uint8_t acroPermitActive;

static logVarId_t altitudeId;
static logVarId_t batteryId;
static logVarId_t pitchId;
static logVarId_t rollId;
static logVarId_t pitchRateId;
static logVarId_t thrustId;
static logVarId_t verticalSpeedId;

static float clampedThrust(const float thrust) {
  return fminf(MAX_THRUST, fmaxf(0.0f, thrust));
}

static bool supervisorAllowsManeuver(void) {
  return supervisorIsFlying() && !supervisorIsLocked() &&
         !supervisorIsCrashed();
}

static void updateMinimumAltitude(void) {
  minimumAltitude = fminf(minimumAltitude, logGetFloat(altitudeId));
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

static void sendLevelSetpoint(const float thrust) {
  setpoint_t setpoint;
  memset(&setpoint, 0, sizeof(setpoint));

  setpoint.mode.x = modeDisable;
  setpoint.mode.y = modeDisable;
  setpoint.mode.z = modeDisable;
  setpoint.mode.roll = modeAbs;
  setpoint.mode.pitch = modeAbs;
  setpoint.mode.yaw = modeVelocity;
  setpoint.thrust = clampedThrust(thrust);

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

static bool holdRateFor(
  const uint32_t durationMs,
  const float pitchRate,
  const float thrust
) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = durationMs / LOOP_PERIOD_MS;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return false;
    }
    sendRateSetpoint(pitchRate, thrust);
    updateMinimumAltitude();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }
  return true;
}

static bool holdLevelFor(const uint32_t durationMs, const float thrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = durationMs / LOOP_PERIOD_MS;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return false;
    }
    sendLevelSetpoint(thrust);
    updateMinimumAltitude();
    if (fabsf(logGetFloat(rollId)) <= LEVEL_RELEASE_TOLERANCE_DEG &&
        fabsf(logGetFloat(pitchId)) <= LEVEL_RELEASE_TOLERANCE_DEG) {
      return true;
    }
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }
  return false;
}

static FlipResult validateStart(void) {
  if (!supervisorAllowsManeuver()) {
    return flipResultNotFlying;
  }

  const float voltage = logGetFloat(batteryId);
  if (voltage < MIN_START_VOLTAGE_V) {
    return flipResultLowBattery;
  }

  const float roll = logGetFloat(rollId);
  const float pitch = logGetFloat(pitchId);
  if (fabsf(roll) > MAX_START_TILT_DEG ||
      fabsf(pitch) > MAX_START_TILT_DEG) {
    return flipResultNotLevel;
  }

  const float altitude = logGetFloat(altitudeId);
  if (altitude < MIN_START_ALTITUDE_M ||
      altitude > MAX_START_ALTITUDE_M) {
    return flipResultAltitude;
  }

  if (learnedHoverThrust < MIN_HOVER_THRUST ||
      learnedHoverThrust > MAX_HOVER_THRUST) {
    return flipResultNoHoverThrust;
  }

  return flipResultNone;
}

static bool isUpright(void) {
  return fabsf(logGetFloat(rollId)) <= UPRIGHT_TOLERANCE_DEG &&
         fabsf(logGetFloat(pitchId)) <= UPRIGHT_TOLERANCE_DEG;
}

static void captureRotationEndAttitude(void) {
  rotationEndRoll = logGetFloat(rollId);
  rotationEndPitch = logGetFloat(pitchId);
}

static RotationResult rotate(const float hoverThrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = ROTATION_TIMEOUT_MS / LOOP_PERIOD_MS;
  const float directionSign = direction >= 0 ? 1.0f : -1.0f;
  const float rateCommand = directionSign * PITCH_RATE_DEG_S;
  const float rotationThrust = hoverThrust * ROTATION_THRUST_FACTOR;
  rotationDeg = 0.0f;
  rawRotationDeg = 0.0f;
  peakPitchRate = 0.0f;
  rotationEndRoll = 0.0f;
  rotationEndPitch = 0.0f;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      captureRotationEndAttitude();
      return rotationSupervisorStopped;
    }

    sendRateSetpoint(rateCommand, rotationThrust);
    updateMinimumAltitude();
    const float effectivePitchRate = -logGetFloat(pitchRateId);
    const float directedPitchRate = directionSign * effectivePitchRate;
    const float stepSeconds = (float)LOOP_PERIOD_MS / 1000.0f;
    rotationDeg += directedPitchRate * stepSeconds;
    rawRotationDeg += fabsf(effectivePitchRate) * stepSeconds;
    peakPitchRate = fmaxf(peakPitchRate, directedPitchRate);

    if (rotationDeg >= TARGET_ROTATION_DEG && isUpright()) {
      captureRotationEndAttitude();
      return rotationCompleted;
    }
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  captureRotationEndAttitude();
  return rotationDeg >= TARGET_ROTATION_DEG ?
    rotationNotUpright : rotationTimedOut;
}

static void releaseControl(void) {
  commanderRelaxPriority();
  acroPermitActive = supervisorIsAcroPermitActive() ? 1 : 0;
}

static bool observeRecovery(void) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = RECOVERY_OBSERVATION_MS / LOOP_PERIOD_MS;

  for (uint32_t i = 0; i < iterations; ++i) {
    updateMinimumAltitude();
    recoveryAltitude = logGetFloat(altitudeId);
    recoveryVerticalSpeed = logGetFloat(verticalSpeedId);
    if (!supervisorAllowsManeuver()) {
      return false;
    }
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  return isUpright() && minimumAltitude >= MIN_MANEUVER_ALTITUDE_M &&
         recoveryAltitude >= MIN_RECOVERY_ALTITUDE_M &&
         recoveryVerticalSpeed >= MIN_RECOVERY_VERTICAL_SPEED_M_S;
}

static void failRotationSafely(const FlipResult result) {
  flipResult = result;
  flipState = flipStateAborted;

  // Keep zero thrust at app priority until the temporary permit expires.
  // This prevents the position controller from applying recovery thrust while
  // the Crazyflie may still be upside down.
  holdRateFor(ACRO_PERMIT_MS, 0.0f, 0.0f);
  releaseControl();
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
  recoveryAltitude = startAltitude;
  recoveryVerticalSpeed = 0.0f;
  flipState = flipStateBoost;
  if (!holdRateFor(
        BOOST_DURATION_MS,
        0.0f,
        MAX_THRUST
      )) {
    flipResult = flipResultSupervisorStopped;
    flipState = flipStateAborted;
    releaseControl();
    return;
  }

  if (!supervisorRequestAcroPermit(ACRO_PERMIT_MS)) {
    flipResult = flipResultPermitDenied;
    flipState = flipStateAborted;
    releaseControl();
    return;
  }
  acroPermitActive = 1;

  flipState = flipStateRotate;
  const RotationResult rotationResult = rotate(hoverThrust);
  if (rotationResult != rotationCompleted) {
    if (rotationResult == rotationSupervisorStopped) {
      failRotationSafely(flipResultSupervisorStopped);
    } else if (rotationResult == rotationNotUpright) {
      failRotationSafely(flipResultNotUpright);
    } else {
      failRotationSafely(flipResultRotationTimeout);
    }
    return;
  }

  flipState = flipStateRecover;
  const bool leveled = holdLevelFor(LEVEL_RECOVERY_TIMEOUT_MS, MAX_THRUST);
  releaseControl();

  if (!leveled) {
    recoveryAltitude = logGetFloat(altitudeId);
    recoveryVerticalSpeed = logGetFloat(verticalSpeedId);
    flipResult = supervisorAllowsManeuver() ?
      flipResultRecoveryFailed : flipResultSupervisorStopped;
    flipState = flipStateAborted;
    return;
  }

  flipState = flipStateLevel;
  if (observeRecovery()) {
    flipResult = flipResultSuccess;
    flipState = flipStateComplete;
  } else {
    flipResult = flipResultRecoveryFailed;
    flipState = flipStateAborted;
  }
}

static bool findLogVariables(void) {
  altitudeId = logGetVarId("stateEstimate", "z");
  batteryId = logGetVarId("pm", "vbat");
  pitchId = logGetVarId("stateEstimate", "pitch");
  rollId = logGetVarId("stateEstimate", "roll");
  pitchRateId = logGetVarId("gyro", "y");
  thrustId = logGetVarId("stabilizer", "thrust");
  verticalSpeedId = logGetVarId("stateEstimate", "vz");

  return logVarIdIsValid(altitudeId) && logVarIdIsValid(batteryId) &&
         logVarIdIsValid(pitchId) && logVarIdIsValid(rollId) &&
         logVarIdIsValid(pitchRateId) && logVarIdIsValid(thrustId) &&
         logVarIdIsValid(verticalSpeedId);
}

static void updateHoverThrustEstimate(void) {
  if (!supervisorIsFlying() || flipState != flipStateIdle) {
    return;
  }

  const float roll = logGetFloat(rollId);
  const float pitch = logGetFloat(pitchId);
  const float thrust = logGetFloat(thrustId);
  if (fabsf(roll) > MAX_START_TILT_DEG ||
      fabsf(pitch) > MAX_START_TILT_DEG ||
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

    if (flipRequest == FLIP_REQUEST_MAGIC && flipState == flipStateIdle) {
      flipRequest = 0;
      flipResult = flipResultNone;
      performFlip();
    } else if (flipRequest != 0) {
      flipRequest = 0;
    }

    vTaskDelay(M2T(LOOP_PERIOD_MS));
  }
}

PARAM_GROUP_START(bumblebee)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, version, &firmwareVersion)
PARAM_ADD(PARAM_UINT16, flipRequest, &flipRequest)
PARAM_ADD(PARAM_INT8, direction, &direction)
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
LOG_ADD(LOG_FLOAT, recoveryAltitude, &recoveryAltitude)
LOG_ADD(LOG_FLOAT, recoveryVz, &recoveryVerticalSpeed)
LOG_ADD(LOG_UINT8, acroPermit, &acroPermitActive)
LOG_GROUP_STOP(bumblebee)
