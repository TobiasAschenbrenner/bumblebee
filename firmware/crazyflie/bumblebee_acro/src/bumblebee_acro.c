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
#include "attitude_controller.h"
#include "commander.h"
#include "controller.h"
#include "controller_pid.h"
#include "log.h"
#include "motors.h"
#include "param.h"
#include "platform_defaults.h"
#include "position_controller.h"
#include "supervisor.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "BUMBLEBEE"
#include "debug.h"

#define APP_VERSION 20
#define FLIP_REQUEST_MAGIC 0xB33F
#define LIFT_TEST_REQUEST_MAGIC 0xB11F
#define LOOP_PERIOD_MS 5
#define ACRO_PERMIT_MS 1450

#define MIN_START_ALTITUDE_M 0.75f
#define MAX_START_ALTITUDE_M 1.65f
#define MAX_MANEUVER_ALTITUDE_M 1.70f
#define MIN_DIRECT_CONTROL_ALTITUDE_M 0.12f
#define DIRECT_CONTROL_IMPACT_ACCELERATION_G 4.0f
#define DIRECT_CONTROL_IMPACT_MIN_ROTATION_DEG 60.0f
#define MIN_START_VOLTAGE_V 3.50f
#define MAX_START_TILT_DEG 10.0f
#define MAX_BOOST_TILT_DEG 15.0f
#define MIN_HOVER_THRUST 20000.0f
#define MAX_HOVER_THRUST 55000.0f
#define SPINUP_TIMEOUT_MS 180
#define ROTATION_TIMEOUT_MS 900
#define BRAKE_TIMEOUT_MS 500
#define RECOVERY_OBSERVATION_MS 400
#define RECOVERY_THRUST_FLOOR_MS 200
#define RECOVERY_THRUST_FLOOR_PERCENT 120.0f
#define RAW_RECOVERY_MIN_LIFT_MS 75
#define RAW_RECOVERY_MAX_LIFT_MS 150
#define RAW_RECOVERY_LIFT_PERCENT 120.0f
#define SPINUP_READY_RATE_DEG_S 450.0f
#define SPINUP_READY_ROTATION_DEG 10.0f
#define BRAKE_READY_RATE_DEG_S 180.0f
#define TARGET_ROTATION_DEG 335.0f
#define MIN_RECOVERY_RAW_ROTATION_DEG 320.0f
#define MAX_RECOVERY_RAW_ROTATION_DEG 380.0f
#define MAX_ROTATION_DEG 410.0f
#define UPRIGHT_TOLERANCE_DEG 25.0f
#define RAW_RECOVERY_UPRIGHT_TOLERANCE_DEG 60.0f
#define RECOVERY_LEVEL_TOLERANCE_DEG 20.0f
#define MIN_RECOVERY_ALTITUDE_M 0.55f

#define MIN_BOOST_HEIGHT_MM 100
#define MAX_BOOST_HEIGHT_MM 300
#define MIN_BOOST_DURATION_MS 150
#define MAX_BOOST_DURATION_MS 350
#define MIN_BRAKE_START_DEG 100
#define MAX_BRAKE_START_DEG 330
#define MIN_SPIN_THRUST_PERCENT 25
#define MAX_SPIN_THRUST_PERCENT 55
#define MIN_COAST_THRUST_PERCENT 15
#define MAX_COAST_THRUST_PERCENT 35
#define MIN_BRAKE_THRUST_PERCENT 30
#define MAX_BRAKE_THRUST_PERCENT 80
#define MIN_TORQUE_AUTHORITY_PERCENT 50
#define MAX_TORQUE_AUTHORITY_PERCENT 95
#define MIN_RECOVERY_DURATION_MS 250
#define MAX_RECOVERY_DURATION_MS 500

#define DIRECT_MOTOR_MAX_PWM 60000.0f
#define PWM_TO_THRUST_A 0.091492681f
#define PWM_TO_THRUST_B 0.067673604f
#define QUAD_X_ARM_SCALE 0.707106781f

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
  flipResultAltitudeCutoff = 15,
  flipResultPermitExpired = 16,
  flipResultImpact = 17,
} FlipResult;

typedef enum {
  directAbortNone = 0,
  directAbortNotFlying = 1,
  directAbortLocked = 2,
  directAbortCrashed = 3,
  directAbortImpact = 4,
  directAbortPermitExpired = 5,
} DirectAbortReason;

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
static uint16_t profileBrakeDeg = 140;
static uint8_t profileSpinPct = 40;
static uint8_t profileCoastPct = 20;
static uint8_t profileBrakePct = 70;
static uint8_t profileSpinTorquePct = 85;
static uint8_t profileBrakeTorquePct = 95;
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
static volatile bool directControlActive;
static volatile bool resetPidAfterDirect;
static volatile bool recoveryBoostActive;
static volatile uint8_t directControlAbort;
static volatile float directCollectiveForce;
static volatile float directPitchTorque;
static volatile float directControlRotationDeg;
static uint8_t recoveryBoostUsed;
static uint8_t rawRecoveryPathUsed;

static logVarId_t altitudeId;
static logVarId_t batteryId;
static logVarId_t pitchId;
static logVarId_t rollId;
static logVarId_t pitchRateId;
static logVarId_t thrustId;

static bool supervisorAllowsManeuver(void) {
  return supervisorIsFlying() && !supervisorIsLocked() &&
         !supervisorIsCrashed();
}

static bool impactDetected(const sensorData_t *sensors) {
  if (directControlRotationDeg < DIRECT_CONTROL_IMPACT_MIN_ROTATION_DEG) {
    return false;
  }
  const float accelerationMagnitude = sqrtf(
    sensors->acc.x * sensors->acc.x +
    sensors->acc.y * sensors->acc.y +
    sensors->acc.z * sensors->acc.z
  );
  return accelerationMagnitude >= DIRECT_CONTROL_IMPACT_ACCELERATION_G;
}

static DirectAbortReason directAbortReason(const sensorData_t *sensors) {
  if (supervisorIsLocked()) {
    return directAbortLocked;
  }
  if (supervisorIsCrashed()) {
    return directAbortCrashed;
  }
  if (!supervisorIsFlying()) {
    return directAbortNotFlying;
  }
  if (!supervisorIsAcroPermitActive()) {
    return directAbortPermitExpired;
  }

  if (impactDetected(sensors)) {
    return directAbortImpact;
  }
  return directAbortNone;
}

static void updateManeuverExtrema(void) {
  minimumAltitude = fminf(minimumAltitude, logGetFloat(altitudeId));
  minimumVoltage = fminf(minimumVoltage, logGetFloat(batteryId));
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

static float motorForceFromPwm(const float pwm) {
  const float normalizedPwm = pwm / 65535.0f;
  return PWM_TO_THRUST_A * normalizedPwm * normalizedPwm +
         PWM_TO_THRUST_B * normalizedPwm;
}

static float hoverForceFromPwm(const float hoverPwm) {
  return 4.0f * motorForceFromPwm(hoverPwm);
}

static float maximumPitchTorque(const float collectiveForce) {
  const float arm = QUAD_X_ARM_SCALE * ARM_LENGTH;
  const float maximumCollectiveForce =
    4.0f * motorForceFromPwm(DIRECT_MOTOR_MAX_PWM);
  const float lowerMotorLimit = arm * collectiveForce;
  const float upperMotorLimit =
    arm * fmaxf(0.0f, maximumCollectiveForce - collectiveForce);
  return fmaxf(0.0f, fminf(lowerMotorLimit, upperMotorLimit));
}

static void setDirectControl(
  const float collectiveForce,
  const float pitchTorque
) {
  directCollectiveForce = collectiveForce;
  directPitchTorque = pitchTorque;
}

static void startDirectControl(void) {
  directControlAbort = 0;
  directControlRotationDeg = 0.0f;
  resetPidAfterDirect = false;
  directControlActive = true;
}

static void stopDirectControl(void) {
  directControlActive = false;
  directCollectiveForce = 0.0f;
  directPitchTorque = 0.0f;
  resetPidAfterDirect = true;
}

void controllerOutOfTreeInit(void) {
  controllerPidInit();
}

bool controllerOutOfTreeTest(void) {
  return controllerPidTest();
}

void controllerOutOfTree(
  control_t *control,
  const setpoint_t *setpoint,
  const sensorData_t *sensors,
  const state_t *state,
  const uint32_t tick
) {
  if (directControlActive) {
    const DirectAbortReason abortReason = directAbortReason(sensors);
    if (abortReason != directAbortNone) {
      directControlAbort = (uint8_t)abortReason;
      stopDirectControl();
      memset(control, 0, sizeof(*control));
      control->controlMode = controlModeForceTorque;
      return;
    }

    control->controlMode = controlModeForceTorque;
    control->thrustSi = directCollectiveForce;
    control->torqueX = 0.0f;
    control->torqueY = directPitchTorque;
    control->torqueZ = 0.0f;
    return;
  }

  if (recoveryBoostActive && impactDetected(sensors)) {
    directControlAbort = directAbortImpact;
    recoveryBoostActive = false;
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeForceTorque;
    return;
  }

  // A direct-control safety abort remains latched until the next power cycle.
  // This prevents one PID iteration from restoring thrust before appMain has
  // observed the abort and completed the supervisor disarm.
  if (directControlAbort != 0) {
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeForceTorque;
    return;
  }

  if (resetPidAfterDirect) {
    attitudeControllerResetAllPID(
      state->attitude.roll,
      state->attitude.pitch,
      state->attitude.yaw
    );
    positionControllerResetAllPID(
      state->position.x,
      state->position.y,
      state->position.z
    );
    resetPidAfterDirect = false;
  }
  controllerPid(control, setpoint, sensors, state, tick);
  if (recoveryBoostActive && control->controlMode == controlModeLegacy) {
    const float thrustFloor = fminf(
      DIRECT_MOTOR_MAX_PWM,
      learnedHoverThrust * RECOVERY_THRUST_FLOOR_PERCENT / 100.0f
    );
    control->thrust = fmaxf(control->thrust, thrustFloor);
  }
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
         profileSpinPct >= MIN_SPIN_THRUST_PERCENT &&
         profileSpinPct <= MAX_SPIN_THRUST_PERCENT &&
         profileCoastPct >= MIN_COAST_THRUST_PERCENT &&
         profileCoastPct <= MAX_COAST_THRUST_PERCENT &&
         profileBrakePct >= MIN_BRAKE_THRUST_PERCENT &&
         profileBrakePct <= MAX_BRAKE_THRUST_PERCENT &&
         profileSpinTorquePct >= MIN_TORQUE_AUTHORITY_PERCENT &&
         profileSpinTorquePct <= MAX_TORQUE_AUTHORITY_PERCENT &&
         profileBrakeTorquePct >= MIN_TORQUE_AUTHORITY_PERCENT &&
         profileBrakeTorquePct <= MAX_TORQUE_AUTHORITY_PERCENT &&
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
  const float collectiveForce,
  const float pitchTorque,
  const float directionSign
) {
  setDirectControl(collectiveForce, pitchTorque);
  const float effectivePitchRate = -logGetFloat(pitchRateId);
  const float directedPitchRate = directionSign * effectivePitchRate;
  const float stepSeconds = (float)LOOP_PERIOD_MS / 1000.0f;

  rotationDeg += directedPitchRate * stepSeconds;
  rawRotationDeg += fabsf(effectivePitchRate) * stepSeconds;
  directControlRotationDeg = rawRotationDeg;
  peakPitchRate = fmaxf(peakPitchRate, directedPitchRate);
  updateManeuverExtrema();
  return directedPitchRate;
}

static RotationResult rotateWithBraking(const float hoverThrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const float directionSign = direction >= 0 ? 1.0f : -1.0f;
  const float hoverForce = hoverForceFromPwm(hoverThrust);
  const float spinForce =
    hoverForce * (float)profileSpinPct / 100.0f;
  const float coastForce =
    hoverForce * (float)profileCoastPct / 100.0f;
  const float brakeForce =
    hoverForce * (float)profileBrakePct / 100.0f;
  const float spinTorque =
    -directionSign * maximumPitchTorque(spinForce) *
    (float)profileSpinTorquePct / 100.0f;
  const float brakeTorque =
    directionSign * maximumPitchTorque(brakeForce) *
    (float)profileBrakeTorquePct / 100.0f;
  uint32_t rotationElapsedMs = 0;
  float directedRate = 0.0f;

  rotationDeg = 0.0f;
  rawRotationDeg = 0.0f;
  peakPitchRate = 0.0f;
  brakeStartRotation = 0.0f;
  brakeEndRate = 0.0f;
  startDirectControl();

  flipState = flipStateSpinup;
  while (rotationElapsedMs < SPINUP_TIMEOUT_MS) {
    if (!supervisorAllowsManeuver() || directControlAbort != 0) {
      return rotationSupervisorStopped;
    }
    directedRate = runRotationStep(
      spinForce,
      spinTorque,
      directionSign
    );
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    rotationElapsedMs += LOOP_PERIOD_MS;
    if (directedRate >= SPINUP_READY_RATE_DEG_S &&
        rotationDeg >= SPINUP_READY_ROTATION_DEG) {
      break;
    }
  }

  flipState = flipStateCoast;
  while (rotationDeg < (float)profileBrakeDeg) {
    if (!supervisorAllowsManeuver() || directControlAbort != 0) {
      return rotationSupervisorStopped;
    }
    if (rotationElapsedMs >= ROTATION_TIMEOUT_MS) {
      return rotationTimedOut;
    }
    directedRate = runRotationStep(
      coastForce,
      0.0f,
      directionSign
    );
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    rotationElapsedMs += LOOP_PERIOD_MS;
  }

  flipState = flipStateBrake;
  brakeStartRotation = rotationDeg;
  const uint32_t brakeStartedMs = rotationElapsedMs;
  while (rotationElapsedMs - brakeStartedMs < BRAKE_TIMEOUT_MS) {
    if (!supervisorAllowsManeuver() || directControlAbort != 0) {
      return rotationSupervisorStopped;
    }

    float commandedTorque = 0.0f;
    if (directedRate > BRAKE_READY_RATE_DEG_S) {
      commandedTorque = brakeTorque;
    }
    directedRate = runRotationStep(
      brakeForce,
      commandedTorque,
      directionSign
    );
    brakeEndRate = directedRate;

    const bool signedRotationReady =
      rotationDeg >= TARGET_ROTATION_DEG &&
      isUprightWithin(UPRIGHT_TOLERANCE_DEG);
    const bool rawRotationReady =
      rawRotationDeg >= MIN_RECOVERY_RAW_ROTATION_DEG &&
      rawRotationDeg <= MAX_RECOVERY_RAW_ROTATION_DEG;
    if ((signedRotationReady || rawRotationReady) &&
        fabsf(directedRate) <= BRAKE_READY_RATE_DEG_S) {
      rawRecoveryPathUsed = !signedRotationReady && rawRotationReady;
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

static bool runRawRecoveryLift(const float hoverThrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = RAW_RECOVERY_MAX_LIFT_MS / LOOP_PERIOD_MS;
  const float liftForce =
    hoverForceFromPwm(hoverThrust) * RAW_RECOVERY_LIFT_PERCENT / 100.0f;

  recoveryBoostUsed = 1;
  setDirectControl(liftForce, 0.0f);
  for (uint32_t i = 0; i < iterations; ++i) {
    if (directControlAbort != directAbortNone ||
        !supervisorAllowsManeuver()) {
      return false;
    }
    updateManeuverExtrema();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
    const uint32_t elapsedMs = (i + 1) * LOOP_PERIOD_MS;
    if (elapsedMs >= RAW_RECOVERY_MIN_LIFT_MS &&
        isUprightWithin(RAW_RECOVERY_UPRIGHT_TOLERANCE_DEG)) {
      return true;
    }
  }

  return false;
}

static bool runPositionRecovery(void) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = profileRecoverMs / LOOP_PERIOD_MS;
  recoveryStartAltitude = logGetFloat(altitudeId);
  recoveryBoostActive = true;
  recoveryBoostUsed = 1;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (directControlAbort != directAbortNone ||
        !supervisorAllowsManeuver() ||
        logGetFloat(altitudeId) < MIN_DIRECT_CONTROL_ALTITUDE_M ||
        !isUprightWithin(60.0f)) {
      recoveryBoostActive = false;
      return false;
    }
    if (i * LOOP_PERIOD_MS >= RECOVERY_THRUST_FLOOR_MS) {
      recoveryBoostActive = false;
    }
    sendPositionSetpoint(startAltitude);
    updateManeuverExtrema();
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }

  recoveryBoostActive = false;
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
  recoveryBoostActive = false;
  stopDirectControl();
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
  recoveryBoostActive = false;
  recoveryBoostUsed = 0;
  rawRecoveryPathUsed = 0;

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
    stopDirectControl();
    if (rotationResult == rotationSupervisorStopped) {
      if (directControlAbort == directAbortPermitExpired) {
        failRotationSafely(flipResultPermitExpired);
      } else if (directControlAbort == directAbortImpact) {
        failRotationSafely(flipResultImpact);
      } else {
        failRotationSafely(flipResultSupervisorStopped);
      }
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
  if (rawRecoveryPathUsed != 0 && !runRawRecoveryLift(hoverThrust)) {
    stopDirectControl();
    flipResult = directControlAbort == directAbortImpact ?
      flipResultImpact : flipResultRecoveryFailed;
    flipState = flipStateAborted;
    disarmAndReleaseControl();
    return;
  }
  stopDirectControl();
  const bool recovered = runPositionRecovery();
  if (!recovered) {
    if (directControlAbort == directAbortImpact) {
      flipResult = flipResultImpact;
    } else {
      flipResult = supervisorAllowsManeuver() ?
        flipResultRecoveryFailed : flipResultSupervisorStopped;
    }
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
PARAM_ADD(PARAM_UINT8, spinPct, &profileSpinPct)
PARAM_ADD(PARAM_UINT8, coastPct, &profileCoastPct)
PARAM_ADD(PARAM_UINT8, brakePct, &profileBrakePct)
PARAM_ADD(PARAM_UINT8, spinTorque, &profileSpinTorquePct)
PARAM_ADD(PARAM_UINT8, brakeTorque, &profileBrakeTorquePct)
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
LOG_ADD(LOG_UINT8, directAbort, &directControlAbort)
LOG_ADD(LOG_UINT8, recoveryBoost, &recoveryBoostUsed)
LOG_ADD(LOG_UINT8, rawRecovery, &rawRecoveryPathUsed)
LOG_GROUP_STOP(bumblebee)
