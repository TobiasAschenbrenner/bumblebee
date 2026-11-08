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

#define APP_VERSION 1
#define FLIP_REQUEST_MAGIC 0xB33F
#define LOOP_PERIOD_MS 5
#define ACRO_PERMIT_MS 1450

#define MIN_START_ALTITUDE_M 0.75f
#define MAX_START_ALTITUDE_M 1.25f
#define MIN_START_VOLTAGE_V 3.75f
#define MAX_START_TILT_DEG 10.0f
#define MIN_HOVER_THRUST 20000.0f
#define MAX_HOVER_THRUST 55000.0f
#define MAX_THRUST 60000.0f

#define BOOST_DURATION_MS 160
#define ROTATION_TIMEOUT_MS 600
#define RECOVERY_DURATION_MS 220
#define LEVEL_DURATION_MS 250
#define TARGET_ROTATION_DEG 340.0f
#define PITCH_RATE_DEG_S 900.0f
#define BOOST_THRUST_FACTOR 1.40f
#define ROTATION_THRUST_FACTOR 0.22f
#define RECOVERY_THRUST_FACTOR 1.30f

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
} FlipResult;

static const uint8_t firmwareVersion = APP_VERSION;
static uint16_t flipRequest;
static int8_t direction = 1;

static uint8_t flipState;
static uint8_t flipResult;
static float rotationDeg;
static float learnedHoverThrust;
static uint8_t acroPermitActive;

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
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }
  return true;
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

static bool rotate(const float hoverThrust) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const uint32_t iterations = ROTATION_TIMEOUT_MS / LOOP_PERIOD_MS;
  const float rateCommand = (direction >= 0 ? 1.0f : -1.0f) *
                            PITCH_RATE_DEG_S;
  const float rotationThrust = hoverThrust * ROTATION_THRUST_FACTOR;
  rotationDeg = 0.0f;

  for (uint32_t i = 0; i < iterations; ++i) {
    if (!supervisorAllowsManeuver()) {
      return false;
    }

    sendRateSetpoint(rateCommand, rotationThrust);
    rotationDeg += fabsf(logGetFloat(pitchRateId)) *
                   ((float)LOOP_PERIOD_MS / 1000.0f);
    if (rotationDeg >= TARGET_ROTATION_DEG) {
      return true;
    }
    vTaskDelayUntil(&lastWakeTime, M2T(LOOP_PERIOD_MS));
  }
  return false;
}

static void releaseControl(void) {
  commanderRelaxPriority();
  acroPermitActive = supervisorIsAcroPermitActive() ? 1 : 0;
}

static void performFlip(void) {
  flipResult = validateStart();
  if (flipResult != flipResultNone) {
    flipState = flipStateAborted;
    return;
  }

  const float hoverThrust = learnedHoverThrust;
  flipState = flipStateBoost;
  if (!holdRateFor(
        BOOST_DURATION_MS,
        0.0f,
        hoverThrust * BOOST_THRUST_FACTOR
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
  const bool completedRotation = rotate(hoverThrust);

  flipState = flipStateRecover;
  const bool recovered = holdRateFor(
    RECOVERY_DURATION_MS,
    0.0f,
    hoverThrust * RECOVERY_THRUST_FACTOR
  );

  flipState = flipStateLevel;
  const bool leveled = holdLevelFor(LEVEL_DURATION_MS, hoverThrust);
  releaseControl();

  if (!recovered || !leveled) {
    flipResult = flipResultSupervisorStopped;
    flipState = flipStateAborted;
  } else if (!completedRotation) {
    flipResult = flipResultRotationTimeout;
    flipState = flipStateAborted;
  } else {
    flipResult = flipResultSuccess;
    flipState = flipStateComplete;
  }
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

    if ((flipState == flipStateComplete || flipState == flipStateAborted) &&
        !supervisorIsFlying()) {
      flipState = flipStateIdle;
      flipResult = flipResultNone;
      rotationDeg = 0.0f;
      learnedHoverThrust = 0.0f;
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
LOG_ADD(LOG_FLOAT, hoverThrust, &learnedHoverThrust)
LOG_ADD(LOG_UINT8, acroPermit, &acroPermitActive)
LOG_GROUP_STOP(bumblebee)
