/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l1x.c: Time-of-flight distance sensor driver
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "i2cdev.h"
#include "zranger2.h"
#include <vl53lx_api.h>
#include <vl53lx_platform.h>
#include "cf_math.h"
#define DEBUG_MODULE "ZR2"
#include "debug_cf.h"
#include "deck_digital.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f; // STD at elevation expPointB [m]
static float expCoeff;

#define INT_BOTTOM 6
#define XSHUT_BOTTOM 7
#define INT_FRONT 8
#define XSHUT_FRONT 9
#define USER_A 0

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static int16_t range_last = 0;

static bool isInit;

static VL53LX_Dev_t dev;

static uint16_t zRanger2GetMeasurementAndRestart(VL53LX_Dev_t *dev)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  VL53LX_MultiRangingData_t rangingData;
  uint8_t dataReady = 0;
  int16_t range;
  int16_t range_min;
  int16_t range_max;
  int16_t range_ave;
  uint8_t count;

  while (dataReady == 0)
  {
    status = VL53LX_GetMeasurementDataReady(dev, &dataReady);
    vTaskDelay(M2T(1));
  }

  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

  // uint32_t start_time = micros();
  VL53LX_GetMultiRangingData(dev, pMultiRangingData);
  // uint32_t end_time = micros();
  // USBSerial.printf("ToF Time%f\n", (float)(end_time - start_time)*1.0e-6);
  uint8_t no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
  // USBSerial.printf("Total N=%d ",no_of_object_found);
  range_min = 10000;
  range_max = 0;
  range_ave = 0;
  if (no_of_object_found == 0)
  {
    range_min = 9999;
    range_max = 0;
  }
  else
  {
    count = 0;
    for (uint8_t j = 0; j < no_of_object_found; j++)
    {
      if (MultiRangingData.RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID)
      {
        count++;
        range = MultiRangingData.RangeData[j].RangeMilliMeter;
        if (range_min > range)
          range_min = range;
        if (range_max < range)
          range_max = range;
        range_ave = range_ave + range;
      }
    }
    if (count != 0)
      range_ave = range_ave / count;
  }

  VL53LX_StopMeasurement(dev);
  status = VL53LX_StartMeasurement(dev);
  status = status;

  return range_max;
}

void zRanger2Init(void)
{
  if (isInit)
    return;

  pinMode(XSHUT_BOTTOM, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(INT_BOTTOM, INPUT);
  pinMode(INT_FRONT, INPUT);
  pinMode(USER_A, INPUT_PULLUP);

  // ToF Disable
  digitalWrite(XSHUT_BOTTOM, LOW);
  digitalWrite(XSHUT_FRONT, LOW);

  // Front ToF I2C address to 0x54
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  delay(100);
  digitalWrite(XSHUT_BOTTOM, HIGH);

  if (vl53l3xInit(&dev, I2C0_DEV))
  {
    DEBUG_PRINTI("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINTW("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void *arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53LX_StopMeasurement(&dev);
  VL53LX_SetDistanceMode(&dev, VL53LX_DISTANCEMODE_MEDIUM);
  VL53LX_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53LX_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    range_last = zRanger2GetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT)
    {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f + expf(expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static uint8_t disable = 0;
#define PARAM_CORE (1 << 5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &disable)
PARAM_GROUP_STOP(deck)

static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)
