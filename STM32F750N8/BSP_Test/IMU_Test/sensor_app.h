/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Demo App for Hillcrest BNO080 Sensor Hub
 */

#ifndef SENSOR_APP_H
#define SENSOR_APP_H

#include "../Middlewares/bno080-driver/sh2_hal.h"
#include "../Middlewares/bno080-driver/sh2_err.h"
#include "../Middlewares/bno080-driver/sh2_SensorValue.h"
#include "cmsis_os.h"

#define NUM_BNO080_SIGNALS 5
#define BNO080_SIGNALS_OFFSET 0

typedef enum {
	SENSOR_APP_NOT_INITIALIZED,
	SENSOR_APP_INITIALIZED,
	SENSOR_APP_NOT_READY,
	SESNSO_APP_MAX
} enum_sensor_app_state_t;

void IMUDemoTaskStart(void *params);
void sensor_app_stop(osThreadId_t handle);
int sensor_app_startReport(int sensorId, uint32_t interval_us);
void sensor_app_getAccel (sh2_Accelerometer_t *accel);
void sensor_app_getGyroUncal (sh2_GyroscopeUncalibrated_t *gyro);
void sensor_app_getRawMag (sh2_RawMagnetometer_t *mag);
enum_sensor_app_state_t sensor_app_get_state (void);
void sensor_app_getUncalMag(sh2_MagneticFieldUncalibrated_t *data);
void sensor_app_getCalMag(sh2_MagneticField_t *data);

#endif
