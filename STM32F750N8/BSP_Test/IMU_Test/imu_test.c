/*******************************************************************************
 * @file imu_test.c
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/
/**
 *=============================================================================
 *                        ##### IMU - Test Execution #####
 *=============================================================================
 * Entry:	[24] [XX] [XX] [XX] [XX]
 *
 * Output:	[24] [00] [Resolution] [0X] [XX]
 *	- Resolution: 6, 8, 10 or 12
 *	- 0X:	4 MSB bits (Zero-ed if resolution < 10)
 *	- XX:	8 LSB bits
 *
 ******************************************************************************/
#include <cmsis_os2.h>
#include "imu_test.h"

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"

/* GPIO BSP Header */
#include "GPIO/bsp_gpio.h"

/* ADC BSP Header */
#include "IMU/bsp_imu.h"
#include "sensor_app.h"

/**
 * @addtogroup	IMU_Test IMU_Test
 * @brief		This file provides a set of functions to be used by IMU Test
 * 				Module.
 * @{
 *
 */

/******************************************************************************
 private constants and types.
 ******************************************************************************/
#define IMU_TEST_TIMEOUT_MS (5000)

typedef enum
{
	T_IMU_NOT_INITIALIZED,
	T_IMU_INITIALIZED,
	T_IMU_NOT_READY,
	T_IMU_MAX_STATUS
} tIMU_ENUM_STATUS;

/******************************************************************************
 private variables.
 ******************************************************************************/
/* Definitions for demoTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t sensorTask_attributes =
{ .name = "sensorTask", .stack_size = 2048, .priority =
		(osPriority_t) osPriorityNormal, };

struct
{
	tIMU_ENUM_STATUS status;
} imu_test_handler =
{ 0 };

/******************************************************************************
 private functions.
 ******************************************************************************/
static void clear_mess(tester_message_t *message)
{
	/** Clear Return Message */
	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
}

/******************************************************************************
 Functions implementation
 ******************************************************************************/
/**
 * @brief	Start the resources used by the IMU_Test module.
 *
 * @param	[in,out]	message
 *
 * @retval	None
 */
void tIMU_StartTest(tester_message_t *message)
{
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (message == NULL)
	{
		return;
	}

	if (imu_test_handler.status == T_IMU_INITIALIZED) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ALREADY_INIT;
		return;
	}

	/** Clear Return Message */
	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	/* creation of demoTask */
	IMUTaskHandle = osThreadNew(IMUDemoTaskStart, NULL, &sensorTask_attributes);
	if (IMUTaskHandle == NULL)
	{
		return;
	}

	imu_test_handler.status = T_IMU_INITIALIZED;
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

/**
 * @brief returns a message with status of test
 * @param message
 */
void tIMU_getStatus(tester_message_t *message)
{
	if (message == NULL)
		return;

	/** Clear Return Message */
	clear_mess(message);

	message->payload[TEST_RESPONSE_BYTE - 1] = imu_test_handler.status;
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_startAccel(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	if (sensor_app_get_state() == SENSOR_APP_NOT_READY) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_NOT_READY;
		return;
	}

	if (sensor_app_startReport(SH2_ACCELEROMETER, 100000) != 0)
	{
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}
void tIMU_startGyro(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	if (sensor_app_get_state() == SENSOR_APP_NOT_READY) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_NOT_READY;
		return;
	}

	if (sensor_app_startReport(SH2_GYROSCOPE_UNCALIBRATED, 100000) != 0)
	{
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;

}
void tIMU_startMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	if (sensor_app_get_state() == SENSOR_APP_NOT_READY) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_NOT_READY;
		return;
	}

	if (sensor_app_startReport(SH2_RAW_MAGNETOMETER, 100000) != 0)
	{
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;

}

void tIMU_startUncalMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	if (sensor_app_get_state() == SENSOR_APP_NOT_READY) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_NOT_READY;
		return;
	}

	if (sensor_app_startReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 100000) != 0)
	{
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_startCalMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	if (sensor_app_get_state() == SENSOR_APP_NOT_READY) {
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_NOT_READY;
		return;
	}

	if (sensor_app_startReport(SH2_MAGNETIC_FIELD_CALIBRATED, 100000) != 0)
	{
		message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_getAccel(tester_message_t *message)
{
	sh2_Accelerometer_t accel_measure;
	clear_mess(message);

	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	sensor_app_getAccel(&accel_measure);
	message->payload[1] = SH2_ACCELEROMETER;
	memcpy(&message->payload[2], &accel_measure, sizeof(sh2_Accelerometer_t));
	TESTER_SetResponseSize(2 + sizeof(sh2_Accelerometer_t));

	return;
}

void tIMU_getGyro(tester_message_t *message)
{
	sh2_GyroscopeUncalibrated_t measure;
	clear_mess(message);

	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	sensor_app_getGyroUncal(&measure);
	message->payload[1] = SH2_GYROSCOPE_UNCALIBRATED;
	memcpy(&message->payload[2], &measure, sizeof(sh2_GyroscopeUncalibrated_t));
	TESTER_SetResponseSize(2 + sizeof(sh2_GyroscopeUncalibrated_t));

	return;
}

void tIMU_getMag(tester_message_t *message)
{
	sh2_RawMagnetometer_t measure;
	clear_mess(message);

	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	sensor_app_getRawMag(&measure);
	message->payload[1] = SH2_RAW_MAGNETOMETER;
	memcpy(&message->payload[2], &measure, sizeof(sh2_RawMagnetometer_t));
	TESTER_SetResponseSize(2 + sizeof(sh2_RawMagnetometer_t));

	return;
}

void tIMU_getCalMag(tester_message_t *message)
{
	sh2_MagneticField_t measure;
	clear_mess(message);

	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	sensor_app_getCalMag(&measure);
	message->payload[1] = SH2_MAGNETIC_FIELD_CALIBRATED;
	memcpy(&message->payload[2], &measure, sizeof(sh2_MagneticField_t));
	TESTER_SetResponseSize(2 + sizeof(sh2_MagneticField_t));

	return;
}

void tIMU_getUncalMag(tester_message_t *message)
{
	sh2_MagneticFieldUncalibrated_t measure;
	clear_mess(message);

	memset(&message->payload[1], 0, sizeof(message->payload) - 1);
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	sensor_app_getUncalMag(&measure);
	message->payload[1] = SH2_MAGNETIC_FIELD_UNCALIBRATED;
	memcpy(&message->payload[2], &measure, sizeof(sh2_MagneticFieldUncalibrated_t));
	TESTER_SetResponseSize(2 + sizeof(sh2_MagneticFieldUncalibrated_t));

	return;
}

void tIMU_stopAccel(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (sensor_app_startReport(SH2_ACCELEROMETER, 0) != 0)
	{
		return;
	}
	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}
void tIMU_stopGyro(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (sensor_app_startReport(SH2_GYROSCOPE_UNCALIBRATED, 0) != 0)
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;

}

void tIMU_stopMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (sensor_app_startReport(SH2_RAW_MAGNETOMETER, 0) != 0)
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_stopUncalMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (sensor_app_startReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0) != 0)
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_stopCalMag(tester_message_t *message)
{
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_ERROR;

	if (sensor_app_startReport(SH2_MAGNETIC_FIELD_CALIBRATED, 0) != 0)
	{
		return;
	}

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}

void tIMU_stopTest(tester_message_t *message)
{
	TaskStatus_t xTaskDetails;
	if ((message == NULL) || (imu_test_handler.status == T_IMU_NOT_INITIALIZED))
	{
		return;
	}
	sensor_app_stop(IMUTaskHandle);

	do
	{
		vTaskDelay(pdMS_TO_TICKS(10));
		vTaskGetInfo(IMUTaskHandle, &xTaskDetails, pdTRUE, eInvalid);
	} while (xTaskDetails.eCurrentState != eDeleted);

	imu_test_handler.status = T_IMU_NOT_INITIALIZED;

	message->payload[TEST_RESPONSE_BYTE] = TEST_IMU_OK;
}
