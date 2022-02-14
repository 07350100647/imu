/*******************************************************************************
 * @file imu_test.h
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 *
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/

#ifndef IMU_TEST_IMU_TEST_H_
#define IMU_TEST_IMU_TEST_H_
/**
 * @addtogroup Test
 * @{
 */

/**
 * @addtogroup IMU_Test
 * @{
 */
/* STM32 HAL Driver */
#include "stm32f7xx_hal.h"

/* BSP Common Header */
#include "bsp_common.h"

/* Tester Header */
#include "Tester/tester.h"



/******************************************************************************
    public constants and types.
******************************************************************************/
/**
 * @brief IMU Return Messages Enumerate
 */
typedef enum {
	TEST_IMU_ERROR = 0,
	TEST_IMU_OK = 1,
	TEST_IMU_ALREADY_INIT = 2,
	TEST_IMU_NOT_READY
} tIMU_RetCode;
    
 
 
/******************************************************************************
    public variables.
******************************************************************************/
 
 
/******************************************************************************
    public functions.
******************************************************************************/


/* Public Declarations */
void tIMU_StartTest(tester_message_t *message);
void tIMU_getStatus(tester_message_t *message);

void tIMU_startAccel (tester_message_t *message);
void tIMU_startGyro(tester_message_t *message);
void tIMU_startMag(tester_message_t *message);
void tIMU_startUncalMag(tester_message_t *message);
void tIMU_startCalMag(tester_message_t *message);

void tIMU_getAccel(tester_message_t *message);
void tIMU_getGyro(tester_message_t *message);
void tIMU_getMag(tester_message_t *message);
void tIMU_getUncalMag(tester_message_t *message);
void tIMU_getCalMag(tester_message_t *message);

void tIMU_stopAccel (tester_message_t *message);
void tIMU_stopGyro(tester_message_t *message);
void tIMU_stopMag(tester_message_t *message);
void tIMU_stopUncalMag(tester_message_t *message);
void tIMU_stopCalMag(tester_message_t *message);

void tIMU_stopTest(tester_message_t *message);
/**
 * @}
 */

/**
 * @}
 */

#endif //IMU_TEST_IMU_TEST_H_
