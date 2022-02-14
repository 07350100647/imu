/*******************************************************************************
 * @file sh2_hal.h
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/

#ifndef IMU_IMU_SH2_HAL_H_
#define IMU_IMU_SH2_HAL_H_

#include <stdbool.h>
#include "bsp_imu_def.h"
#include "../Middlewares/bno080-driver/sh2_hal.h"

/******************************************************************************
 public constants and types.
 ******************************************************************************/

/******************************************************************************
 public variables.
 ******************************************************************************/

/******************************************************************************
 public functions.
 ******************************************************************************/
int tx_dfu(bsp_imu_handle_t *imu_handler_p, uint8_t *pData, uint32_t len);
int rx_dfu(bsp_imu_handle_t *imu_handler_p, uint8_t *pData, uint32_t len);

int tx_shtp(uint8_t *pData, uint32_t len);
#endif //IMU_IMU_SH2_HAL_H_
