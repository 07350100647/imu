/*******************************************************************************
 * @file bsp_imu.h
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/

#ifndef IMU_BSP_IMU_H_
#define IMU_BSP_IMU_H_

/**
 * @addtogroup IMU IMU
 *
 * @{
 */

#include "bsp_common.h"
#include "bsp_imu_def.h"
#include "../Middlewares/bno080-driver/sh2_SensorValue.h"

#define IMU_PIN_FUNCTION			GPIO_AF5_SPI3

#define IMU_SPI					SPI3
#define IMU_SPI_IRQHandler		SPI3_IRQHandler
#define IMU_SPI_IRQn			SPI3_IRQn

#define IMU_SPI_CLOCK_ENABLE() 	__HAL_RCC_SPI3_CLK_ENABLE()
#define IMU_SPI_CLOCK_DISABLE() __HAL_RCC_SPI3_CLK_DISABLE()


/******************************************************************************
 public constants and types.
 ******************************************************************************/
void halTask_init(const void *params);

/******************************************************************************
 public variables.
 ******************************************************************************/

/******************************************************************************
 public functions.
 ******************************************************************************/
/**
 * @brief Initialize IMU.
 *
 * Whem called, this function will instantiate SPI and initialize IMU
 *
 * @param evt_cb Function pointer to handle events from driver.
 * @param sensor_cb Function pointer to handle sensor events.
 *
 * @return BSP_OK on success, BSP_ERROR otherwise.
 *
 */
bsp_imu_handle_t *bsp_imu_Init(sh2_EventCallback_t *evt_cb,
		sh2_SensorCallback_t *sensor_cb, lock_unlock_f *tx_lock,
		lock_unlock_f *tx_unlock, sh2_hal_block_unblock_t block_f,
		sh2_hal_block_unblock_t unblock_f, delay_func_p *delay_func);

/**
 * @brief Deinitialize IMU module.
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_Deinit(bsp_imu_handle_t **handle);

/**
 * @brief Resets IMU without deinitialize SPI
 *
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_reset (bsp_imu_handle_t *handle);


/**
 * @brief Block access to the IMU avoiding concurrent access.
 *
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_block (bsp_imu_handle_t *handle);

/**
 * @brief Release access to the IMU avoiding concurrent access.
 *
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_unblock (bsp_imu_handle_t *handle);

/**
 * @brief IMU callback
 *
 */
void IMU_EXTI_Callback(void);

/**
 * @}
 */// IMU
#endif //IMU_BSP_IMU_H_

