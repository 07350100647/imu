/*******************************************************************************
 * @file bsp_imu_def.h
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/

#ifndef IMU_BSP_IMU_DEF_H_
#define IMU_BSP_IMU_DEF_H_

/* Includes */
#include <stdbool.h>

/* STM32 HAL Driver */
#include "stm32f7xx_hal.h"

#include "../Middlewares/bno080-driver/sh2_hal.h"

/******************************************************************************
 public constants and types.
 ******************************************************************************/
/**
 * @addtogroup	IMU_Defines	Defines
 * @brief		IMU Hardware Definitions
 * @{
 */

/**
 * @name IMU SPI Definitions
 * @{
 */
#define IMU_SPI						SPI3
#define IMU_SPI_IRQHandler			SPI3_IRQHandler
#define IMU_SPI_IRQn				SPI3_IRQn
/**
 * @}
 */ //IMU SPI Definitions
/**
 * @}
 */ //IMU_Defines


/**
 * @brief Accelerometer Handle Structure
 */
// Timing parameters for DFU
#define DFU_BOOT_DELAY (50)          // [mS]
#define RESET_DELAY    (20)          // [mS]

#define DFU_CS_DEASSERT_DELAY_RX (0) // [mS]
#define DFU_CS_DEASSERT_DELAY_TX (5) // [mS]
#define DFU_CS_TIMING_US (20)        // [uS]
#define DFU_BYTE_TIMING_US (28)      // [uS]

#define MAX_EVENTS (16)
#define SHTP_HEADER_LEN (4)

#define RSTN_GPIO_PORT GPIOJ
#define RSTN_GPIO_PIN  GPIO_PIN_11

//#define BOOTN_GPIO_PORT GPIOB
//#define BOOTN_GPIO_PIN  GPIO_PIN_5

#define CSN_GPIO_PORT GPIOC // from STM32Cube pin config via mxconstants.h
#define CSN_GPIO_PIN  GPIO_PIN_6

#define WAKEN_GPIO_PORT GPIOK // from STM32Cube pin config via mxconstants.h
#define WAKEN_GPIO_PIN  GPIO_PIN_0

#define TIMESTAMP_0 (0)
#define SH2_HDR_SIZE	(2)

typedef enum {
	DEV_IDLE, DEV_IN_PROG, DEV_NEW_INTN,
} enum_dev_states_t;

typedef uint8_t (lock_unlock_f)(void);

typedef int (sh2_hal_block_unblock_t)(void);

/**
 * Prototype of delay function to be used by bsp_imu to delay some process.
 * @param time_ms delay duration ms.
 *
 */
typedef void (delay_func_p)(uint8_t time_ms);

typedef struct {
	SPI_HandleTypeDef spi_handle;

	lock_unlock_f *tx_lock;
	lock_unlock_f *tx_unlock;

	sh2_hal_block_unblock_t *imu_block;
	sh2_hal_block_unblock_t *imu_unblock;

	delay_func_p *delay;

	// Rx resources
	uint8_t rxBuf[SH2_HAL_MAX_TRANSFER];
	uint16_t rxLen;

	// Tx resources
	uint8_t txBuf[SH2_HAL_MAX_TRANSFER];
	uint16_t txLen;

	bool dfuMode;
	sh2_rxCallback_t *onRx;
	void *cookie;

	uint32_t pending_t_uS;
	uint32_t t_uS;

	bool is_initialized;

	enum_dev_states_t state;

	HAL_StatusTypeDef (*tx)(SPI_HandleTypeDef *spi_handle, uint8_t *data,
			uint16_t size);
	HAL_StatusTypeDef (*rx)(SPI_HandleTypeDef *spi_handle, uint8_t *data,
			uint16_t size);
} bsp_imu_handle_t;

/******************************************************************************
 public variables.
 ******************************************************************************/

/******************************************************************************
 public functions.
 ******************************************************************************/

#endif //IMU_BSP_IMU_DEF_H_
