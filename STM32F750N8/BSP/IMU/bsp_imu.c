/*******************************************************************************
 * @file bsp_imu.c
 *
 * @brief
 *
 * @version 0.0.1
 *
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/
#include "bsp_imu.h"
#include "imu_sh2_hal.h"

#include <string.h>

#include <dma.h>

#include "../Middlewares/bno080-driver/sh2.h"
#include "../Middlewares/bno080-driver/sh2_hal.h"
#include "../Middlewares/bno080-driver/sh2_err.h"
#include "../Middlewares/bno080-driver/sh2_SensorValue.h"

#include "main.h"

/**
 * @addtogroup BSP
 *
 * @{
 */
/**
 * @defgroup IMU IMU
 * @brief The board features a BNO080 IMU module that is controlled through SPI
 * using Hillcrest’s Sensor Hub Transport Protocol (SHTP).
 *
 * 		BSP_IMU Module implements functions to handle SPI peripheral and
 * required functions by Hillcrest's driver as Hardware abstraction layer.
 *
 * 		Bellow the block diagram describe the interaction between application
 * 	and BNO080 IMU module.
 *
 * @image html "Diagrama em blocos imu.png" width=361px
 * @image latex "Diagrama em blocos imu.png" IMU block diagram
 *
 * To keep compatibility with Hillcrest's driver
 * (https://github.com/hcrest/bno080-driver), this files will be integrated as
 * middleware and application would use it directly.
 *
 * 		Through @ref bsp_imu_Init developer will initialize, configure initial
 * parameter of BN0080 and configure function handler for the events.
 *
 * 		To use BNO080, please visit:
 * 	- https://github.com/hcrest/bno080-driver
 * 	- https://www.ceva-dsp.com/resource/sh-2-shtp-reference-manual/
 *
 * @{
 */
/**
 * @defgroup Hardware Hardware
 *
 * @{
 */
/******************************************************************************
 private constants and types.
 ******************************************************************************/
typedef enum {
	TRANSFER_IDLE = 0,    // SPI device not in use
	TRANSFER_DATA, // Transferring bulk of data (DFU or second phase SHTP)
	TRANSFER_HDR,         // Transferring first two bytes of SHTP
} TransferPhase_t;

typedef enum {
	EVT_INTN, EVT_OP_CPLT, EVT_OP_ERR,
} EventId_t;

typedef struct {
	EventId_t id;
	uint32_t t_uS;
} Event_t;

/******************************************************************************
 private variables.
 ******************************************************************************/
bsp_imu_handle_t imu_handle = { 0 };

// SPI Bus access
static int spiOpStatus;
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER];
static const uint8_t *spiTxData;
static uint8_t *spiRxData;
static TransferPhase_t transferPhase;

/******************************************************************************
 private functions.
 ******************************************************************************/
static HAL_StatusTypeDef spiReset(bool dfuMode);
void ImuSPI_HW_Init(void);
void spiEvent(Event_t event);

#ifdef IMU_HAS_DFU
static void delayUs(uint32_t count);
#endif //IMU_HAS_DFU
/******************************************************************************
 Functions implementation
 ******************************************************************************/
static void rstn0(bool state) {
	HAL_GPIO_WritePin(RSTN_GPIO_PORT, RSTN_GPIO_PIN,
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void csn0(bool state) {
	HAL_GPIO_WritePin(CSN_GPIO_PORT, CSN_GPIO_PIN,
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void waken0(bool state) {
	HAL_GPIO_WritePin(WAKEN_GPIO_PORT, WAKEN_GPIO_PIN,
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint16_t exti_cnt = 0;
/**
 * @brief Handles the INTN interrupt raised by BNO080 device.
 *
 * In this IRQ, transmission and reception must be execute on SPI.
 *
 */
void IMU_EXTI_Callback(void) {
	Event_t event;
	exti_cnt++;
	event.t_uS = HAL_GetTick() * 1000;
	event.id = EVT_INTN;

	spiEvent(event);
}

/**
 * @brief Deinitialize IMU pins
 */
void ImuSPI_HW_Deinit(void) {
	IMU_SPI_CLOCK_DISABLE();

	HAL_GPIO_PIN_DeInit(IMU_RST_GPIO_Port, IMU_RST_Pin);
	HAL_GPIO_PIN_DeInit(IMU_NSS_GPIO_Port, IMU_NSS_Pin);
	HAL_GPIO_PIN_DeInit(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin);
	HAL_GPIO_PIN_DeInit(IMU_INTN_GPIO_Port, IMU_INTN_Pin);

	HAL_GPIO_PIN_DeInit(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin);
	HAL_GPIO_PIN_DeInit(IMU_MISO_GPIO_Port, IMU_MISO_Pin);
	HAL_GPIO_PIN_DeInit(IMU_SCK_GPIO_Port, IMU_SCK_Pin);
}

/**
 * @brief Initialize IMU pins
 */
void ImuSPI_HW_Init(void) {
	IMU_SPI_CLOCK_ENABLE();

	//Set Pins initial state
	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin, GPIO_PIN_RESET);

	HAL_GPIO_PIN_Init(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_MODE_OUTPUT_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_DEFAULT);
	HAL_GPIO_PIN_Init(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_MODE_OUTPUT_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_DEFAULT);
	HAL_GPIO_PIN_Init(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_MODE_OUTPUT_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_DEFAULT);
	HAL_GPIO_PIN_Init(IMU_INTN_GPIO_Port, IMU_INTN_Pin, GPIO_MODE_IT_FALLING,
	GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_DEFAULT);

	HAL_GPIO_PIN_Init(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin, GPIO_MODE_AF_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, IMU_PIN_FUNCTION);
	HAL_GPIO_PIN_Init(IMU_MISO_GPIO_Port, IMU_MISO_Pin, GPIO_MODE_AF_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, IMU_PIN_FUNCTION);
	HAL_GPIO_PIN_Init(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_MODE_AF_PP,
	GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, IMU_PIN_FUNCTION);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);

}

/**
 * @brief Handles Tx or Rx Cplt events form SPI.
 *
 * @param hspi SPI handle struct.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	bool opFinished = false;
	Event_t event;
	// What to do next depends on transfer phase
	if (transferPhase == TRANSFER_HDR) {
		//Header phase: Header was received. Verify integrity and start data rx.
		uint16_t txLen = ((spiTxData[0] + (spiTxData[1] << 8)) & ~0x8000);
		uint16_t rxLen = ((spiRxData[0] + (spiRxData[1] << 8)) & ~0x8000);

		if (rxLen == 0x7FFF) {
			// 0x7FFF is an invalid length
			rxLen = 0;
		}

		uint16_t len = (txLen > rxLen) ? txLen : rxLen;
		if (len > SH2_HAL_MAX_TRANSFER) {
			len = SH2_HAL_MAX_TRANSFER;
		}

		if (len == 0) {
			// Nothing left to transfer!
			spiOpStatus = SH2_OK;
			opFinished = true;
		} else {
			// Start data phase of tranfer
			transferPhase = TRANSFER_DATA;
			imu_handle.rxLen = len;

			HAL_SPI_DMAStop(hspi);

			int rc = HAL_SPI_TransmitReceive_DMA(hspi,
					(uint8_t*) (spiTxData + SH2_HDR_SIZE),
					(spiRxData + SH2_HDR_SIZE), len - SH2_HDR_SIZE);
			if (rc != 0) {
				// Signal IO Error to HAL task
				spiOpStatus = SH2_ERR_IO;
				opFinished = true;
			}
		}
	} else if (transferPhase == TRANSFER_DATA) {
		//Transfer is finished. Process it.
		spiOpStatus = SH2_OK;
		opFinished = true;
	}

// If operation finished for any reason, unblock the caller
	if (opFinished) {
		transferPhase = TRANSFER_IDLE;

		event.id = EVT_OP_CPLT;
		event.t_uS = 99;    // not used

		spiEvent(event);
	}
}

/**
 * @brief Handles with any fault occurred on SPI operation.
 *
 * @param hspi SPI handle struct.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	Event_t event;
// transfer is over
	transferPhase = TRANSFER_IDLE;

// Set status from this operation
	spiOpStatus = SH2_ERR_IO;

	event.id = EVT_OP_ERR;
	event.t_uS = 99;    // not used

	spiEvent(event);
}

/**
 * @brief Implementation required by STM structure
 *
 */
void SPI3_IRQHandler(void) {
	HAL_SPI_IRQHandler(&imu_handle.spi_handle);
}

/**
 *@}
 */ // Hardware
/**
 * @defgroup SH2_HAL SH2 Hardware Abstraction Layer
 * @{
 */

/**
 * @brief Initialize SH2 HW
 */
void sh2_hal_init(void) {
	rstn0(false);  // Hold in reset
	csn0(true);    // deassert CSN
	waken0(true);  // deassert WAKEN.

	imu_handle.is_initialized = true;
}

/**
 * @brief Function called by SH2 to reset IMU
 *
 * @param dfuMode	Define IMU boot in dfuMODE (not implemented)
 * @param onRx		Function pointer to be called on SPI RX Complete
 * @param cookie	Cookie to be used on transactions
 *
 * @return	SH2_OK on success
 */
int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie) {
	//Avoid spurious int
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

// Store params for later reference
	imu_handle.dfuMode = dfuMode;
	imu_handle.cookie = cookie;
	imu_handle.onRx = onRx;

// Wait a bit before asserting reset.
// (Because this may be a reset after a DFU and that process needs
// an extra few ms to store data in flash before the device is
// actually reset.)
	imu_handle.delay(RESET_DELAY);

// Assert reset
	rstn0(0);

// Deassert CSN in case it was asserted
	csn0(1);

// Set BOOTN according to dfuMode
//imu_handle.bootn(dfuMode ? 0 : 1);

// set PS0 (WAKEN) to support booting into SPI mode.
	waken0(1);

// Reset SPI parameters
	spiReset(imu_handle.dfuMode);

// Wait for reset to take effect
	imu_handle.delay(RESET_DELAY);

// Enable INTN interrupt
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//    SPI3_IRQHandler();

// Deassert reset
	rstn0(1);
	imu_handle.delay(RESET_DELAY);

// If reset into DFU mode, wait until bootloader should be ready
	if (imu_handle.dfuMode) {
		imu_handle.delay(DFU_BOOT_DELAY);
	}
	return SH2_OK;
}

/**
 * @brief Send data to SH-2
 *
 * @param pData pointer to data to be transmitted
 * @param len	length of data to be transmitted
 * @return
 */
int sh2_hal_tx(uint8_t *pData, uint32_t len) {
// Do nothing if len is zero
	if (len == 0) {
		return SH2_OK;
	}

	if (imu_handle.dfuMode) {
		return tx_dfu(&imu_handle, pData, len);
	} else {
		return tx_shtp(pData, len);
	}
}

/**
 * @brief Initiate a read of <len> bytes from SH-2
 *
 * This is a blocking read, pData will contain read data on return
 * if return value was SH2_OK.
 *
 * @warning THIS FUNCTION DON'T HAVE EFFECT BECAUSE DFU IS NOT IMPLEMENTED
 *
 * @param pData
 * @param len
 * @return SH2_OK on success, SH2_ERR otherwise
 */
int sh2_hal_rx(uint8_t *pData, uint32_t len) {
// Do nothing if len is zero
	if (len == 0) {
		return SH2_OK;
	}

	if (imu_handle.dfuMode) {
		return rx_dfu(&imu_handle, pData, len);
	} else {
		// sh2_hal_rx API isn't used in non-DFU mode.
		return SH2_ERR;
	}
	return SH2_ERR;
}

/**
 * @brief This function is called on SH2 commands, requesting RTOS to hold until
 * finish.
 *
 * @return SH2_OK on success
 */
int sh2_hal_block(void) {
	if (imu_handle.imu_block != NULL)
		return imu_handle.imu_block();

	return SH2_OK;
}

/**
 * @brief This function is called by SH2, in the end of commands processing,
 * releasing RTOS from hold.
 *
 * @return SH2_OK on success
 */
int sh2_hal_unblock(void) {
	if (imu_handle.imu_unblock != NULL)
		return imu_handle.imu_unblock();

	return SH2_OK;
}

/**
 * @}
 */ // SH2_HAL
/**
 * @defgroup Application Application
 * @{
 */

static void takeBus(void) {
// get bus mutex
	//!TODO
//	xSemaphoreTake(spiMutex, portMAX_DELAY);
}

static void relBus(void) {
// Release bus
	//!TODO
//	xSemaphoreGive(spiMutex);
}

static void deliverRx(uint32_t t_uS) {
// Deliver results via onRx callback
	if (imu_handle.onRx != NULL) {
		if (imu_handle.rxLen) {
			imu_handle.onRx(imu_handle.cookie, imu_handle.rxBuf,
					imu_handle.rxLen, t_uS);
		}
	}
}

static void endOpShtp(void) {
// Release transmit mutex if it's held.
	imu_handle.txLen = 0;

	if (imu_handle.tx_unlock != NULL) {
		imu_handle.tx_unlock();
	}

// deassert CSN
	csn0(true);

// Release the bus
	relBus();
}

static int startOpShtp(void) {
	int retval = 0;

// Set up operation on bus
	takeBus();

// assert CSN
	csn0(false);

// Read into device's rxBuf
	spiRxData = imu_handle.rxBuf;

// If there is stuff to transmit, deassert WAKE and do it now.
	spiTxData = txZeros;
	if (imu_handle.txLen) {
		waken0(true);
		spiTxData = imu_handle.txBuf;
	}

// initiate (Header phase of) transfer
	transferPhase = TRANSFER_HDR;
	imu_handle.rxLen = SH2_HDR_SIZE;

	int rc = HAL_SPI_TransmitReceive_DMA(&imu_handle.spi_handle,
			(uint8_t*) spiTxData, spiRxData, SH2_HDR_SIZE);

	if (rc != 0) {
		// Failed to start!  Abort!
		HAL_SPI_Abort_IT(&imu_handle.spi_handle);
		endOpShtp();
		retval = -1;
	}

	return retval;
}

static uint16_t dma_err_cnt = 0;
static uint16_t evt_def_cnt = 0;

void spiEvent(Event_t event) {
	int rc;

	// Handle the event
	switch (event.id) {

	case EVT_INTN:
		if (imu_handle.dfuMode) {
			// Ignore INTN in DFU mode
		} else {
			if (imu_handle.state == DEV_IDLE) {
				// Start a new operation and go to IN-PROGRESS state
				imu_handle.state = DEV_IN_PROG;
				imu_handle.t_uS = event.t_uS;
				rc = startOpShtp();

				if (rc) {
					// failure to start
					imu_handle.state = DEV_IDLE;
				}
			} else {
				// An operation is still in progress, go to NEW-INTN state
				imu_handle.pending_t_uS = event.t_uS;
				imu_handle.state = DEV_NEW_INTN;

			}
		}
		break;
	case EVT_OP_CPLT:
		if (imu_handle.dfuMode) {
			// Ignore this event.
			// By design, this shouldn't happen.  DFU mode doesn't use interrupt
			// mode SPI API.
		} else {
			// Post-op for operation that just completed
			endOpShtp();

			// Deliver received content
			deliverRx(imu_handle.t_uS);

			// If a new INTN was signalled, start the next op
			if (imu_handle.state == DEV_NEW_INTN) {
				// start next op
				imu_handle.t_uS = imu_handle.pending_t_uS;
				imu_handle.state = DEV_IN_PROG;
				rc = startOpShtp();
				if (rc) {
					// failure to start
					imu_handle.state = DEV_IDLE;
				}
			} else {
				// no operation in progress now.
				imu_handle.state = DEV_IDLE;

			}
		}
		break;
	case EVT_OP_ERR:
		if (imu_handle.dfuMode) {
			// Ignore this event.
			// By design, this shouldn't happen.  DFU mode doesn't use interrupt
			// mode SPI API.
		} else {
			endOpShtp();

			// If a new INTN was signalled, start the next op
			if (imu_handle.state == DEV_NEW_INTN) {
				// start next op
				imu_handle.t_uS = imu_handle.pending_t_uS;
				imu_handle.state = DEV_IN_PROG;
				rc = startOpShtp();
				if (rc) {
					// failure to start
					imu_handle.state = DEV_IDLE;
				}
			} else {
				// no operation in progress now.
				imu_handle.state = DEV_IDLE;
			}
		}
		break;
	default:
		// Unknown event type.  Ignore.
		evt_def_cnt++;
		break;
	}
}

void dma_err_cb(DMA_HandleTypeDef *_hdma) {
	dma_err_cnt++;
	HAL_Delay(10);
}

// DeInit and Init SPI Peripheral.
static HAL_StatusTypeDef spiReset(bool dfuMode) {
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_HandleTypeDef *hspi = &imu_handle.spi_handle;

	HAL_SPI_Abort(hspi);
	HAL_NVIC_DisableIRQ(IMU_SPI_IRQn);

	memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

	HAL_SPI_DeInit(hspi);

// Common parameters
//Configure SPI
	hspi->Instance = IMU_SPI;
	hspi->Init.Mode = SPI_MODE_MASTER;
	hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	hspi->Init.NSS = SPI_NSS_SOFT;
	hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi->Init.CRCPolynomial = 10;
	hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

// Differences between DFU and App
	if (dfuMode) {
		hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;

		// 84MHz/128=0.65MHz
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	} else {
		hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
		hspi->Init.CLKPhase = SPI_PHASE_2EDGE;

		// 54MHz/64=0.84375MHz; 32=1,68MHz
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	}

	if (HAL_SPI_Init(hspi) != HAL_OK) {
		return HAL_ERROR;
	}

	/* Initialize SPI Interrupt */
	HAL_NVIC_SetPriority(IMU_SPI_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(IMU_SPI_IRQn);

	HAL_DMA_RegisterCallback(hspi->hdmarx, HAL_DMA_XFER_ALL_CB_ID, dma_err_cb);

	MX_DMA_Init();

	csn0(true);
// We need to establish SCLK in proper initial state.
// Do one SPI operation with reset asserted and no CS asserted to get clock
// sorted.
	uint8_t dummyTx[1];
	uint8_t dummyRx[1];

	memset(dummyTx, 0xAA, sizeof(dummyTx));

//dbgPulse(5);

	return HAL_SPI_TransmitReceive(hspi, dummyTx, dummyRx, sizeof(dummyTx), 1);

}

int tx_shtp(uint8_t *pData, uint32_t len) {
	int status = SH2_OK;

// Get semaphore on device
	if (imu_handle.tx_lock != NULL) {
		//Try lock
		if (imu_handle.tx_lock() != BSP_OK) {
			return SH2_ERR;
		}
	}

// Set up txLen and txBuf
	memset(imu_handle.txBuf, 0, sizeof(imu_handle.txBuf));
	memcpy(imu_handle.txBuf, pData, len);

// Assert WAKE
//dbgSet();

	waken0(false);

// Finally, set len, which triggers tx processing in HAL task
	imu_handle.txLen = len;

// Transmission will take place after INTN is processed.

	return status;
}

//static void bootn0(bool state)
//{
//	HAL_GPIO_WritePin(BOOTN_GPIO_PORT, BOOTN_GPIO_PIN,
//	                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
//}

/**
 * @brief Resets IMU without deinitialize SPI
 *
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_reset(bsp_imu_handle_t *handle) {
	sh2_hal_reset(false, imu_handle.onRx, imu_handle.cookie);
	return BSP_OK;
}

/**
 * @brief Deinitialize IMU module.
 * @param handle Struct pointer to hold module data.
 * @return
 */
BSP_Return bsp_imu_Deinit(bsp_imu_handle_t **handle) {
	if ((handle == NULL) || ((*handle) == NULL) || ((*handle) != &imu_handle))
		return BSP_NOT_INITIALIZED;

	//Deassert wake and CSN
	waken0(true);
	csn0(true);
	rstn0(false);

	//!TODO Deinit SPI

	//Disable INTs
	//Avoid spurious int
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	//Disable SPI
	if (HAL_SPI_DeInit(&(*handle)->spi_handle) != HAL_OK)
		return BSP_ERROR;

	//Disable PINs
	ImuSPI_HW_Deinit();

	//Release handle
	(*handle)->is_initialized = false;

	memset((*handle), 0, sizeof(bsp_imu_handle_t));
	*handle = NULL;

	return BSP_OK;

}

/**
 * @brief Initialize IMU.
 *
 * When called, this function will instantiate SPI and initialize IMU
 *
 * @param evt_cb Function pointer to handle events from driver.
 * @param sensor_cb Function pointer to handle sensor events.
 *
 * @return BSP_OK on success, BSP_ERROR otherwise.
 *
 * @todo complements with SH2 required parameters
 */
bsp_imu_handle_t* bsp_imu_Init(sh2_EventCallback_t *evt_cb,
		sh2_SensorCallback_t *sensor_cb, lock_unlock_f *tx_lock,
		lock_unlock_f *tx_unlock, sh2_hal_block_unblock_t block_f,
		sh2_hal_block_unblock_t unblock_f, delay_func_p *delay_func) {
	if (imu_handle.is_initialized) {
		return NULL;
	}

	if ((evt_cb == NULL) || (sensor_cb == NULL)) {
		return NULL;
	}

	ImuSPI_HW_Init();

	if (spiReset(false) != HAL_OK) {
		// This should not have happened, debug it.
		ImuSPI_HW_Deinit();
		return NULL;
	}

	//Initialize SH2 Driver
	sh2_hal_init();

	// Semaphore to protect transmit state
	if (tx_lock != NULL) {
		imu_handle.tx_lock = tx_lock;
		imu_handle.tx_lock();
	}

	if (tx_unlock != NULL) {
		imu_handle.tx_unlock = tx_unlock;
		imu_handle.tx_unlock();
	}

	if (block_f != NULL) {
		imu_handle.imu_block = block_f;
	}

	if (unblock_f != NULL) {
		imu_handle.imu_unblock = unblock_f;
	}

	if (delay_func != NULL) {
		imu_handle.delay = delay_func;
	}

	// Create queue to pass events from ISRs to task context.

	sh2_initialize(evt_cb, NULL);

	// Register event listener
	sh2_setSensorCallback(sensor_cb, NULL);
	// wait for reset notification, or just go ahead after 100ms
	//Application would wait for a reset event to start programming sensor

	imu_handle.is_initialized = true;
	return &imu_handle;

}

/**
 * @}
 */ //Application
/**
 * @}
 */// IMU
/**
 * @}
 */// BSP
//#define IMU_HAS_DFU
#ifdef IMU_HAS_DFU

#define DELAY_LOOP_1_US (54)
#define DELAY_LOOP_OFFSET (-360)

static void delayUs(uint32_t count) {
	float delayCount;
	volatile unsigned delayCounter;

	delayCount = DELAY_LOOP_1_US * count + DELAY_LOOP_OFFSET;
	if (delayCount < 1.0)
		return;

	delayCounter = (unsigned) delayCount;

	while (delayCounter > 0)
		delayCounter--;
}
#endif //IMU_HAS_DFU
