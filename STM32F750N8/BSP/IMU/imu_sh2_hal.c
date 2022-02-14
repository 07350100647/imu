/*******************************************************************************
 * @file sh2_hal.c
 * 
 * @brief
 * 
 * @version 0.0.1
 *  
 * 
 * All rights are reserved. Reproduction in whole or part is prohibited without
 * the written consent of the copyright owner.
 ******************************************************************************/
#include <IMU/imu_sh2_hal.h>
#include "../Middlewares/bno080-driver/sh2_err.h"

/******************************************************************************
 private constants and types.
 ******************************************************************************/

/******************************************************************************
 private variables.
 ******************************************************************************/

/******************************************************************************
 private functions.
 ******************************************************************************/

/******************************************************************************
 Functions implementation
 ******************************************************************************/
//#define IMU_HAS_DFU
int tx_dfu(bsp_imu_handle_t *imu_handler_p, uint8_t *pData, uint32_t len) {
#ifndef IMU_HAS_DFU
	return SH2_ERR;
#else
	int status = SH2_OK;

	takeBus();

// assert CSN
	dev_p->csn(false);

	delayUs(DFU_CS_TIMING_US);

// Set up Tx, Rx bufs
	spiTxData = pData;
	spiRxData = dev_p->rxBuf;

// We will just use a simple one-phase transfer for DFU
	transferPhase = TRANSFER_DATA;

// initiate transfers.  Do them one byte at a time because BNO needs some
// time between bytes.
	int rc = 0;
	for (int n = 0; n < len; n++) {
		//dbgPulse(5);
		rc = HAL_SPI_Transmit(&imu_handle.spi_handle, (uint8_t*) spiTxData + n,
				1, 1);
		if (rc != 0) {
			break;
		}
		delayUs(DFU_BYTE_TIMING_US);
	}
	spiTransferLen = len;

	if (rc == 0) {
		// Set return status
		dev_p->rxLen = spiTransferLen;
		status = spiOpStatus;
	} else {
		// SPI operation failed
		dev_p->rxLen = 0;
		status = SH2_ERR_IO;
	}

// deassert CSN
	dev_p->csn(true);

// Wait on each CSN assertion.  DFU Requires at least 5ms of deasserted time!
	vTaskDelay(DFU_CS_DEASSERT_DELAY_TX);

	relBus();

	return status;
#endif //IMU_HAS_DFU
}

int rx_dfu(bsp_imu_handle_t *imu_handler_p, uint8_t *pData, uint32_t len) {
#ifndef IMU_HAS_DFU
	return SH2_ERR;
#else
	int status = SH2_OK;

	takeBus();

// Wait on each CSN assertion.  DFU Requires at least 5ms of deasserted time!
	vTaskDelay(DFU_CS_DEASSERT_DELAY_RX);

// assert CSN
	dev_p->csn(false);
	delayUs(DFU_CS_TIMING_US);

// Set up Tx, Rx bufs
	spiTxData = txZeros;
	spiRxData = pData;

// We will just use a simple one-phase transfer for DFU
	transferPhase = TRANSFER_DATA;

// initiate transfer
	int rc = 0;
	spiTransferLen = len;
	for (int n = 0; n < len; n++) {
		//dbgPulse(5);
		rc = HAL_SPI_Receive(&imu_handle.spi_handle, spiRxData + n, 1, 1);
		if (rc != 0) {
			break;
		}
		delayUs(DFU_BYTE_TIMING_US);
	}

	if (rc == 0) {
		// Set return status
		dev_p->rxLen = spiTransferLen;
		status = spiOpStatus;
	} else {
		// SPI operation failed
		dev_p->rxLen = 0;
		status = SH2_ERR_IO;
	}

// deassert CSN
	dev_p->csn(true);

// Wait after each CSN deassertion in DFU mode to ensure proper timing.
	vTaskDelay(DFU_CS_DEASSERT_DELAY_RX);

	relBus();

	return status;
#endif	//IMU_HAS_DFU
}
