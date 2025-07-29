/*
 * BMI088.c
 *
 *  Created on: Mar 7, 2025
 *      Author: crist
 */

#include "config.h"
#include "BMI088.h"

/**
 * @brief Initialize the BMI088 IMU sensor (accelerometer and gyroscope).
 * @param imu Pointer to BMI088 structure.
 * @param spiHandle SPI handle.
 * @param csAccPinBank GPIO bank for accelerometer CS.
 * @param csAccPin Pin number for accelerometer CS.
 * @param csGyrPinBank GPIO bank for gyroscope CS.
 * @param csGyrPin Pin number for gyroscope CS.
 * @return Initialization status (0 if OK).
 */
uint8_t BMI088_Init(BMI088 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				 GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin)
{

	/* Store interface parameters in struct */
		imu->spiHandle = spiHandle;
		imu->csAccPinBank = csAccPinBank;
		imu->csAccPin = csAccPin;
		imu->csGyrPinBank = csGyrPinBank;
		imu->csGyrPin = csGyrPin;

		/* Clear DMA flags */
		imu->readingAcc = 0;
		imu->readingGyr = 0;

		uint8_t status = 0;

		/* Accelerometer requires rising edge on CSB at start-up to activate SPI */
		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
		HAL_Delay(50);

		/* Perform accelerometer soft reset */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
		HAL_Delay(50);

		/* Check chip ID */
		uint8_t chipID;
		status += BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

		if (chipID != 0x1E) {
			// return 0;  // Commented out for debug or testing purposes
		}
		HAL_Delay(10);

		/* Configure accelerometer: no oversampling, ODR = 200 Hz, BW = 40 Hz */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, 0x09);
		HAL_Delay(10);

		/* +-6g measurement range */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, 0x01);
		HAL_Delay(10);

		/* Configure accelerometer interrupt */
		status += BMI088_WriteAccRegister(imu, BMI_INT1_IO_CONF, 0x0A);
		HAL_Delay(10);
		status += BMI088_WriteAccRegister(imu, BMI_INT1_INT2_MAP_DATA, 0x04);
		HAL_Delay(10);

		/* Set accelerometer to active mode */
		status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
		HAL_Delay(10);
		status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
		HAL_Delay(10);

		/* Compute conversion factor for acceleration data */
		imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f;

		/* Prepare TX buffer for DMA read */
		imu->accTxBuf[0] = BMI_ACC_DATA | 0x80;

		/* GYROSCOPE CONFIGURATION */
		HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

		/* Perform gyroscope soft reset */
		status += BMI088_WriteGyrRegister(imu, BMI_GYR_SOFTRESET, 0xB6);
		HAL_Delay(250);

		/* Check chip ID */
		status += BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);
		if (chipID != 0x0F) {
			// return 0;  // Commented out for debug or testing purposes
		}
		HAL_Delay(10);

		/* Set gyroscope range to +-1000 deg/s */
		status += BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, 0x01);
		HAL_Delay(10);

		/* Set gyroscope bandwidth to 64 Hz, ODR = 200 Hz */
		status += BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, 0x06);
		HAL_Delay(10);

		/* Configure gyroscope interrupt */
		status += BMI088_WriteGyrRegister(imu, BMI_GYR_INT_CTRL, 0x80);
		HAL_Delay(10);
		status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_CONF, 0x01);
		HAL_Delay(10);
		status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_MAP, 0x01);
		HAL_Delay(10);

		/* Compute conversion factor for gyro data (rad/s) */
		imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f;

		/* Prepare TX buffer for DMA read */
		imu->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;

		return status;
}


/**
 * @brief Initializes the gyroscope and accelerometer biases by averaging a number of samples.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param cycles Number of measurement cycles used to compute the bias.
 */
void Init_BMI088_Bias(BMI088* imu, int cycles)
{
    // Accumulate values over a given number of cycles
    for (int i = 0; i < cycles; i++)
    {
        imu->gyr_bias[0] += imu->gyr_rps[0];
        imu->gyr_bias[1] += imu->gyr_rps[1];
        imu->gyr_bias[2] += imu->gyr_rps[2];
        imu->acc_bias[0] += imu->acc_mps2[0];
        imu->acc_bias[1] += imu->acc_mps2[1];
        imu->acc_bias[2] += imu->acc_mps2[2];
    }

    // Compute the average to obtain the bias
    for (int i = 0; i < 3; i++)
    {
    	imu->gyr_bias[i] = imu->gyr_bias[i] / cycles;
    	imu->acc_bias[i] = imu->acc_bias[i] / cycles;
    }
}

/**
 * @brief Takes IMU measurements from memory (not sensor), applies bias correction and sign adjustment, and stores them in the packet.
 *
 * @param imu Pointer to the BMI088 structure containing the latest sensor values.
 * @param pkt Pointer to the binary packet structure where processed values are stored.
 */
void Take_IMU_Measurements(BMI088 *imu, BinaryPacket *pkt)
{
	pkt->timestamp = HAL_GetTick();  // Save timestamp when values are copied to the packet

	/* Axis and sign correction with bias removal.
	 * These values are taken from memory (not fresh from sensor).
	 * imu->gyr_rps and imu->acc_mps2 hold latest sensor data.
	 * The pkt struct holds processed data for use elsewhere.
	 */

	pkt->gyr[0] = -imu->gyr_rps[1] + imu->gyr_bias[1];  // Apply axis/sign adjustment + bias correction
	pkt->gyr[1] = imu->gyr_rps[0] - imu->gyr_bias[0];
	pkt->gyr[2] = imu->gyr_rps[2] - imu->gyr_bias[2];

	pkt->acc[0] = -imu->acc_mps2[1];  // Apply axis correction (no bias compensation here)
	pkt->acc[1] = imu->acc_mps2[0];
	pkt->acc[2] = imu->acc_mps2[2];
}


/*
 * ===================================================
 * LOW-LEVEL REGISTER FUNCTIONS FOR BMI088 SENSOR
 * ===================================================
 */

/**
 * @brief Reads a register from the accelerometer (BMI088).
 *
 * Accelerometer reads require sending one byte (register address), discarding one dummy byte,
 * and then reading the actual data.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param regAddr Register address to read from.
 * @param data Pointer to store the read byte.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[3] = {regAddr | 0x80, 0x00, 0x00};  // First byte: register address with read bit set
	uint8_t rxBuf[3];  // Buffer to receive dummy + actual data

	// Pull CS low to start SPI communication with accelerometer
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);

	// Transmit register address and receive 3 bytes (dummy + data)
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);

	// Pull CS high to end communication
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	if (status == 1) {
		*data = rxBuf[2];  // The actual data is in the third byte
	}

	return status;
}

/**
 * @brief Reads a register from the gyroscope (BMI088).
 *
 * Gyroscope reads only require sending one address byte and receiving one data byte.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param regAddr Register address to read from.
 * @param data Pointer to store the read byte.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};  // First byte: register address with read bit
	uint8_t rxBuf[2];  // Buffer to receive the response

	// Pull CS low to start SPI communication with gyroscope
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);

	// Transmit register address and receive 2 bytes
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);

	// Pull CS high to end communication
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	if (status == 1) {
		*data = rxBuf[1];  // Actual data is in second byte
	}

	return status;
}

/**
 * @brief Write a value to a register in the accelerometer.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param regAddr Register address to write.
 * @param data Data to write to the register.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);  // CS Low: start SPI transaction
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);  // Wait for SPI to finish
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);  // CS High: end transaction

	return status;
}

/**
 * @brief Write a value to a register in the gyroscope.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param regAddr Register address to write.
 * @param data Data to write to the register.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);  // CS Low
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);  // Wait until SPI is ready
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);  // CS High

	return status;
}


/*
 *
 * POLLING FUNCTIONS
 *
 */

/**
 * @brief Reads raw accelerometer data from BMI088 via polling.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {

	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // First byte: address, followed by dummy/data
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);  // Start communication
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);  // End communication

	// Reconstruct raw 16-bit signed values
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	// Convert to m/s² using scaling factor
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	return status;
}

/**
 * @brief Reads raw gyroscope data from BMI088 via polling.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return uint8_t 1 if successful, 0 otherwise.
 */
uint8_t BMI088_ReadGyroscope(BMI088 *imu) {

	uint8_t txBuf[7] = {(BMI_GYR_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Read address + dummy/data
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);  // Start SPI
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);  // End SPI

	// Reconstruct raw 16-bit signed values
	int16_t gyrX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t gyrY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t gyrZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

	// Convert to rad/s using scaling factor
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

	return status;
}


/*
 *
 * DMA FUNCTIONS
 *
 */

/**
 * @brief Starts DMA-based read of the accelerometer.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return uint8_t 1 if started successfully, 0 otherwise.
 */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu) {

	if (imu->readingAcc)  // Prevent overlapping DMA requests
		return 0;

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);  // Start SPI

	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->accTxBuf, (uint8_t *) imu->accRxBuf, 8) == HAL_OK) {
		imu->readingAcc = 1;
		return 1;
	} else {
		HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);  // SPI failure, set CS high
		return 0;
	}
}

/**
 * @brief Callback to be called upon DMA completion of accelerometer read.
 *
 * @param imu Pointer to the BMI088 structure.
 */
void BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu) {

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);  // End SPI communication
	imu->readingAcc = 0;

	// Convert received bytes to 16-bit signed integers
	int16_t accX = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
	int16_t accY = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);
	int16_t accZ = (int16_t) ((imu->accRxBuf[7] << 8) | imu->accRxBuf[6]);

	// Convert to m/s² with scaling (Cristiano’s x2 multiplier applied)
	imu->acc_mps2[0] = imu->accConversion * accX * 2;
	imu->acc_mps2[1] = imu->accConversion * accY * 2;
	imu->acc_mps2[2] = imu->accConversion * accZ * 2;
}

/**
 * @brief Starts DMA-based read of the gyroscope.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return uint8_t 1 if started successfully, 0 otherwise.
 */
uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu) {

	if (imu->readingGyr)  // Prevent overlapping DMA requests
		return 0;

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);  // Start SPI

	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->gyrTxBuf, (uint8_t *) imu->gyrRxBuf, 7) == HAL_OK) {
		imu->readingGyr = 1;
		return 1;
	} else {
		HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);  // SPI failure
		return 0;
	}
}

/**
 * @brief Callback to be called upon DMA completion of gyroscope read.
 *
 * @param imu Pointer to the BMI088 structure.
 */
void BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu) {

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);  // End SPI communication
	imu->readingGyr = 0;

	// Convert received bytes to 16-bit signed integers
	int16_t gyrX = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
	int16_t gyrY = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
	int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);

	// Convert to rad/s using scaling factor
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;
}

