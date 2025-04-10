/**
 * @file API_Comm.c
 * @author CriIera
 * @brief Script to display data on API: https://github.com/Reefwing-Software/Reefwing-xIMU3
 */


//#ifdef USE_API

/// Includes

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include "usbd_cdc_if.h"

#include "API_Comm.h"
//#include <utils.h>

/**
 * @brief Function to display on API the Roll, Pitch, Yaw
 * @param angle [0]: Roll, [1]: Pitch, [2]: Yaw
 */
void API_PrintAngles(uint32_t timestamp, float* angle)
{
	char txBuff[128];
	sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\n", timestamp, angle[0], angle[1], angle[2]);
	/*------------------------------*/
	/* sprintf made by programmer
	 * char tmp[20];
	uint16_t idx = 0;

	// "A,"
	memcpy(&txBuff[idx], "A,", 2); idx += 2;

	// timestamp
	ultoa(timestamp, tmp, 10);
	uint16_t len = strlen(tmp);
	memcpy(&txBuff[idx], tmp, len); idx += len;

	// ","
	txBuff[idx++] = ',';

	// angle[0]
	ftoa(angle[0], tmp, 4);
	len = strlen(tmp);
	memcpy(&txBuff[idx], tmp, len); idx += len;
	txBuff[idx++] = ',';

	// angle[1]
	ftoa(angle[1], tmp, 4);
	len = strlen(tmp);
	memcpy(&txBuff[idx], tmp, len); idx += len;
	txBuff[idx++] = ',';

	// angle[2]
	ftoa(angle[2], tmp, 4);
	len = strlen(tmp);
	memcpy(&txBuff[idx], tmp, len); idx += len;

	// "\r\n"
	txBuff[idx++] = '\r';
	txBuff[idx++] = '\n';

	// Trasmetti
	CDC_Transmit_FS((uint8_t *) txBuff, idx);*/
	/*------------------------------*/
	CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
}

/**
 * @brief Function to display on API the gyroscope and accelerometer values
 * @param gyro array of gyroscope measurements --> gyroX, gyroY, gyroZ
 * @param accel array of accelerometer measurements --> accelX, accelY, accelZ
 */
void API_SendInertial(uint32_t timestamp, float* gyro, float* accel)
{
	char txBuff[128];
	sprintf(txBuff, "I,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}




//#endif //USE_API

/// End of file
