/**
 * @file Serial_Comm.c
 * @author CriIera
 * @brief Script to display data with USB communication
 */

//#ifdef USE_SERIAL

/// Includes

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include "usbd_cdc_if.h"

#include "Serial_Comm.h"


/**
 * @brief Function to display on serial the Roll, Pitch, Yaw
 * @param r Roll
 * @param r Pitch
 * @param r Yaw
 */
void Serial_PrintAngles(float r, float p, float y)
{
	sprintf(txBuff, "Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\r\n", r, p, y);
    while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}


/**
 * @brief Function to display on serial the acceleration vector
 * @param aX acceleration X axis
 * @param aY acceleration Y axis
 * @param aZ acceleration Z axis
 */
void Serial_PrintAcc(float aX, float aY, float aZ)
{
	sprintf(txBuff, "aX=%.3f,\taY=%.3f,\taZ=%.3f \r\n", aX, aY, aZ);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}

/**
 * @brief Function to display on serial the gyroscope vector
 * @param gX gyroscope X axis
 * @param gY gyroscope Y axis
 * @param gZ gyroscope Z axis
 */
void Serial_PrintGyr(float gX, float gY, float gZ)
{
	sprintf(txBuff, "gX=%.3f,\tgY=%.3f,\tgZ=%.3f \r\n", gX, gY, gZ);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}

/**
 * @brief Function to display on serial the bias gyroscope vector
 * @param biasX X axis
 * @param biasY Y axis
 * @param biasZ Z axis
 */
void Serial_GyroBias(float biasX, float biasY, float biasZ)
{
	sprintf(txBuff, "BIAS: X=%.2f°, Y=%.2f°, Z=%.2f°\r\n", biasX, biasY, biasZ);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}


//#endif //USE_SERIAL


/// End of file
