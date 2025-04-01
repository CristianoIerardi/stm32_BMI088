/*
 * Serial_Comm.c
 *
 *  Created on: Mar 31, 2025
 *      Author: crist
 */

#ifdef USE_SERIAL

#include "Serial_Comm.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

char txBuff[128];


// It prints Roll, Pitch and Yaw
void Serial_PrintAngles(float r, float p, float y)
{
	sprintf(txBuff, "Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\r\n", r, p, y);
    while(CDC_Transmit_FS((uint8_t *) buffer, strlen(buffer)) == HAL_BUSY);

}


void Serial_PrintAcc(float aX, float aY, float aZ)
{
	sprintf(txBuff, "aX=%.3f,\taY=%.3f,\taZ=%.3f \r\n", aX, aY, aZ);
	CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
}


void Serial_PrintGyr(float gX, float gY, float gZ)
{
	sprintf(txBuff, "gX=%.3f,\tgY=%.3f,\tgZ=%.3f \r\n", gX, gY, gZ);
	CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
}


void Serial_GyroBias(float biasX, float biasY, float biasZ)
{
sprintf(txBuff, "BIAS: X=%.2f°, Y=%.2f°, Z=%.2f°\r\n", biasX, biasY, biasZ);
while(CDC_Transmit_FS((uint8_t *) buffer, strlen(buffer)) == HAL_BUSY);


}


#endif //USE_SERIAL
