/*
 * API_Comm.c
 *
 *  Created on: Mar 31, 2025
 *      Author: crist
 */
//#define USE_API
#ifdef USE_API

#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "API_Comm.h"


char txBuff[128];



void API_PrintAngles(uint32_t timestamp, float r, float p, float y)
{
	sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\n", timestamp, r, p, y);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}





#endif //USE_API
