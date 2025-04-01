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


/**
 * @brief Function to display on serial the Roll, Pitch, Yaw
 * @param r Roll
 * @param r Pitch
 * @param r Yaw
 */
void API_PrintAngles(uint32_t timestamp, float r, float p, float y)
{
	sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\n", timestamp, r, p, y);
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
}





//#endif //USE_API

/// End of file
