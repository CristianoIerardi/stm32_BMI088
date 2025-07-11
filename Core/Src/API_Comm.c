/**
 * @file API_Comm.c
 * @author CriIera
 * @brief Script to display data on API: https://github.com/Reefwing-Software/Reefwing-xIMU3
 */

//#ifdef USE_API

/// Includes

#include <stdint.h>                 // Standard integer types
#include "stm32f4xx_hal.h"         // STM32 HAL library
#include <string.h>                // String handling functions
#include "usbd_cdc_if.h"           // USB CDC interface

#include <stdlib.h>                // For atoi, strtoul
#include <stdio.h>                 // For sprintf, printf

#include "API_Comm.h"              // Header file for this source
//#include <utils.h>               // Optional utilities

extern uint32_t SAMPLE_TIME_MS_TOGGLE;  // Global variable to hold sampling interval

/**
 * @brief Function to display on API the Roll, Pitch, Yaw
 * @param angle [0]: Roll, [1]: Pitch, [2]: Yaw
 */
/*void API_PrintAngles(uint32_t timestamp, float* angle)
{
    // Buffer to hold formatted string
	char txBuff[128];

    // Format: A,<timestamp>,<roll>,<pitch>,<yaw>\r\n
	sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\n", timestamp, angle[0], angle[1], angle[2]);

    // Transmit formatted string over USB CDC
	//while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
	CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
}*/

/**
 * @brief Function to display on API the gyroscope and accelerometer values
 * @param gyro array of gyroscope measurements --> gyroX, gyroY, gyroZ
 * @param accel array of accelerometer measurements --> accelX, accelY, accelZ
 */
/*void API_SendInertial(uint32_t timestamp, float* gyro, float* accel)
{
    // Buffer to hold formatted string
	char txBuff[128];

    // Format: I,<timestamp>,<gyroX>,<gyroY>,<gyroZ>,<accX>,<accY>,<accZ>\r\n
	sprintf(txBuff, "I,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);

    // Transmit formatted string over USB CDC
	//while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
	CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
}*/

/**
 * @brief Checks whether a string contains only numeric digits.
 * @param s Pointer to null-terminated string to check.
 * @return 1 if the string is a valid number, 0 otherwise.
 */
uint8_t is_valid_number(const char *s)
{
    // Check for NULL or empty string
    if (s == NULL || *s == '\0')
    	return 0;

    // Iterate through each character to verify it's a digit
    for (int i = 0; s[i] != '\0'; i++) {
        if (s[i] < '0' || s[i] > '9')
        	return 0;
    }

    return 1;
}

/**
 * @brief Handles a received string, parses it as an integer, and updates the sampling time.
 * @param str Pointer to the received null-terminated string.
 */
void HandleReceivedString(char *str)
{
	//char rxBuff[64];  // Buffer for optional debug response

    // Remove possible trailing \r or \n characters
    char *clean_str = str;

    // Trim line endings
    for (int i = 0; clean_str[i] != '\0'; i++) {
        if (clean_str[i] == '\r' || clean_str[i] == '\n') {
            clean_str[i] = '\0';
            break;
        }
    }

    // Validate that input is numeric
	if (!is_valid_number(clean_str)) {
		//sprintf(rxBuff, "Invalid input");
		//CDC_Transmit_FS((uint8_t *) rxBuff, strlen(rxBuff));
		return;
	}

    // Convert string to unsigned integer
    uint32_t value = (uint32_t)strtoul(clean_str, NULL, 10);

    // Check that value is in acceptable range
    if (value >= 0 && value <= 5000)
    {
        uint32_t parsed_value = value;
        //sprintf(rxBuff, "ParsVal: %i", parsed_value);
        //CDC_Transmit_FS((uint8_t *) rxBuff, strlen(rxBuff));

        // Update global sampling time
        SAMPLE_TIME_MS_TOGGLE = parsed_value;
    }
    else
    {
    	//sprintf(rxBuff, "Parsing error");
    	//CDC_Transmit_FS((uint8_t *) rxBuff, strlen(rxBuff));
    }
}

//#endif //USE_API

/// End of file
