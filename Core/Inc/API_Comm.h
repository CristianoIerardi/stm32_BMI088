/**
 * @file API_Comm.c
 * @author CriIera
 * @brief Script to display data on API: https://github.com/Reefwing-Software/Reefwing-xIMU3
 *
 * This file contains functions to format and send sensor data (angles, gyroscope,
 * and accelerometer) to an external API such as x-IMU3.
 */

//#define USE_API                ///< Optional macro for conditional compilation
#ifndef INC_API_COMM_H_        ///< Include guard start
#define INC_API_COMM_H_

//#ifdef USE_API                ///< Optional macro for conditional inclusion

/// @brief Sends roll, pitch, and yaw angle data to the API.
/// @param timestamp Time in milliseconds.
/// @param angle Pointer to an array of 3 floats: [Roll, Pitch, Yaw].
void API_PrintAngles(uint32_t timestamp, float* angle);

/// @brief Sends gyroscope and accelerometer data to the API.
/// @param timestamp Time in milliseconds.
/// @param gyro Pointer to an array of 3 floats: [GyroX, GyroY, GyroZ].
/// @param accel Pointer to an array of 3 floats: [AccelX, AccelY, AccelZ].
void API_SendInertial(uint32_t timestamp, float* gyro, float* accel);

/// @brief Parses and processes a received string representing a number.
/// @param str Null-terminated string received via communication (e.g. USB).
void HandleReceivedString(char *str);

/// @brief Checks if a string is a valid positive integer.
/// @param s Pointer to a null-terminated string.
/// @return 1 if the string contains only digits, 0 otherwise.
uint8_t is_valid_number(const char *s);

//#endif //USE_API              ///< Optional macro for conditional compilation end

#endif /* INC_API_COMM_H_ */  ///< Include guard end

/// End of file
