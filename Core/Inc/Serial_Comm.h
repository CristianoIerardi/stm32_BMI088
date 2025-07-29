/**
 * @file Serial_Comm.h
 * @author CriIera
 * @brief Script to display data with USB communication
 */

#ifndef INC_SERIAL_COMM_H_
#define INC_SERIAL_COMM_H_

//#ifdef USE_SERIAL

/**
 * @brief Transmission buffer used for USB serial communication.
 */
extern char txBuff[128];

/// Function Declarations

/**
 * @brief Print the orientation angles (roll, pitch, yaw) over USB serial.
 *
 * @param r Roll angle in degrees
 * @param p Pitch angle in degrees
 * @param y Yaw angle in degrees
 */
void Serial_PrintAngles(float r, float p, float y);

/**
 * @brief Print the accelerometer data over USB serial.
 *
 * @param aX Acceleration on X axis (g)
 * @param aY Acceleration on Y axis (g)
 * @param aZ Acceleration on Z axis (g)
 */
void Serial_PrintAcc(float aX, float aY, float aZ);

/**
 * @brief Print the gyroscope data over USB serial.
 *
 * @param gX Angular velocity on X axis (rad/s)
 * @param gY Angular velocity on Y axis (rad/s)
 * @param gZ Angular velocity on Z axis (rad/s)
 */
void Serial_PrintGyr(float gX, float gY, float gZ);

/**
 * @brief Print the calculated gyroscope bias over USB serial.
 *
 * @param biasX Bias on X axis
 * @param biasY Bias on Y axis
 * @param biasZ Bias on Z axis
 */
void Serial_GyroBias(float biasX, float biasY, float biasZ);

//#endif //USE_SERIAL

#endif /* INC_SERIAL_COMM_H_ */

/// End of file
