/*
 * Serial_Comm.h
 *
 *  Created on: Mar 31, 2025
 *      Author: crist
 */

#ifndef INC_SERIAL_COMM_H_
#define INC_SERIAL_COMM_H_

#ifdef USE_SERIAL



extern char txBuff[128];

// It prints Roll, Pitch and Yaw
void Serial_PrintAngles(float r, float p, float y);

void Serial_PrintAcc(float aX, float aY, float aZ);

void Serial_PrintGyr(float gX, float gY, float gZ);

void Serial_GyroBias(float biasX, float biasY, float biasZ);



#endif //USE_SERIAL

#endif /* INC_SERIAL_COMM_H_ */
