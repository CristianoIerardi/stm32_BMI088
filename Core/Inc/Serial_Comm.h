/**
 * @file Serial_Comm.h
 * @author CriIera
 * @brief Script to display data with USB communication
 */

#ifndef INC_SERIAL_COMM_H_
#define INC_SERIAL_COMM_H_

//#ifdef USE_SERIAL
extern char txBuff[128];


/// Function Declarations
void Serial_PrintAngles(float r, float p, float y);

void Serial_PrintAcc(float aX, float aY, float aZ);

void Serial_PrintGyr(float gX, float gY, float gZ);

void Serial_GyroBias(float biasX, float biasY, float biasZ);



//#endif //USE_SERIAL

#endif /* INC_SERIAL_COMM_H_ */

/// End of file
