
/**
 * @file API_Comm.h
 * @author CriIera
 * @brief Script to display data on API: https://github.com/Reefwing-Software/Reefwing-xIMU3
 */

//#define USE_API
#ifndef INC_API_COMM_H_
#define INC_API_COMM_H_

//#ifdef USE_API

extern char txBuff[128];


/// Function Declarations
void API_PrintAngles(uint32_t timestamp, float* angle);

void API_SendInertial(uint32_t timestamp, float* gyro, float* accel);



//#endif //USE_API

#endif /* INC_API_COMM_H_ */

/// End of file
