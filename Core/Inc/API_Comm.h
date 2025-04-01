/*
 * API_Comm.h
 *
 *  Created on: Mar 31, 2025
 *      Author: crist
 */

//#define USE_API
#ifndef INC_API_COMM_H_
#define INC_API_COMM_H_


#ifdef USE_API


extern char txBuff[128];


void API_PrintAngles(uint32_t timestamp, float r, float p, float y);



#endif //USE_API

#endif /* INC_API_COMM_H_ */
