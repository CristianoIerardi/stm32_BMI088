/*
 * ximu3_wrapper.h
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#ifndef XIMU3_WRAPPER_H
#define XIMU3_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void xIMU3_init();
void xIMU3_sendEulerAngles(float roll, float pitch, float yaw);

#ifdef __cplusplus
}
#endif

#endif
