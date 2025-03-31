/*
 * ximu3_wrapper.cpp
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#include "Reefwing_xIMU3.h"  // including C++ library
#include "ximu3_wrapper.h"       // including wrapper for C++ library

// It allows the use of C++ functions
extern "C" {

    // Instances of ximu3 class
    static Reefwing_xIMU3_GUI imu3;

    // Funcion for serial communication
    void xIMU3_init() {
        imu3.begin(Serial);  // Be sure that the serial is initialized in main
    }

    // Funcion to send data: roll, pitch, yaw, timestamp
    void xIMU3_sendEulerAngles(float roll, float pitch, float yaw) {
        EulerAngles euler;
        euler.roll = roll;
        euler.pitch = pitch;
        euler.yaw = yaw;
        euler.timeStamp = micros();  // Timestamp in microseconds

        imu3.sendEulerAngles(euler);  // Send data to GUI
    }
}


