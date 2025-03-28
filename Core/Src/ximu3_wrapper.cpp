/*
 * ximu3_wrapper.cpp
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#include "Reefwing_xIMU3.h"  // Include la libreria C++
#include "ximu3_wrapper.h"       // Include il wrapper per il codice C

// Permette l'uso delle funzioni in C
extern "C" {

    // Istanza della classe xIMU3
    static Reefwing_xIMU3_GUI imu3;

    // Funzione per inizializzare la comunicazione seriale
    void xIMU3_init() {
        imu3.begin(Serial);  // Assicurati che Serial sia inizializzato in main.c
    }

    // Funzione per inviare i dati di Roll, Pitch e Yaw
    void xIMU3_sendEulerAngles(float roll, float pitch, float yaw) {
        EulerAngles euler;
        euler.roll = roll;
        euler.pitch = pitch;
        euler.yaw = yaw;
        euler.timeStamp = micros();  // Timestamp in microsecondi

        imu3.sendEulerAngles(euler);  // Invia i dati alla GUI
    }
}


