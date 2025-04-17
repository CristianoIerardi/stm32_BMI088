/*
 * ComputeOrientation.h
 *
 *  Created on: Apr 2, 2025
 *      Author: crist
 */

#ifndef INC_COMPUTEORIENTATION_H_
#define INC_COMPUTEORIENTATION_H_

// Struttura per il quaternione
/*typedef union {
	float array[4];

	struct {
		float w, x, y, z;
	} value;
} Quaternion;

// Funzione per aggiornare il quaternione con il giroscopio
typedef union {
    float array[3];

    struct {
        float x, y, z;
    } axis;
} Vector3;*/


#define RAD_TO_DEG 	57.2957795f  // Conversione da radianti a gradi
#define DEG_TO_RAD 	0.0174533f

typedef struct {
    float w, x, y, z;
} Quaternion;


typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    float roll, pitch, yaw;
} EulerAngles;



// Set angles to a specified quantity
void SetQuaternionFromEuler(Quaternion *q, float roll, float pitch, float yaw);
// Reset angles
void ResetAngles(float* angles, float roll, float pitch, float yaw);
// Function to normalize a generic quaternion
void NormalizeQuaternion(Quaternion *q);
// Update the rotation
void UpdateQuaternion(Quaternion *q, Vector3 gyro, float dt);
// Correction with accelerometer
void CorrectQuaternionWithAccel(Quaternion *q, Vector3 accel, float alpha);
// Conversion from quaternion to euler angles
void QuaternionToEuler(Quaternion q, float* ang);














#endif /* INC_COMPUTEORIENTATION_H_ */
