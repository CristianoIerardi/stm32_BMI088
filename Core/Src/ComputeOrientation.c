/*
 * ComputeOrientation.c
 *
 *  Created on: Apr 2, 2025
 *      Author: crist
 */


#include "main.h"
#include "ComputeOrientation.h"
#include "EKF.h"
#include <stdint.h> 	// → It defines uint32_t e uint8_t
#include <math.h> 		// → Needed for atan2f and sqrtf



#include <math.h>
#include <stdint.h>


float correction_factor = 0.0f;


// Set angles to a specified quantity
void SetQuaternionFromEuler(Quaternion *q, float roll, float pitch, float yaw) {
	roll = roll * DEG_TO_RAD;
	pitch = pitch * DEG_TO_RAD;
	yaw = yaw* DEG_TO_RAD;
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}




// Function to normalize a generic quaternion
void NormalizeQuaternion(Quaternion *q) {
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}


// Update the rotation
void UpdateQuaternion(Quaternion *q, Vector3 gyro, float dt) {
    float gx = gyro.x * 0.5f * dt;
    float gy = gyro.y * 0.5f * dt;
    float gz = gyro.z * 0.5f * dt;

    Quaternion q_dot = {
        -q->x * gx - q->y * gy - q->z * gz,
         q->w * gx + q->y * gz - q->z * gy,
         q->w * gy - q->x * gz + q->z * gx,
         q->w * gz + q->x * gy - q->y * gx
    };

    q->w += q_dot.w;
    q->x += q_dot.x;
    q->y += q_dot.y;
    q->z += q_dot.z;

    NormalizeQuaternion(q);
}

// Correction with accelerometer
void CorrectQuaternionWithAccel(Quaternion *q, Vector3 accel, float alpha) {
	// (alpha not used right now)
    Vector3 gravity = { 2.0f * (q->x * q->z - q->w * q->y),
                        2.0f * (q->w * q->x + q->y * q->z),
                        q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z };

    Vector3 error = { accel.y * gravity.z - accel.z * gravity.y,
                      accel.z * gravity.x - accel.x * gravity.z,
                      accel.x * gravity.y - accel.y * gravity.x };

	correction_factor = 0.0f;
    q->x += correction_factor * error.x;
	q->y += correction_factor * error.y;
	q->z += correction_factor * error.z;

    NormalizeQuaternion(q);
}



// Conversion from quaternion to euler angles
void QuaternionToEuler(Quaternion q, float* ang) {
    //EulerAngles angles;

    // Roll (X-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    //angles.roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    ang[0] = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        ang[1] = copysign(90.0f, sinp); // Evita errori numerici, blocco di gimbal lock
    else
        ang[1] = asin(sinp) * RAD_TO_DEG;

    // Yaw (Z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    ang[2] = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;


    /*ang[0] = angles.roll;
    ang[1] = angles.pitch;
    ang[2] = angles.yaw;*/
}






