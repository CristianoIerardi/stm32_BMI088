#include "main.h"
#include "ComputeOrientation.h"

#include <stdint.h>
#include <string.h>  // for memset
#include <math.h>

//----------------------------------------
// EKF parameters & state (Euler-based)
//----------------------------------------
#define EKF_STATE_SIZE   6   // [roll, pitch, yaw, bias_x, bias_y, bias_z]
#define EKF_MEAS_SIZE     2  // [roll_accel, pitch_accel]

// Process noise
static const float Q_angle = 0.001f;
static const float Q_bias  = 0.003f;
// Measurement noise
static const float R_measure = 0.03f;
// Gravity constant
#define GRAVITY       9.80665f
#define ACC_THRESHOLD  0.2f   // m/s^2

// EKF state
static float x_state[EKF_STATE_SIZE];    // [phi, theta, psi, bx, by, bz]
static float P[EKF_STATE_SIZE][EKF_STATE_SIZE];

// Buffers for computation
static float F[EKF_STATE_SIZE][EKF_STATE_SIZE];        // State transition Jacobian
static float H[EKF_MEAS_SIZE][EKF_STATE_SIZE] = {
    {1, 0, 0, 0, 0, 0},   // roll measurement
    {0, 1, 0, 0, 0, 0}    // pitch measurement
};
static float R[EKF_MEAS_SIZE][EKF_MEAS_SIZE] = {
    {R_measure, 0},
    {0, R_measure}
};

// Temporary matrices
static float P_pred[EKF_STATE_SIZE][EKF_STATE_SIZE];
static float S[EKF_MEAS_SIZE][EKF_MEAS_SIZE];
static float K[EKF_STATE_SIZE][EKF_MEAS_SIZE];

//----------------------------------------------------------------------
// Initialize EKF: must call once with initial quaternion
//----------------------------------------------------------------------
void EKF_Init(const Quaternion *q_init) {
    // Convert initial quaternion -> Euler angles
    QuaternionToEuler(*q_init, x_state);
    // Initialize biases to zero
    x_state[3] = x_state[4] = x_state[5] = 0.0f;

    // Initialize covariance P to small values
    memset(P, 0, sizeof(P));
    for (int i = 0; i < 3; i++) {
        P[i][i] = 0.01f;   // angle uncertainty
    }
    for (int i = 3; i < EKF_STATE_SIZE; i++) {
        P[i][i] = 0.01f;   // bias uncertainty
    }
}

//----------------------------------------------------------------------
// EKF predict + update using gyro and accel
//----------------------------------------------------------------------
void EKF_UpdateIMU(const Vector3 gyro, const Vector3 acc, float dt, Quaternion *q_out) {
    // 1) Predict step
    //float phi   = x_state[0];
    //float theta = x_state[1];
    //float psi   = x_state[2];
    float phi   = 0.1;
	float theta = 0.1;
	float psi   = 0.1;
    // Biases
    float bx = x_state[3];
    float by = x_state[4];
    float bz = x_state[5];

    // State prediction
    float omega_x = gyro.x - bx;
    float omega_y = gyro.y - by;
    float omega_z = gyro.z - bz;

    x_state[0] += omega_x * dt;
    x_state[1] += omega_y * dt;
    x_state[2] += omega_z * dt;
    // biases remain constant

    // Build Jacobian F = d f / d x
    // F = I + dt * [ 0  0  0 -1  0  0;
    //                0  0  0  0 -1  0;
    //                0  0  0  0  0 -1;
    //                ... identity for biases]
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        F[i][i] = 1.0f;
    }
    F[0][3] = -dt;
    F[1][4] = -dt;
    F[2][5] = -dt;

    // P_pred = F * P * F^T + Q
    // First compute P_pred = F * P
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            P_pred[i][j] = 0;
            for (int k = 0; k < EKF_STATE_SIZE; k++) {
                P_pred[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    // Then P = P_pred * F^T + Q
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            float sum = 0;
            for (int k = 0; k < EKF_STATE_SIZE; k++) {
                sum += P_pred[i][k] * F[j][k];
            }
            // add process noise
            if (i == j) {
                if (i < 3)      sum += Q_angle * dt;
                else            sum += Q_bias  * dt;
            }
            P[i][j] = sum;
        }
    }

    // 2) Measurement update if accel is stable
    // Normalize accel vector
    float acc_norm = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    if (fabsf(acc_norm - GRAVITY) < ACC_THRESHOLD) {
        // Compute roll and pitch from accel
        float roll_acc = atan2f(acc.y, acc.z) * RAD_TO_DEG;
        float pitch_acc = atan2f(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z)) * RAD_TO_DEG;
        float z[2] = { roll_acc, pitch_acc };

        // S = H * P * H^T + R (2x2)
        for (int i = 0; i < EKF_MEAS_SIZE; i++) {
            for (int j = 0; j < EKF_MEAS_SIZE; j++) {
                float sum = 0;
                for (int k = 0; k < EKF_STATE_SIZE; k++) {
                    sum += H[i][k] * P[k][k] * H[j][k];
                }
                S[i][j] = sum + R[i][j];
            }
        }
        // Compute Kalman gain K = P * H^T * inv(S)
        // First compute P*H^T (6x2)
        for (int i = 0; i < EKF_STATE_SIZE; i++) {
            for (int j = 0; j < EKF_MEAS_SIZE; j++) {
                float sum = 0;
                for (int k = 0; k < EKF_STATE_SIZE; k++) {
                    sum += P[i][k] * H[j][k];
                }
                K[i][j] = sum;
            }
        }
        // Invert 2x2 S
        float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        float invS[2][2] = {
            { S[1][1]/det, -S[0][1]/det },
            { -S[1][0]/det, S[0][0]/det }
        };
        // K = (P*H^T) * inv(S)
        for (int i = 0; i < EKF_STATE_SIZE; i++) {
            float k0 = K[i][0], k1 = K[i][1];
            K[i][0] = k0*invS[0][0] + k1*invS[1][0];
            K[i][1] = k0*invS[0][1] + k1*invS[1][1];
        }

        // Update state: x = x + K * (z - H*x)
        for (int i = 0; i < EKF_STATE_SIZE; i++) {
            float y_err = 0;
            if (i < 2) {
                // Only first two states have measurement
                float hx = x_state[i];
                y_err = z[i] - hx;
            }
            x_state[i] += K[i][0] * y_err + K[i][1] * ((i==1)?(z[1] - x_state[1]):0);
        }

        // Update covariance P = (I - K*H) * P
        float KH[EKF_STATE_SIZE][EKF_STATE_SIZE];
        memset(KH, 0, sizeof(KH));
        for (int i = 0; i < EKF_STATE_SIZE; i++) {
            for (int j = 0; j < EKF_STATE_SIZE; j++) {
                for (int k = 0; k < EKF_MEAS_SIZE; k++) {
                    KH[i][j] += K[i][k] * H[k][j];
                }
            }
        }
        for (int i = 0; i < EKF_STATE_SIZE; i++) {
            for (int j = 0; j < EKF_STATE_SIZE; j++) {
                P[i][j] = (i==j ? 1.0f : 0.0f) - KH[i][j];
                // then P = (I-KH)*P_old; approximated by writing into P directly
            }
        }
    }

    // 3) Output quaternion from updated Euler angles
    SetQuaternionFromEuler(q_out, x_state[0], x_state[1], x_state[2]);
    NormalizeQuaternion(q_out);
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
