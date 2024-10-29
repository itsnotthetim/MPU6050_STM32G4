#ifndef KALMAN_MODEL_H
#define KALMAN_MODEL_H

#include "arm_math.h"

// Define the dimensions of the matrices
#define STATE_DIM 6         // [phi; theta; psi; bias_phi; bias_theta; bias_psi]
#define MEASUREMENT_DIM 6   // [phi_acc; theta_acc; psi_mag; omega_phi; omega_theta; omega_psi]

// Sampling time (adjust based on your sampling rate)
extern float32_t dt;

// Kalman Filter structure
typedef struct {
    // State vector x_k: [phi; theta; psi; bias_phi; bias_theta; bias_psi]
    arm_matrix_instance_f32 x_k;
    float32_t x_k_data[STATE_DIM];

    // Error covariance matrix P_k
    arm_matrix_instance_f32 P_k;
    float32_t P_k_data[STATE_DIM * STATE_DIM];

    // State transition matrix A
    arm_matrix_instance_f32 A;
    float32_t A_data[STATE_DIM * STATE_DIM];

    // Control input matrix B
    arm_matrix_instance_f32 B;
    float32_t B_data[STATE_DIM * 3]; // Control input is a 3-element vector

    // Process noise covariance Q
    arm_matrix_instance_f32 Q;
    float32_t Q_data[STATE_DIM * STATE_DIM];

    // Measurement matrix H
    arm_matrix_instance_f32 H;
    float32_t H_data[MEASUREMENT_DIM * STATE_DIM];

    // Measurement noise covariance R
    arm_matrix_instance_f32 R;
    float32_t R_data[MEASUREMENT_DIM * MEASUREMENT_DIM];

    // Kalman gain K_k
    arm_matrix_instance_f32 K_k;
    float32_t K_k_data[STATE_DIM * MEASUREMENT_DIM];

    // Measurement vector z_k: [phi_acc; theta_acc; psi_mag; omega_phi; omega_theta; omega_psi]
    arm_matrix_instance_f32 z_k;
    float32_t z_k_data[MEASUREMENT_DIM];

    // Control input vector u_k
    arm_matrix_instance_f32 u_k;
    float32_t u_k_data[3]; // Control inputs for roll, pitch, yaw

    // Identity matrix I
    arm_matrix_instance_f32 I;
    float32_t I_data[STATE_DIM * STATE_DIM];

    // Temporary matrices for calculations
    arm_matrix_instance_f32 temp1;
    float32_t temp1_data[STATE_DIM * STATE_DIM];

    arm_matrix_instance_f32 temp2;
    float32_t temp2_data[STATE_DIM * STATE_DIM];

    arm_matrix_instance_f32 temp3;
    float32_t temp3_data[MEASUREMENT_DIM * MEASUREMENT_DIM];

    arm_matrix_instance_f32 temp4;
    float32_t temp4_data[STATE_DIM * MEASUREMENT_DIM];

    arm_matrix_instance_f32 temp5;
    float32_t temp5_data[MEASUREMENT_DIM * STATE_DIM];

    arm_matrix_instance_f32 temp6;
    float32_t temp6_data[STATE_DIM * 1];

    arm_matrix_instance_f32 temp7;
    float32_t temp7_data[MEASUREMENT_DIM * 1];

    // Add more temp matrices if needed

} KalmanFilter;

// Function prototypes
void KalmanFilter_Init(KalmanFilter *kf, float32_t delta_t);
void KalmanFilter_Update(KalmanFilter *kf, float32_t phi_acc, float32_t theta_acc, float32_t psi_mag, float32_t omega_phi, float32_t omega_theta, float32_t omega_psi);
void ComputeControlInput(float32_t phi_i, float32_t theta_i, float32_t psi_i, float32_t omega_phi_i, float32_t omega_theta_i, float32_t omega_psi_i, float32_t *u_i1);

#endif // KALMAN_ANGLE_H
