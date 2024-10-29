//#ifndef KALMAN_ANGLE_H
//#define KALMAN_ANGLE_H
//
//#include "arm_math.h"
//
//// Define the dimensions of the matrices
//#define STATE_DIM 2         // [theta; bias]
//#define MEASUREMENT_DIM 2   // [theta_acc; omega_meas]
//
//// Sampling time (adjust based on your sampling rate)
//extern float32_t dt;
//
//// Kalman Filter structure
//typedef struct {
//    // State vector x_k: [theta; bias]
//    arm_matrix_instance_f32 x_k;
//    float32_t x_k_data[STATE_DIM];
//
//    // Error covariance matrix P_k
//    arm_matrix_instance_f32 P_k;
//    float32_t P_k_data[STATE_DIM * STATE_DIM];
//
//    // State transition matrix A
//    arm_matrix_instance_f32 A;
//    float32_t A_data[STATE_DIM * STATE_DIM];
//
//    // Control input matrix B
//    arm_matrix_instance_f32 B;
//    float32_t B_data[STATE_DIM * 1];
//
//    // Process noise covariance Q
//    arm_matrix_instance_f32 Q;
//    float32_t Q_data[STATE_DIM * STATE_DIM];
//
//    // Measurement matrix H
//    arm_matrix_instance_f32 H;
//    float32_t H_data[MEASUREMENT_DIM * STATE_DIM];
//
//    // Measurement noise covariance R
//    arm_matrix_instance_f32 R;
//    float32_t R_data[MEASUREMENT_DIM * MEASUREMENT_DIM];
//
//    // Kalman gain K_k
//    arm_matrix_instance_f32 K_k;
//    float32_t K_k_data[STATE_DIM * MEASUREMENT_DIM];
//
//    // Measurement vector z_k: [theta_acc; omega_meas]
//    arm_matrix_instance_f32 z_k;
//    float32_t z_k_data[MEASUREMENT_DIM];
//
//    // Control input vector u_k
//    arm_matrix_instance_f32 u_k;
//    float32_t u_k_data[1];
//
//    // Identity matrix I
//    arm_matrix_instance_f32 I;
//    float32_t I_data[STATE_DIM * STATE_DIM];
//
//    // Temporary matrices for calculations
//    arm_matrix_instance_f32 temp1;
//    float32_t temp1_data[STATE_DIM * STATE_DIM];
//
//    arm_matrix_instance_f32 temp2;
//    float32_t temp2_data[STATE_DIM * STATE_DIM];
//
//    arm_matrix_instance_f32 temp3;
//    float32_t temp3_data[MEASUREMENT_DIM * MEASUREMENT_DIM];
//
//    arm_matrix_instance_f32 temp4;
//    float32_t temp4_data[STATE_DIM * MEASUREMENT_DIM];
//
//    arm_matrix_instance_f32 temp5;
//    float32_t temp5_data[MEASUREMENT_DIM * STATE_DIM];
//
//    arm_matrix_instance_f32 temp6;
//    float32_t temp6_data[STATE_DIM * 1];
//
//    arm_matrix_instance_f32 temp7;
//    float32_t temp7_data[MEASUREMENT_DIM * 1];
//
//    arm_matrix_instance_f32 temp8;
//    float32_t temp8_data[STATE_DIM * MEASUREMENT_DIM];
//} KalmanFilter;
//
//// Function prototypes
//void KalmanFilter_Init(KalmanFilter *kf, float32_t delta_t);
//void KalmanFilter_Update(KalmanFilter *kf, float32_t theta_acc, float32_t omega_meas);
//void ComputeControlInput(float32_t theta_i, float32_t omega_i, float32_t *u_i1);
//
//#endif // KALMAN_ANGLE_H
