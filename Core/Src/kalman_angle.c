//#include "kalman_angle.h"
//#include <math.h>
//
//// Sampling time (adjust based on your sampling rate)
//float32_t dt;
//
//// Function to initialize the Kalman Filter
//void KalmanFilter_Init(KalmanFilter *kf, float32_t delta_t)
//{
//    dt = delta_t;
//
//    // Initialize matrix instances
//    arm_mat_init_f32(&kf->x_k, STATE_DIM, 1, kf->x_k_data);
//    arm_mat_init_f32(&kf->P_k, STATE_DIM, STATE_DIM, kf->P_k_data);
//    arm_mat_init_f32(&kf->A, STATE_DIM, STATE_DIM, kf->A_data);
//    arm_mat_init_f32(&kf->B, STATE_DIM, 1, kf->B_data);
//    arm_mat_init_f32(&kf->Q, STATE_DIM, STATE_DIM, kf->Q_data);
//    arm_mat_init_f32(&kf->H, MEASUREMENT_DIM, STATE_DIM, kf->H_data);
//    arm_mat_init_f32(&kf->R, MEASUREMENT_DIM, MEASUREMENT_DIM, kf->R_data);
//    arm_mat_init_f32(&kf->K_k, STATE_DIM, MEASUREMENT_DIM, kf->K_k_data);
//    arm_mat_init_f32(&kf->z_k, MEASUREMENT_DIM, 1, kf->z_k_data);
//    arm_mat_init_f32(&kf->u_k, 1, 1, kf->u_k_data);
//    arm_mat_init_f32(&kf->I, STATE_DIM, STATE_DIM, kf->I_data);
//    arm_mat_init_f32(&kf->temp1, STATE_DIM, STATE_DIM, kf->temp1_data);
//    arm_mat_init_f32(&kf->temp2, STATE_DIM, STATE_DIM, kf->temp2_data);
//    arm_mat_init_f32(&kf->temp3, MEASUREMENT_DIM, MEASUREMENT_DIM, kf->temp3_data);
//    arm_mat_init_f32(&kf->temp4, STATE_DIM, MEASUREMENT_DIM, kf->temp4_data);
//    arm_mat_init_f32(&kf->temp5, MEASUREMENT_DIM, STATE_DIM, kf->temp5_data);
//    arm_mat_init_f32(&kf->temp6, STATE_DIM, 1, kf->temp6_data);
//    arm_mat_init_f32(&kf->temp7, MEASUREMENT_DIM, 1, kf->temp7_data);
//    arm_mat_init_f32(&kf->temp8, STATE_DIM, MEASUREMENT_DIM, kf->temp8_data);
//
//    // Initialize the identity matrix I
//    kf->I_data[0] = 1.0f; kf->I_data[1] = 0.0f;
//    kf->I_data[2] = 0.0f; kf->I_data[3] = 1.0f;
//
//    // State transition matrix A
//    kf->A_data[0] = 1.0f;      kf->A_data[1] = -dt;
//    kf->A_data[2] = 0.0f;      kf->A_data[3] = 1.0f;
//
//    // Control input matrix B
//    kf->B_data[0] = dt;    // B[0][0] = dt
//    kf->B_data[1] = 0.0f;  // B[1][0] = 0
//
//    // Process noise covariance Q
//    float32_t sigma_theta = 0.001f; // Standard deviation of the angle process noise
//    float32_t sigma_bias = 0.003f;  // Standard deviation of the bias process noise
//    kf->Q_data[0] = sigma_theta * sigma_theta; kf->Q_data[1] = 0.0f;
//    kf->Q_data[2] = 0.0f;                     kf->Q_data[3] = sigma_bias * sigma_bias;
//
//    // Measurement matrix H
//    kf->H_data[0] = 1.0f; kf->H_data[1] = 0.0f;
//    kf->H_data[2] = 1.0f; kf->H_data[3] = -1.0f;
//
//    // Measurement noise covariance R
//    float32_t sigma_acc = 0.03f;    // Standard deviation of the accelerometer noise
//    float32_t sigma_gyro = 0.02f;   // Standard deviation of the gyroscope noise
//    kf->R_data[0] = sigma_acc * sigma_acc;   kf->R_data[1] = 0.0f;
//    kf->R_data[2] = 0.0f;                   kf->R_data[3] = sigma_gyro * sigma_gyro;
//
//    // Initialize error covariance matrix P_k
//    kf->P_k_data[0] = 1.0f; kf->P_k_data[1] = 0.0f;
//    kf->P_k_data[2] = 0.0f; kf->P_k_data[3] = 1.0f;
//
//    // Initialize state vector x_k
//    kf->x_k_data[0] = 0.0f; // Initial angle estimate
//    kf->x_k_data[1] = 0.0f; // Initial bias estimate
//}
//
//// Function to compute the control input u_{i+1}
//void ComputeControlInput(float32_t theta_i, float32_t omega_i, float32_t *u_i1)
//{
//    // Compute the rotation matrix R_i for rotation around the y-axis
//    float32_t cos_theta = cosf(theta_i);
//    float32_t sin_theta = sinf(theta_i);
//
//    // Rotation matrix R_i (from body frame to inertial frame)
//    float32_t R_i_data[9] = {
//        cos_theta, 0.0f, sin_theta,
//        0.0f,      1.0f, 0.0f,
//        -sin_theta, 0.0f, cos_theta
//    };
//
//    // Compute the inverse of R_i (transpose for rotation matrices)
//    float32_t R_i_inv_data[9];
//    for (int i = 0; i < 3; i++)
//    {
//        for (int j = 0; j < 3; j++)
//        {
//            R_i_inv_data[i * 3 + j] = R_i_data[j * 3 + i];
//        }
//    }
//
//    // Gyroscope measurement vector omega_i in body frame
//    float32_t omega_i_data[3] = {0.0f, omega_i, 0.0f};
//
//    // Multiply R_i_inv with omega_i to get omega_inertial
//    float32_t omega_inertial[3];
//    for (int i = 0; i < 3; i++)
//    {
//        omega_inertial[i] = 0.0f;
//        for (int j = 0; j < 3; j++)
//        {
//            omega_inertial[i] += R_i_inv_data[i * 3 + j] * omega_i_data[j];
//        }
//    }
//
//    // Control input u_{i+1} = omega_inertial[1] (angular velocity in inertial frame)
//    *u_i1 = omega_inertial[1];
//}
//
//
//// Function to update the Kalman Filter with new measurements
//void KalmanFilter_Update(KalmanFilter *kf, float32_t theta_acc, float32_t omega_meas)
//{
//	arm_status status;
//
//	// Current estimate of theta_i
//	float32_t theta_i = kf->x_k_data[0];
//
//	// Compute control input u_{i+1}
//	float32_t u_i1;
//	ComputeControlInput(theta_i, omega_meas, &u_i1);
//
//	// Set the control input vector u_k
//	kf->u_k_data[0] = u_i1;
//
//    // Predict Step
//    // x_k = A * x_k + B * u_k
//    status = arm_mat_mult_f32(&kf->A, &kf->x_k, &kf->temp6);    // temp6 = A * x_k
//    status = arm_mat_mult_f32(&kf->B, &kf->u_k, &kf->temp1);    // temp1 = B * u_k
//    status = arm_mat_add_f32(&kf->temp6, &kf->temp1, &kf->x_k); // x_k = temp6 + temp1
//
//    // P_k = A * P_k * A^T + Q
//    status = arm_mat_mult_f32(&kf->A, &kf->P_k, &kf->temp1);    // temp1 = A * P_k
//    status = arm_mat_trans_f32(&kf->A, &kf->temp2);             // temp2 = A^T
//    status = arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->P_k); // P_k = temp1 * temp2
//    status = arm_mat_add_f32(&kf->P_k, &kf->Q, &kf->P_k);       // P_k = P_k + Q
//
//    // Update Step
//    // z_k = [theta_acc; omega_meas]
//    kf->z_k_data[0] = theta_acc;
//    kf->z_k_data[1] = omega_meas;
//
//    // y_k = z_k - H * x_k
//    status = arm_mat_mult_f32(&kf->H, &kf->x_k, &kf->temp7);    // temp7 = H * x_k
//    status = arm_mat_sub_f32(&kf->z_k, &kf->temp7, &kf->temp7); // temp7 = y_k = z_k - H * x_k
//
//    // S_k = H * P_k * H^T + R
//    status = arm_mat_mult_f32(&kf->H, &kf->P_k, &kf->temp4);    // temp4 = H * P_k
//    status = arm_mat_trans_f32(&kf->H, &kf->temp5);             // temp5 = H^T
//    status = arm_mat_mult_f32(&kf->temp4, &kf->temp5, &kf->temp3); // temp3 = temp4 * temp5
//    status = arm_mat_add_f32(&kf->temp3, &kf->R, &kf->temp3);   // temp3 = S_k = H * P_k * H^T + R
//
//    // K_k = P_k * H^T * inv(S_k)
//    status = arm_mat_inverse_f32(&kf->temp3, &kf->temp2);       // temp2 = inv(S_k)
//    status = arm_mat_mult_f32(&kf->P_k, &kf->temp5, &kf->temp4); // temp4 = P_k * H^T
//    status = arm_mat_mult_f32(&kf->temp4, &kf->temp2, &kf->K_k); // K_k = temp4 * temp2
//
//    // x_k = x_k + K_k * y_k
//    status = arm_mat_mult_f32(&kf->K_k, &kf->temp7, &kf->temp6); // temp6 = K_k * y_k
//    status = arm_mat_add_f32(&kf->x_k, &kf->temp6, &kf->x_k);    // x_k = x_k + temp6
//
//    // P_k = (I - K_k * H) * P_k
//    status = arm_mat_mult_f32(&kf->K_k, &kf->H, &kf->temp1);     // temp1 = K_k * H
//    status = arm_mat_sub_f32(&kf->I, &kf->temp1, &kf->temp2);    // temp2 = I - K_k * H
//    status = arm_mat_mult_f32(&kf->temp2, &kf->P_k, &kf->P_k);   // P_k = temp2 * P_k
//}
