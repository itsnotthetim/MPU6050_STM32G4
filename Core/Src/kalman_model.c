#include "kalman_model.h"
#include <math.h>
#include <string.h> // For memset

// Sampling time
float32_t dt;

// Function to initialize the Kalman Filter
void KalmanFilter_Init(KalmanFilter *kf, float32_t delta_t)
{
    dt = delta_t;

    // Initialize matrix instances
    arm_mat_init_f32(&kf->x_k, STATE_DIM, 1, kf->x_k_data);
    arm_mat_init_f32(&kf->P_k, STATE_DIM, STATE_DIM, kf->P_k_data);
    arm_mat_init_f32(&kf->A, STATE_DIM, STATE_DIM, kf->A_data);
    arm_mat_init_f32(&kf->B, STATE_DIM, 3, kf->B_data);
    arm_mat_init_f32(&kf->Q, STATE_DIM, STATE_DIM, kf->Q_data);
    arm_mat_init_f32(&kf->H, MEASUREMENT_DIM, STATE_DIM, kf->H_data);
    arm_mat_init_f32(&kf->R, MEASUREMENT_DIM, MEASUREMENT_DIM, kf->R_data);
    arm_mat_init_f32(&kf->K_k, STATE_DIM, MEASUREMENT_DIM, kf->K_k_data);
    arm_mat_init_f32(&kf->z_k, MEASUREMENT_DIM, 1, kf->z_k_data);
    arm_mat_init_f32(&kf->u_k, 3, 1, kf->u_k_data);
    arm_mat_init_f32(&kf->I, STATE_DIM, STATE_DIM, kf->I_data);

    // Initialize temporary matrices as needed
    arm_mat_init_f32(&kf->temp1, STATE_DIM, STATE_DIM, kf->temp1_data);
    arm_mat_init_f32(&kf->temp2, STATE_DIM, STATE_DIM, kf->temp2_data);
    arm_mat_init_f32(&kf->temp3, MEASUREMENT_DIM, MEASUREMENT_DIM, kf->temp3_data);
    arm_mat_init_f32(&kf->temp4, STATE_DIM, MEASUREMENT_DIM, kf->temp4_data);
    arm_mat_init_f32(&kf->temp5, MEASUREMENT_DIM, STATE_DIM, kf->temp5_data);
    arm_mat_init_f32(&kf->temp6, STATE_DIM, 1, kf->temp6_data);
    arm_mat_init_f32(&kf->temp7, MEASUREMENT_DIM, 1, kf->temp7_data);

    // Initialize identity matrix I
    memset(kf->I_data, 0, sizeof(kf->I_data));
    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->I_data[i * STATE_DIM + i] = 1.0f;
    }

    // State transition matrix A
    memset(kf->A_data, 0, sizeof(kf->A_data));
    for (int i = 0; i < 3; i++)
    {
        kf->A_data[i * STATE_DIM + i] = 1.0f;                 // Diagonal elements for angles
        kf->A_data[i * STATE_DIM + i + 3] = -dt;              // Coupling with biases
        kf->A_data[(i + 3) * STATE_DIM + i + 3] = 1.0f;       // Diagonal elements for biases
    }

    // Control input matrix B
    memset(kf->B_data, 0, sizeof(kf->B_data));
    for (int i = 0; i < 3; i++)
    {
        kf->B_data[i * 3 + i] = dt; // B[i][i] = dt
    }

    // Process noise covariance Q
    float32_t sigma_phi = 0.001f;
    float32_t sigma_theta = 0.001f;
    float32_t sigma_psi = 0.001f;
    float32_t sigma_bias_phi = 0.003f;
    float32_t sigma_bias_theta = 0.003f;
    float32_t sigma_bias_psi = 0.003f;
    memset(kf->Q_data, 0, sizeof(kf->Q_data));
    kf->Q_data[0 * STATE_DIM + 0] = sigma_phi * sigma_phi * dt * dt;
    kf->Q_data[1 * STATE_DIM + 1] = sigma_theta * sigma_theta * dt * dt;
    kf->Q_data[2 * STATE_DIM + 2] = sigma_psi * sigma_psi * dt * dt;
    kf->Q_data[3 * STATE_DIM + 3] = sigma_bias_phi * sigma_bias_phi * dt ;
    kf->Q_data[4 * STATE_DIM + 4] = sigma_bias_theta * sigma_bias_theta * dt;
    kf->Q_data[5 * STATE_DIM + 5] = sigma_bias_psi * sigma_bias_psi * dt ;

    // Measurement matrix H
    memset(kf->H_data, 0, sizeof(kf->H_data));
    // Accelerometer measurements for roll and pitch
    kf->H_data[0 * STATE_DIM + 0] = 1.0f; // phi_acc depends on phi
    kf->H_data[1 * STATE_DIM + 1] = 1.0f; // theta_acc depends on theta
    // Magnetometer measurement for yaw
    kf->H_data[2 * STATE_DIM + 2] = 1.0f; // psi_mag depends on psi
    // Gyroscope measurements
    kf->H_data[3 * STATE_DIM + 0] = 1.0f;   kf->H_data[3 * STATE_DIM + 3] = -1.0f; // omega_phi
    kf->H_data[4 * STATE_DIM + 1] = 1.0f;   kf->H_data[4 * STATE_DIM + 4] = -1.0f; // omega_theta
    kf->H_data[5 * STATE_DIM + 2] = 1.0f;   kf->H_data[5 * STATE_DIM + 5] = -1.0f; // omega_psi

    // Measurement noise covariance R
    float32_t sigma_acc_phi = 0.03f;
    float32_t sigma_acc_theta = 0.03f;
    float32_t sigma_mag_psi = 1000.0f; // Large value due to lack of magnetometer
    float32_t sigma_gyro_phi = 0.02f;
    float32_t sigma_gyro_theta = 0.02f;
    float32_t sigma_gyro_psi = 0.02f;
    memset(kf->R_data, 0, sizeof(kf->R_data));
    kf->R_data[0 * MEASUREMENT_DIM + 0] = sigma_acc_phi * sigma_acc_phi ;
    kf->R_data[1 * MEASUREMENT_DIM + 1] = sigma_acc_theta * sigma_acc_theta ;
    kf->R_data[2 * MEASUREMENT_DIM + 2] = sigma_mag_psi * sigma_mag_psi ; // High uncertainty for psi_mag
    kf->R_data[3 * MEASUREMENT_DIM + 3] = sigma_gyro_phi * sigma_gyro_phi ;
    kf->R_data[4 * MEASUREMENT_DIM + 4] = sigma_gyro_theta * sigma_gyro_theta ;
    kf->R_data[5 * MEASUREMENT_DIM + 5] = sigma_gyro_psi * sigma_gyro_psi ;

    // Initialize error covariance matrix P_k
    memset(kf->P_k_data, 0, sizeof(kf->P_k_data));
    for (int i = 0; i < STATE_DIM; i++)
    {
        kf->P_k_data[i * STATE_DIM + i] = 1.0f;
    }

    // Initialize state vector x_k
    memset(kf->x_k_data, 0, sizeof(kf->x_k_data));
}

// Function to compute the control input u_{i+1}
void ComputeControlInput(float32_t phi_i, float32_t theta_i, float32_t psi_i, float32_t omega_phi_i, float32_t omega_theta_i, float32_t omega_psi_i, float32_t *u_i1)
{
    // Compute the rotation matrix R_i^{-1} (from inertial frame to body frame)
    float32_t cos_phi = cosf(phi_i);
    float32_t sin_phi = sinf(phi_i);
    float32_t cos_theta = cosf(theta_i);
    float32_t sin_theta = sinf(theta_i);
    float32_t cos_psi = cosf(psi_i);
    float32_t sin_psi = sinf(psi_i);

    // Compute the elements of the rotation matrix R_i^{-1}
    float32_t R_inv[3][3];
    R_inv[0][0] = cos_theta * cos_psi;
    R_inv[0][1] = sin_phi * sin_theta * cos_psi - cos_phi * sin_psi;
    R_inv[0][2] = cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
    R_inv[1][0] = cos_theta * sin_psi;
    R_inv[1][1] = sin_phi * sin_theta * sin_psi + cos_phi * cos_psi;
    R_inv[1][2] = cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
    R_inv[2][0] = -sin_theta;
    R_inv[2][1] = sin_phi * cos_theta;
    R_inv[2][2] = cos_phi * cos_theta;

    // Gyroscope measurement vector omega_i in body frame
    float32_t omega_i[3] = {omega_phi_i, omega_theta_i, omega_psi_i};

    // Compute omega_inertial = R_i^{-1} * omega_i
    float32_t omega_inertial[3];
    for (int i = 0; i < 3; i++)
    {
        omega_inertial[i] = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            omega_inertial[i] += R_inv[i][j] * omega_i[j];
        }
    }

    // Control input u_{i+1} = omega_inertial
    u_i1[0] = omega_inertial[0]; // Roll rate in inertial frame
    u_i1[1] = omega_inertial[1]; // Pitch rate in inertial frame
    u_i1[2] = omega_inertial[2]; // Yaw rate in inertial frame
}

// Function to update the Kalman Filter with new measurements
void KalmanFilter_Update(KalmanFilter *kf, float32_t phi_acc, float32_t theta_acc, float32_t psi_mag, float32_t omega_phi, float32_t omega_theta, float32_t omega_psi)
{
    arm_status status;

    // Current estimates
    float32_t phi_i = kf->x_k_data[0];
    float32_t theta_i = kf->x_k_data[1];
    float32_t psi_i = kf->x_k_data[2];

    // Compute control input u_{i+1}
    float32_t u_i1[3];
    ComputeControlInput(phi_i, theta_i, psi_i, omega_phi, omega_theta, omega_psi, u_i1);

    // Set the control input vector u_k
    kf->u_k_data[0] = u_i1[0];
    kf->u_k_data[1] = u_i1[1];
    kf->u_k_data[2] = u_i1[2];

    // Predict Step
    // x_k = A * x_k + B * u_k
    status = arm_mat_mult_f32(&kf->A, &kf->x_k, &kf->temp6); // temp6 = A * x_k
    status = arm_mat_mult_f32(&kf->B, &kf->u_k, &kf->temp1); // temp1 = B * u_k
    status = arm_mat_add_f32(&kf->temp6, &kf->temp1, &kf->x_k); // x_k = temp6 + temp1

    // P_k = A * P_k * A^T + Q
    status = arm_mat_mult_f32(&kf->A, &kf->P_k, &kf->temp1); // temp1 = A * P_k
    status = arm_mat_trans_f32(&kf->A, &kf->temp2); // temp2 = A^T
    status = arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->P_k); // P_k = temp1 * temp2
    status = arm_mat_add_f32(&kf->P_k, &kf->Q, &kf->P_k); // P_k = P_k + Q

    // Update State
    // z_k = [phi_acc; theta_acc; psi_mag; omega_phi; omega_theta; omega_psi]
    kf->z_k_data[0] = phi_acc;
    kf->z_k_data[1] = theta_acc;
    kf->z_k_data[2] = psi_mag; // This value is not available; set as needed
    kf->z_k_data[3] = omega_phi;
    kf->z_k_data[4] = omega_theta;
    kf->z_k_data[5] = omega_psi;

    // y_k = z_k - H * x_k
    status = arm_mat_mult_f32(&kf->H, &kf->x_k, &kf->temp7); // temp7 = H * x_k
    status = arm_mat_sub_f32(&kf->z_k, &kf->temp7, &kf->temp7); // temp7 = y_k = z_k - H * x_k

    // S_k = H * P_k * H^T + R
    status = arm_mat_mult_f32(&kf->H, &kf->P_k, &kf->temp4); // temp4 = H * P_k
    status = arm_mat_trans_f32(&kf->H, &kf->temp5); // temp5 = H^T
    status = arm_mat_mult_f32(&kf->temp4, &kf->temp5, &kf->temp3); // temp3 = temp4 * temp5
    status = arm_mat_add_f32(&kf->temp3, &kf->R, &kf->temp3); // temp3 = S_k = H * P_k * H^T + R

    // K_k = P_k * H^T * inv(S_k)
    status = arm_mat_inverse_f32(&kf->temp3, &kf->temp2); // temp2 = inv(S_k)
    status = arm_mat_mult_f32(&kf->P_k, &kf->temp5, &kf->temp4); // temp4 = P_k * H^T
    status = arm_mat_mult_f32(&kf->temp4, &kf->temp2, &kf->K_k); // K_k = temp4 * temp2

    // x_k = x_k + K_k * y_k
    status = arm_mat_mult_f32(&kf->K_k, &kf->temp7, &kf->temp6); // temp6 = K_k * y_k
    status = arm_mat_add_f32(&kf->x_k, &kf->temp6, &kf->x_k); // x_k = x_k + temp6

    // P_k = (I - K_k * H) * P_k
    status = arm_mat_mult_f32(&kf->K_k, &kf->H, &kf->temp1); // temp1 = K_k * H
    status = arm_mat_sub_f32(&kf->I, &kf->temp1, &kf->temp2); // temp2 = I - K_k * H
    status = arm_mat_mult_f32(&kf->temp2, &kf->P_k, &kf->P_k); // P_k = temp2 * P_k
}
