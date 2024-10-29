/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <mpu6050.h>
#include <imu_interfaces/srv/imu_calibration.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>
#include <rosidl_runtime_c/string_functions.h>
#include "math.h"
//#include "arm_math.h"
//#include "kalman_angle.h"
#include "kalman_model.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	double x;
	double y;
	double z;
}offset3d_t;

bool is_calib = false;

typedef struct{
	double ax;
	double ay;
	double az;
	double gx;
	double gy;
	double gz;
}tuple_double_t;

typedef struct{
	double roll;
	double pitch;
	double yaw;
}angle_t;

bool is_active = false;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) if(fn != RCL_RET_OK) {};

#define G2M_S2 9.81
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rcl_timer_t mpu6050_timer;
rclc_executor_t executor;
rcl_service_t mpu6050_service;
rcl_service_t status_service;

rcl_publisher_t mpu6050_publisher;
sensor_msgs__msg__Imu mpu6050_msg;
imu_interfaces__srv__ImuCalibration_Response mpu6050_response;
imu_interfaces__srv__ImuCalibration_Request mpu6050_request;
std_srvs__srv__Trigger_Request status_request;
std_srvs__srv__Trigger_Response status_response;

MPU6050_t MPU6050;
offset3d_t accel_offset;
offset3d_t gyro_offset;

tuple_double_t data;
tuple_double_t read_data;
float gyro_roll = 0,gyro_pitch = 0, gyro_yaw = 0;
float accel_roll,accel_pitch;

angle_t gyro_angle, accel_angle;
rmw_ret_t mrag_status;
rcl_publisher_t cmd_vel;
geometry_msgs__msg__Twist cmdvel_msg;

Kalman_t KX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};


KalmanFilter kf;
HAL_StatusTypeDef device_status;
double v_x,v_y;
uint8_t reset_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

tuple_double_t readImuCalibrated();
tuple_double_t readImuNonCalibrated();
void rpfromAccel();
void rpfromGyro();
double mapd(double input, double in_min, double in_max, double out_min, double out_max) ;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  while (MPU6050_Init(&hi2c1) == 1);
//  KalmanFilter_Init();
//  KalmanFilter_Init(&kf_roll, 0.01f);
//  KalmanFilter_Init(&kf_pitch, 0.01f);
  KalmanFilter_Init(&kf, 0.01);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {

		// Synchronize time with the agent
		rmw_uros_sync_session(1000);
		device_status = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 10);


		if(device_status == HAL_OK){
			is_active = true;
			if(is_calib ) {

				double linear_x, angular_z;
				readImuCalibrated();

				rpfromAccel();
				rpfromGyro();
				RCCHECK(rcl_publish(&mpu6050_publisher, &mpu6050_msg, NULL));

//				v_x = Kalman_getAngle(&KX, accel_angle.roll, data.gx, 0.01);
//				v_y = Kalman_getAngle(&KY, accel_angle.pitch, data.gy, 0.01);

				KalmanFilter_Update(&kf, accel_angle.roll, accel_angle.pitch, 0, data.gx, data.gy, data.gz);

				linear_x = kf.x_k_data[0];
				angular_z = - kf.x_k_data[1];

				if((kf.x_k_data[0] >= -0.1) && (kf.x_k_data[0] <= 0.1)){
					linear_x = 0.0;
				}
				if((kf.x_k_data[1] >= -0.1) && (kf.x_k_data[1] <= 0.1)){
					angular_z = 0.0;
				}



				cmdvel_msg.linear.x =  mapd(linear_x,-60.0,60.0,-5.0,5.0);
				cmdvel_msg.angular.z = mapd(angular_z,-60.0,60.0,-2.0,2.0);

				cmdvel_msg.angular.x =  v_x;
				cmdvel_msg.angular.y = v_y;

				RCCHECK(rcl_publish(&cmd_vel, &cmdvel_msg, NULL));

			}
			else{
				readImuNonCalibrated();
				RCCHECK(rcl_publish(&mpu6050_publisher, &mpu6050_msg, NULL));
			}
		}
		else{
			is_active = false;
			if(reset_flag == 1){
				HAL_NVIC_SystemReset();
			}
		}

		HAL_IWDG_Refresh(&hiwdg);
	}

}


void service_callback(const void * request_msg, void * response_msg){
	// Cast messages to expected types
	imu_interfaces__srv__ImuCalibration_Request * req_in = (imu_interfaces__srv__ImuCalibration_Request * ) request_msg;

	imu_interfaces__srv__ImuCalibration_Response * res_in = (imu_interfaces__srv__ImuCalibration_Response * ) response_msg;

	for(int i = 0; i < 9; i++){
		mpu6050_msg.linear_acceleration_covariance[i] = req_in->imu_calib.linear_acceleration_covariance[i];
		mpu6050_msg.angular_velocity_covariance[i] = req_in->imu_calib.angular_velocity_covariance[i];
	}


	accel_offset.x = req_in->imu_calib.linear_acceleration.x;
	accel_offset.y = req_in->imu_calib.linear_acceleration.y;
	accel_offset.z = req_in->imu_calib.linear_acceleration.z;

	gyro_offset.x = req_in->imu_calib.angular_velocity.x;
	gyro_offset.y = req_in->imu_calib.angular_velocity.y;
	gyro_offset.z = req_in->imu_calib.angular_velocity.z;

	is_calib = true;

	res_in->success = true;

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


}

void status_callback(const void * request_msg, void * response_msg){
	// Cast messages to expected types

	std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response  * ) response_msg;

	if(is_active){
		res_in->success = true;
		rosidl_runtime_c__String__assign(&res_in->message, "Connection Successful");
	}
	else{
		res_in->success = false;
		rosidl_runtime_c__String__assign(&res_in->message, "Connection Failed. Resetting . . .");

		reset_flag = 1;


	}

}

tuple_double_t readImuCalibrated(){
	MPU6050_Read_All(&hi2c1, &MPU6050);
	mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
	mpu6050_msg.header.stamp.nanosec = 	rmw_uros_epoch_nanos();
	mpu6050_msg.linear_acceleration.x = G2M_S2 * MPU6050.Ax - accel_offset.x;
	mpu6050_msg.linear_acceleration.y = G2M_S2 * MPU6050.Ay - accel_offset.y;
	mpu6050_msg.linear_acceleration.z = G2M_S2 * MPU6050.Az - accel_offset.z;

	mpu6050_msg.angular_velocity.x = DEG2RAD * MPU6050.Gx - gyro_offset.x;
	mpu6050_msg.angular_velocity.y = DEG2RAD * MPU6050.Gy - gyro_offset.y;
	mpu6050_msg.angular_velocity.z = DEG2RAD * MPU6050.Gz - gyro_offset.z;

	data.ax = mpu6050_msg.linear_acceleration.x;
	data.ay = mpu6050_msg.linear_acceleration.y;
	data.az = mpu6050_msg.linear_acceleration.z;

	data.gx = mpu6050_msg.angular_velocity.x;
	data.gy = mpu6050_msg.angular_velocity.y;
	data.gz = mpu6050_msg.angular_velocity.z;

	return data;
}

tuple_double_t readImuNonCalibrated(){
	MPU6050_Read_All(&hi2c1, &MPU6050);
	mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
	mpu6050_msg.header.stamp.nanosec = 	rmw_uros_epoch_nanos();
	mpu6050_msg.linear_acceleration.x = G2M_S2 * MPU6050.Ax ;
	mpu6050_msg.linear_acceleration.y = G2M_S2 * MPU6050.Ay;
	mpu6050_msg.linear_acceleration.z = G2M_S2 * MPU6050.Az;

	mpu6050_msg.angular_velocity.x = DEG2RAD * MPU6050.Gx;
	mpu6050_msg.angular_velocity.y = DEG2RAD * MPU6050.Gy ;
	mpu6050_msg.angular_velocity.z = DEG2RAD * MPU6050.Gz ;

	data.ax = mpu6050_msg.linear_acceleration.x;
	data.ay = mpu6050_msg.linear_acceleration.y;
	data.az = mpu6050_msg.linear_acceleration.z;

	data.gx = mpu6050_msg.angular_velocity.x;
	data.gy = mpu6050_msg.angular_velocity.y;
	data.gz = mpu6050_msg.angular_velocity.z;

	return data;
}

void rpfromAccel(){
//	accel_angle.roll =  atan2(data.ay, data.az) * RAD2DEG ;
	accel_angle.roll =  atan2(data.ay, sqrt(data.ax * data.ax + data.az * data.az )) * RAD2DEG ;
	accel_angle.pitch = -atan2(data.ax, sqrt(data.ay * data.ay + data.az * data.az )) * RAD2DEG;

}

void rpfromGyro(){
	gyro_angle.roll =  gyro_roll + data.gx * 0.01 * RAD2DEG ;
	gyro_angle.pitch = gyro_pitch + data.gy * 0.01 * RAD2DEG ;
	gyro_angle.yaw = gyro_yaw + data.gz * 0.01 * RAD2DEG ;

}

double mapd(double input, double in_min, double in_max, double out_min, double out_max) {
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &hlpuart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  allocator = rcl_get_default_allocator();

  //create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 88));

//  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);


  GPIO_PinState button = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
  uint8_t num_executor = 2  ; // total number of handles =  #timer + service

  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);



  // create timer
  rclc_timer_init_default(&mpu6050_timer, &support, RCL_MS_TO_NS(10), timer_callback);

  // create publisher

  rclc_publisher_init_best_effort(
    &mpu6050_publisher, &node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "mpu6050_publisher");

  rclc_publisher_init_default(
      &cmd_vel, &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // create service
  rclc_service_init_default(&status_service, &node,
  			ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger)
  			, "status");

	if(button == GPIO_PIN_SET){
		rclc_service_init_default(&mpu6050_service, &node,
			ROSIDL_GET_SRV_TYPE_SUPPORT(imu_interfaces, srv, ImuCalibration)
			, "mpu6050_calibration");
		num_executor += 1;
	}

	else{
		is_calib = true;
		accel_offset.x = 0.6312768530273459;
		accel_offset.y = -0.0819649929199216;
		accel_offset.z = 0.5116870961298351;

		gyro_offset.x = -0.023353091609142815;
		gyro_offset.y = 0.023326005697926703;
		gyro_offset.z = 0.005351206132862714;

		double gyro_cov[9] =  {
				2.155817054758849e-06, 1.1030785686223727e-07, 1.229854119042769e-08,
				1.1030785686223727e-07, 2.5384651252082968e-06, 1.7032517411039515e-08,
				1.229854119042769e-08, 1.7032517411039515e-08, 1.906569917368417e-06
			};
		double acc_cov[9] = {
		    0.0013081652288234637,  1.0781109688085633e-05,  2.3493748179790447e-05,
		    1.0781109688085633e-05, 0.0011941322210539112,  4.108841533445584e-05,
		    2.3493748179790447e-05, 4.108841533445584e-05,  0.004555163865375252
		};

		for(int i = 0; i < 9; i++){
				mpu6050_msg.linear_acceleration_covariance[i] = acc_cov[i];
				mpu6050_msg.angular_velocity_covariance[i] = gyro_cov[i];
			}
    }

	// Synchronize time with the agent (t check the connectivity of micro_ros_agent)
	  rmw_uros_sync_session(1000);


  // create message
  mpu6050_msg.header.frame_id = micro_ros_string_utilities_init("imu_frame");

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();


  rclc_executor_init(&executor, &support.context, num_executor, &allocator);
  rclc_executor_add_timer(&executor, &mpu6050_timer); // add timer
  if(button == GPIO_PIN_SET) rclc_executor_add_service(&executor, &mpu6050_service, &mpu6050_request,&mpu6050_response, service_callback);
  rclc_executor_add_service(&executor, &status_service, &status_request,&status_response, status_callback);
  rclc_executor_spin(&executor);


  for(;;){
    osDelay(10);
  }
  /* USER CODE END 5 */
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
