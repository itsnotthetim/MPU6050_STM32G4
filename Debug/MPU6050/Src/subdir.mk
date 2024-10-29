################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU6050/Src/mpu6050.c 

OBJS += \
./MPU6050/Src/mpu6050.o 

C_DEPS += \
./MPU6050/Src/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
MPU6050/Src/%.o MPU6050/Src/%.su MPU6050/Src/%.cyclo: ../MPU6050/Src/%.c MPU6050/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I"/home/tim/Documents/GitHub/MPU6050_STM32G4/Drivers/CMSIS/DSP/Include" -I../MPU6050/Src -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MPU6050-2f-Src

clean-MPU6050-2f-Src:
	-$(RM) ./MPU6050/Src/mpu6050.cyclo ./MPU6050/Src/mpu6050.d ./MPU6050/Src/mpu6050.o ./MPU6050/Src/mpu6050.su

.PHONY: clean-MPU6050-2f-Src

