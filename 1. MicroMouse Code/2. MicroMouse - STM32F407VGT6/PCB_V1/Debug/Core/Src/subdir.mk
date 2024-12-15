################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AML_DebugDevice.c \
../Core/Src/AML_Encoder.c \
../Core/Src/AML_Keyboard.c \
../Core/Src/AML_LaserSensor.c \
../Core/Src/AML_MPUSensor.c \
../Core/Src/AML_MotorControl.c \
../Core/Src/AML_Remote.c \
../Core/Src/KalmanFilter.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/vl53l0x_api.c \
../Core/Src/vl53l0x_api_calibration.c \
../Core/Src/vl53l0x_api_core.c \
../Core/Src/vl53l0x_api_ranging.c \
../Core/Src/vl53l0x_api_strings.c \
../Core/Src/vl53l0x_platform.c \
../Core/Src/vl53l0x_platform_log.c 

OBJS += \
./Core/Src/AML_DebugDevice.o \
./Core/Src/AML_Encoder.o \
./Core/Src/AML_Keyboard.o \
./Core/Src/AML_LaserSensor.o \
./Core/Src/AML_MPUSensor.o \
./Core/Src/AML_MotorControl.o \
./Core/Src/AML_Remote.o \
./Core/Src/KalmanFilter.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/vl53l0x_api.o \
./Core/Src/vl53l0x_api_calibration.o \
./Core/Src/vl53l0x_api_core.o \
./Core/Src/vl53l0x_api_ranging.o \
./Core/Src/vl53l0x_api_strings.o \
./Core/Src/vl53l0x_platform.o \
./Core/Src/vl53l0x_platform_log.o 

C_DEPS += \
./Core/Src/AML_DebugDevice.d \
./Core/Src/AML_Encoder.d \
./Core/Src/AML_Keyboard.d \
./Core/Src/AML_LaserSensor.d \
./Core/Src/AML_MPUSensor.d \
./Core/Src/AML_MotorControl.d \
./Core/Src/AML_Remote.d \
./Core/Src/KalmanFilter.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/vl53l0x_api.d \
./Core/Src/vl53l0x_api_calibration.d \
./Core/Src/vl53l0x_api_core.d \
./Core/Src/vl53l0x_api_ranging.d \
./Core/Src/vl53l0x_api_strings.d \
./Core/Src/vl53l0x_platform.d \
./Core/Src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AML_DebugDevice.cyclo ./Core/Src/AML_DebugDevice.d ./Core/Src/AML_DebugDevice.o ./Core/Src/AML_DebugDevice.su ./Core/Src/AML_Encoder.cyclo ./Core/Src/AML_Encoder.d ./Core/Src/AML_Encoder.o ./Core/Src/AML_Encoder.su ./Core/Src/AML_Keyboard.cyclo ./Core/Src/AML_Keyboard.d ./Core/Src/AML_Keyboard.o ./Core/Src/AML_Keyboard.su ./Core/Src/AML_LaserSensor.cyclo ./Core/Src/AML_LaserSensor.d ./Core/Src/AML_LaserSensor.o ./Core/Src/AML_LaserSensor.su ./Core/Src/AML_MPUSensor.cyclo ./Core/Src/AML_MPUSensor.d ./Core/Src/AML_MPUSensor.o ./Core/Src/AML_MPUSensor.su ./Core/Src/AML_MotorControl.cyclo ./Core/Src/AML_MotorControl.d ./Core/Src/AML_MotorControl.o ./Core/Src/AML_MotorControl.su ./Core/Src/AML_Remote.cyclo ./Core/Src/AML_Remote.d ./Core/Src/AML_Remote.o ./Core/Src/AML_Remote.su ./Core/Src/KalmanFilter.cyclo ./Core/Src/KalmanFilter.d ./Core/Src/KalmanFilter.o ./Core/Src/KalmanFilter.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/vl53l0x_api.cyclo ./Core/Src/vl53l0x_api.d ./Core/Src/vl53l0x_api.o ./Core/Src/vl53l0x_api.su ./Core/Src/vl53l0x_api_calibration.cyclo ./Core/Src/vl53l0x_api_calibration.d ./Core/Src/vl53l0x_api_calibration.o ./Core/Src/vl53l0x_api_calibration.su ./Core/Src/vl53l0x_api_core.cyclo ./Core/Src/vl53l0x_api_core.d ./Core/Src/vl53l0x_api_core.o ./Core/Src/vl53l0x_api_core.su ./Core/Src/vl53l0x_api_ranging.cyclo ./Core/Src/vl53l0x_api_ranging.d ./Core/Src/vl53l0x_api_ranging.o ./Core/Src/vl53l0x_api_ranging.su ./Core/Src/vl53l0x_api_strings.cyclo ./Core/Src/vl53l0x_api_strings.d ./Core/Src/vl53l0x_api_strings.o ./Core/Src/vl53l0x_api_strings.su ./Core/Src/vl53l0x_platform.cyclo ./Core/Src/vl53l0x_platform.d ./Core/Src/vl53l0x_platform.o ./Core/Src/vl53l0x_platform.su ./Core/Src/vl53l0x_platform_log.cyclo ./Core/Src/vl53l0x_platform_log.d ./Core/Src/vl53l0x_platform_log.o ./Core/Src/vl53l0x_platform_log.su

.PHONY: clean-Core-2f-Src

