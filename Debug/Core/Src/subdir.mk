################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/API_Comm.c \
../Core/Src/BMI088.c \
../Core/Src/ComputeOrientation.c \
../Core/Src/EKF.c \
../Core/Src/FusionAhrs.c \
../Core/Src/FusionCompass.c \
../Core/Src/FusionOffset.c \
../Core/Src/LPF.c \
../Core/Src/MadgwickAHRS.c \
../Core/Src/Serial_Comm.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/API_Comm.o \
./Core/Src/BMI088.o \
./Core/Src/ComputeOrientation.o \
./Core/Src/EKF.o \
./Core/Src/FusionAhrs.o \
./Core/Src/FusionCompass.o \
./Core/Src/FusionOffset.o \
./Core/Src/LPF.o \
./Core/Src/MadgwickAHRS.o \
./Core/Src/Serial_Comm.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/API_Comm.d \
./Core/Src/BMI088.d \
./Core/Src/ComputeOrientation.d \
./Core/Src/EKF.d \
./Core/Src/FusionAhrs.d \
./Core/Src/FusionCompass.d \
./Core/Src/FusionOffset.d \
./Core/Src/LPF.d \
./Core/Src/MadgwickAHRS.d \
./Core/Src/Serial_Comm.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_API -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/API_Comm.cyclo ./Core/Src/API_Comm.d ./Core/Src/API_Comm.o ./Core/Src/API_Comm.su ./Core/Src/BMI088.cyclo ./Core/Src/BMI088.d ./Core/Src/BMI088.o ./Core/Src/BMI088.su ./Core/Src/ComputeOrientation.cyclo ./Core/Src/ComputeOrientation.d ./Core/Src/ComputeOrientation.o ./Core/Src/ComputeOrientation.su ./Core/Src/EKF.cyclo ./Core/Src/EKF.d ./Core/Src/EKF.o ./Core/Src/EKF.su ./Core/Src/FusionAhrs.cyclo ./Core/Src/FusionAhrs.d ./Core/Src/FusionAhrs.o ./Core/Src/FusionAhrs.su ./Core/Src/FusionCompass.cyclo ./Core/Src/FusionCompass.d ./Core/Src/FusionCompass.o ./Core/Src/FusionCompass.su ./Core/Src/FusionOffset.cyclo ./Core/Src/FusionOffset.d ./Core/Src/FusionOffset.o ./Core/Src/FusionOffset.su ./Core/Src/LPF.cyclo ./Core/Src/LPF.d ./Core/Src/LPF.o ./Core/Src/LPF.su ./Core/Src/MadgwickAHRS.cyclo ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/Serial_Comm.cyclo ./Core/Src/Serial_Comm.d ./Core/Src/Serial_Comm.o ./Core/Src/Serial_Comm.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

