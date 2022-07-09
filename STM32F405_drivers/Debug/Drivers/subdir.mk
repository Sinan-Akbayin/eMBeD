################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F4xx_GPIO.c 

OBJS += \
./Drivers/STM32F4xx_GPIO.o 

C_DEPS += \
./Drivers/STM32F4xx_GPIO.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/efakb/STM32CubeIDE/workspace_stm32f4/001HelloWorld/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/STM32F4xx_GPIO.d ./Drivers/STM32F4xx_GPIO.o ./Drivers/STM32F4xx_GPIO.su

.PHONY: clean-Drivers

