################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OSModule/kernel.c 

C_DEPS += \
./OSModule/kernel.d 

OBJS += \
./OSModule/kernel.o 


# Each subdirectory must supply rules for building sources it contributes
OSModule/%.o OSModule/%.su OSModule/%.cyclo: ../OSModule/%.c OSModule/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Development/RtosSensorDataLogger/STM32Cube/OSModule/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-OSModule

clean-OSModule:
	-$(RM) ./OSModule/kernel.cyclo ./OSModule/kernel.d ./OSModule/kernel.o ./OSModule/kernel.su

.PHONY: clean-OSModule

