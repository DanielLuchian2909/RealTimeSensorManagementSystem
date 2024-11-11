################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/OperatingSystem/kernel.c \
../Modules/OperatingSystem/queue.c \
../Modules/OperatingSystem/scheduler.c \
../Modules/OperatingSystem/task.c 

C_DEPS += \
./Modules/OperatingSystem/kernel.d \
./Modules/OperatingSystem/queue.d \
./Modules/OperatingSystem/scheduler.d \
./Modules/OperatingSystem/task.d 

OBJS += \
./Modules/OperatingSystem/kernel.o \
./Modules/OperatingSystem/queue.o \
./Modules/OperatingSystem/scheduler.o \
./Modules/OperatingSystem/task.o 


# Each subdirectory must supply rules for building sources it contributes
Modules/OperatingSystem/%.o Modules/OperatingSystem/%.su Modules/OperatingSystem/%.cyclo: ../Modules/OperatingSystem/%.c Modules/OperatingSystem/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-OperatingSystem

clean-Modules-2f-OperatingSystem:
	-$(RM) ./Modules/OperatingSystem/kernel.cyclo ./Modules/OperatingSystem/kernel.d ./Modules/OperatingSystem/kernel.o ./Modules/OperatingSystem/kernel.su ./Modules/OperatingSystem/queue.cyclo ./Modules/OperatingSystem/queue.d ./Modules/OperatingSystem/queue.o ./Modules/OperatingSystem/queue.su ./Modules/OperatingSystem/scheduler.cyclo ./Modules/OperatingSystem/scheduler.d ./Modules/OperatingSystem/scheduler.o ./Modules/OperatingSystem/scheduler.su ./Modules/OperatingSystem/task.cyclo ./Modules/OperatingSystem/task.d ./Modules/OperatingSystem/task.o ./Modules/OperatingSystem/task.su

.PHONY: clean-Modules-2f-OperatingSystem

