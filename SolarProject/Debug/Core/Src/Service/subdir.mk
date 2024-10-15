################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Service/motorService.c \
../Core/Src/Service/sensorService.c \
../Core/Src/Service/uartService.c 

OBJS += \
./Core/Src/Service/motorService.o \
./Core/Src/Service/sensorService.o \
./Core/Src/Service/uartService.o 

C_DEPS += \
./Core/Src/Service/motorService.d \
./Core/Src/Service/sensorService.d \
./Core/Src/Service/uartService.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Service/%.o Core/Src/Service/%.su Core/Src/Service/%.cyclo: ../Core/Src/Service/%.c Core/Src/Service/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Service

clean-Core-2f-Src-2f-Service:
	-$(RM) ./Core/Src/Service/motorService.cyclo ./Core/Src/Service/motorService.d ./Core/Src/Service/motorService.o ./Core/Src/Service/motorService.su ./Core/Src/Service/sensorService.cyclo ./Core/Src/Service/sensorService.d ./Core/Src/Service/sensorService.o ./Core/Src/Service/sensorService.su ./Core/Src/Service/uartService.cyclo ./Core/Src/Service/uartService.d ./Core/Src/Service/uartService.o ./Core/Src/Service/uartService.su

.PHONY: clean-Core-2f-Src-2f-Service

