################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ov2640/ov2640.c 

OBJS += \
./Drivers/BSP/Components/ov2640/ov2640.o 

C_DEPS += \
./Drivers/BSP/Components/ov2640/ov2640.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ov2640/%.o Drivers/BSP/Components/ov2640/%.su Drivers/BSP/Components/ov2640/%.cyclo: ../Drivers/BSP/Components/ov2640/%.c Drivers/BSP/Components/ov2640/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/BSP/STM32F413H-Discovery -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ov2640

clean-Drivers-2f-BSP-2f-Components-2f-ov2640:
	-$(RM) ./Drivers/BSP/Components/ov2640/ov2640.cyclo ./Drivers/BSP/Components/ov2640/ov2640.d ./Drivers/BSP/Components/ov2640/ov2640.o ./Drivers/BSP/Components/ov2640/ov2640.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ov2640

