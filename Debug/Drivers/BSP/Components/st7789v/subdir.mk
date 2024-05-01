################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/st7789v/st7789v.c \
../Drivers/BSP/Components/st7789v/st7789v_reg.c 

OBJS += \
./Drivers/BSP/Components/st7789v/st7789v.o \
./Drivers/BSP/Components/st7789v/st7789v_reg.o 

C_DEPS += \
./Drivers/BSP/Components/st7789v/st7789v.d \
./Drivers/BSP/Components/st7789v/st7789v_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/st7789v/%.o Drivers/BSP/Components/st7789v/%.su Drivers/BSP/Components/st7789v/%.cyclo: ../Drivers/BSP/Components/st7789v/%.c Drivers/BSP/Components/st7789v/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../DISPLAY/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/st7789v -I../Drivers/BSP/Components/Common -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-st7789v

clean-Drivers-2f-BSP-2f-Components-2f-st7789v:
	-$(RM) ./Drivers/BSP/Components/st7789v/st7789v.cyclo ./Drivers/BSP/Components/st7789v/st7789v.d ./Drivers/BSP/Components/st7789v/st7789v.o ./Drivers/BSP/Components/st7789v/st7789v.su ./Drivers/BSP/Components/st7789v/st7789v_reg.cyclo ./Drivers/BSP/Components/st7789v/st7789v_reg.d ./Drivers/BSP/Components/st7789v/st7789v_reg.o ./Drivers/BSP/Components/st7789v/st7789v_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-st7789v

