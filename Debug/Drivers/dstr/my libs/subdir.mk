################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/dstr/my\ libs/simple_delay.c 

OBJS += \
./Drivers/dstr/my\ libs/simple_delay.o 

C_DEPS += \
./Drivers/dstr/my\ libs/simple_delay.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/dstr/my\ libs/simple_delay.o: ../Drivers/dstr/my\ libs/simple_delay.c Drivers/dstr/my\ libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../LIBJPEG/App -I../LIBJPEG/Target -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/LibJPEG/include -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/Inc" -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/Src" -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/my libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/dstr/my libs/simple_delay.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-dstr-2f-my-20-libs

clean-Drivers-2f-dstr-2f-my-20-libs:
	-$(RM) ./Drivers/dstr/my\ libs/simple_delay.cyclo ./Drivers/dstr/my\ libs/simple_delay.d ./Drivers/dstr/my\ libs/simple_delay.o ./Drivers/dstr/my\ libs/simple_delay.su

.PHONY: clean-Drivers-2f-dstr-2f-my-20-libs

