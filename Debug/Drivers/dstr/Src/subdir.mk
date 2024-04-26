################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/dstr/Src/stm32f407xx_gpio.c \
../Drivers/dstr/Src/stm32f407xx_i2c.c \
../Drivers/dstr/Src/stm32f407xx_rcc.c \
../Drivers/dstr/Src/stm32f407xx_rtc.c \
../Drivers/dstr/Src/stm32f407xx_spi.c \
../Drivers/dstr/Src/stm32f407xx_system.c \
../Drivers/dstr/Src/stm32f407xx_tim.c \
../Drivers/dstr/Src/stm32f407xx_usart.c 

OBJS += \
./Drivers/dstr/Src/stm32f407xx_gpio.o \
./Drivers/dstr/Src/stm32f407xx_i2c.o \
./Drivers/dstr/Src/stm32f407xx_rcc.o \
./Drivers/dstr/Src/stm32f407xx_rtc.o \
./Drivers/dstr/Src/stm32f407xx_spi.o \
./Drivers/dstr/Src/stm32f407xx_system.o \
./Drivers/dstr/Src/stm32f407xx_tim.o \
./Drivers/dstr/Src/stm32f407xx_usart.o 

C_DEPS += \
./Drivers/dstr/Src/stm32f407xx_gpio.d \
./Drivers/dstr/Src/stm32f407xx_i2c.d \
./Drivers/dstr/Src/stm32f407xx_rcc.d \
./Drivers/dstr/Src/stm32f407xx_rtc.d \
./Drivers/dstr/Src/stm32f407xx_spi.d \
./Drivers/dstr/Src/stm32f407xx_system.d \
./Drivers/dstr/Src/stm32f407xx_tim.d \
./Drivers/dstr/Src/stm32f407xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/dstr/Src/%.o Drivers/dstr/Src/%.su Drivers/dstr/Src/%.cyclo: ../Drivers/dstr/Src/%.c Drivers/dstr/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../LIBJPEG/App -I../LIBJPEG/Target -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/LibJPEG/include -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/Inc" -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/Src" -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/dstr/my libs" -I"D:/Users/nabla/STM32CubeIDE/TI_like_workspace/STM32_VIDEO_SURVEILLANCE/Drivers/bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-dstr-2f-Src

clean-Drivers-2f-dstr-2f-Src:
	-$(RM) ./Drivers/dstr/Src/stm32f407xx_gpio.cyclo ./Drivers/dstr/Src/stm32f407xx_gpio.d ./Drivers/dstr/Src/stm32f407xx_gpio.o ./Drivers/dstr/Src/stm32f407xx_gpio.su ./Drivers/dstr/Src/stm32f407xx_i2c.cyclo ./Drivers/dstr/Src/stm32f407xx_i2c.d ./Drivers/dstr/Src/stm32f407xx_i2c.o ./Drivers/dstr/Src/stm32f407xx_i2c.su ./Drivers/dstr/Src/stm32f407xx_rcc.cyclo ./Drivers/dstr/Src/stm32f407xx_rcc.d ./Drivers/dstr/Src/stm32f407xx_rcc.o ./Drivers/dstr/Src/stm32f407xx_rcc.su ./Drivers/dstr/Src/stm32f407xx_rtc.cyclo ./Drivers/dstr/Src/stm32f407xx_rtc.d ./Drivers/dstr/Src/stm32f407xx_rtc.o ./Drivers/dstr/Src/stm32f407xx_rtc.su ./Drivers/dstr/Src/stm32f407xx_spi.cyclo ./Drivers/dstr/Src/stm32f407xx_spi.d ./Drivers/dstr/Src/stm32f407xx_spi.o ./Drivers/dstr/Src/stm32f407xx_spi.su ./Drivers/dstr/Src/stm32f407xx_system.cyclo ./Drivers/dstr/Src/stm32f407xx_system.d ./Drivers/dstr/Src/stm32f407xx_system.o ./Drivers/dstr/Src/stm32f407xx_system.su ./Drivers/dstr/Src/stm32f407xx_tim.cyclo ./Drivers/dstr/Src/stm32f407xx_tim.d ./Drivers/dstr/Src/stm32f407xx_tim.o ./Drivers/dstr/Src/stm32f407xx_tim.su ./Drivers/dstr/Src/stm32f407xx_usart.cyclo ./Drivers/dstr/Src/stm32f407xx_usart.d ./Drivers/dstr/Src/stm32f407xx_usart.o ./Drivers/dstr/Src/stm32f407xx_usart.su

.PHONY: clean-Drivers-2f-dstr-2f-Src

