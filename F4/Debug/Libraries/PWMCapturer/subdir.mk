################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/PWMCapturer/PWMCapturer.cpp 

OBJS += \
./Libraries/PWMCapturer/PWMCapturer.o 

CPP_DEPS += \
./Libraries/PWMCapturer/PWMCapturer.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/PWMCapturer/PWMCapturer.o: E:/Projects/testing_board/F4/Libraries/PWMCapturer/PWMCapturer.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Libraries/ -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Libraries/PWMCapturer/PWMCapturer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

