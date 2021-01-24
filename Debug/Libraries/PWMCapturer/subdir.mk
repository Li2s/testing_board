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
Libraries/PWMCapturer/PWMCapturer.o: C:/projects/STM32/testing_board/Libraries/PWMCapturer/PWMCapturer.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Libraries -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Libraries/PWMCapturer/PWMCapturer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

