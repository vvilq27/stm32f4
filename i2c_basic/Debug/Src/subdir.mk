################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dwt_delay.c \
../Src/main.c \
../Src/ssd1306.c \
../Src/ssd1306_fonts.c \
../Src/ssd1306_tests.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/dwt_delay.o \
./Src/main.o \
./Src/ssd1306.o \
./Src/ssd1306_fonts.o \
./Src/ssd1306_tests.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/dwt_delay.d \
./Src/main.d \
./Src/ssd1306.d \
./Src/ssd1306_fonts.d \
./Src/ssd1306_tests.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/Users/arkadiusz.zelazowski/STMworkspace/i2c_basic/Inc" -I"C:/Users/arkadiusz.zelazowski/STMworkspace/i2c_basic/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/arkadiusz.zelazowski/STMworkspace/i2c_basic/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/arkadiusz.zelazowski/STMworkspace/i2c_basic/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/arkadiusz.zelazowski/STMworkspace/i2c_basic/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

