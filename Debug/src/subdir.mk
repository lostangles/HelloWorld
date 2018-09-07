################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lib_exti.c \
../src/lib_gpio.c \
../src/lib_i2c.c \
../src/lib_uart.c \
../src/main.c \
../src/mpu6050.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/lib_exti.o \
./src/lib_gpio.o \
./src/lib_i2c.o \
./src/lib_uart.o \
./src/main.o \
./src/mpu6050.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/lib_exti.d \
./src/lib_gpio.d \
./src/lib_i2c.d \
./src/lib_uart.d \
./src/main.d \
./src/mpu6050.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -DSTM32 -DSTM32F4 -DSTM32F429ZITx -DNUCLEO_F429ZI -DDEBUG -I/home/branjb/workspace/HelloWorld/inc -I/home/branjb/.ac6/SW4STM32/firmwares/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Include -I/home/branjb/.ac6/SW4STM32/firmwares/STM32Cube_FW_F4_V1.21.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/home/branjb/.ac6/SW4STM32/firmwares/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -Og -g3 -Wall -Wparentheses -fmessage-length=0 -DSTM32F429xx -fpermissive -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/main.o: ../src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -DSTM32 -DSTM32F4 -DSTM32F429ZITx -DNUCLEO_F429ZI -DDEBUG -I/home/branjb/workspace/HelloWorld/inc -I/home/branjb/.ac6/SW4STM32/firmwares/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Include -I/home/branjb/.ac6/SW4STM32/firmwares/STM32Cube_FW_F4_V1.21.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -Og -g3 -Wall -Wparentheses -fmessage-length=0 -DSTM32F429xx -fpermissive -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


