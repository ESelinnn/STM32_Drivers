################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/010i2c_master_tx_testing.c 

OBJS += \
./Src/010i2c_master_tx_testing.o 

C_DEPS += \
./Src/010i2c_master_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/010i2c_master_tx_testing.o: ../Src/010i2c_master_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/MCU1-Course/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/010i2c_master_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

