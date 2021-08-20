################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f446retx.s 

C_SRCS += \
../Startup/sysmem.c 

OBJS += \
./Startup/startup_stm32f446retx.o \
./Startup/sysmem.o 

S_DEPS += \
./Startup/startup_stm32f446retx.d 

C_DEPS += \
./Startup/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Startup/startup_stm32f446retx.o: ../Startup/startup_stm32f446retx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Startup/startup_stm32f446retx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Startup/sysmem.o: ../Startup/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"D:/embedded systems/mcu-1/gpio project/stm32f446xx_drivers/drivers/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Startup/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

