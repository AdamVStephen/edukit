################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/Components/l6474/l6474.c 

OBJS += \
./Drivers/BSP/Components/l6474/l6474.o 

C_DEPS += \
./Drivers/BSP/Components/l6474/l6474.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/l6474/l6474.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/Components/l6474/l6474.c Drivers/BSP/Components/l6474/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-l6474

clean-Drivers-2f-BSP-2f-Components-2f-l6474:
	-$(RM) ./Drivers/BSP/Components/l6474/l6474.cyclo ./Drivers/BSP/Components/l6474/l6474.d ./Drivers/BSP/Components/l6474/l6474.o ./Drivers/BSP/Components/l6474/l6474.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-l6474

