################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm01a1_stm32f4xx.c 

OBJS += \
./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o \
./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.o 

C_DEPS += \
./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d \
./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm01a1_stm32f4xx.c Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32F4xx-2d-Nucleo

clean-Drivers-2f-BSP-2f-STM32F4xx-2d-Nucleo:
	-$(RM) ./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.cyclo ./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.d ./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o ./Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.su ./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.cyclo ./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.d ./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.o ./Drivers/BSP/STM32F4xx-Nucleo/x_nucleo_ihm01a1_stm32f4xx.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32F4xx-2d-Nucleo

