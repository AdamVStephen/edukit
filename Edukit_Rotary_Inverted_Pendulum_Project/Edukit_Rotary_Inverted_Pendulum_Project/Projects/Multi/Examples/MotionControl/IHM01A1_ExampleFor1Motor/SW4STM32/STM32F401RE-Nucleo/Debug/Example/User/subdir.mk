################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f4.c \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/edukit_system.c \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c \
D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c 

OBJS += \
./Example/User/clock_f4.o \
./Example/User/edukit_system.o \
./Example/User/main.o \
./Example/User/stm32f4xx_hal_msp.o \
./Example/User/stm32f4xx_it.o 

C_DEPS += \
./Example/User/clock_f4.d \
./Example/User/edukit_system.d \
./Example/User/main.d \
./Example/User/stm32f4xx_hal_msp.d \
./Example/User/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Example/User/clock_f4.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/clock_f4.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/edukit_system.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/edukit_system.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/main.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_hal_msp.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Example/User/stm32f4xx_it.o: D:/EduKit/Edukit_Rotary_Inverted_Pendulum_Project/Edukit_Rotary_Inverted_Pendulum_Project/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c Example/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO -DNUCLEO_F401RE -DSTM32F401RETx -DSTM32 -DSTM32F4 -DDEBUG -c -I"../../../Inc" -I"../../../../../../../../Drivers/BSP/STM32F4xx-Nucleo" -I"../../../../../../../../Drivers/BSP/X-NUCLEO-IHMxx" -I"../../../../../../../../Drivers/BSP/Components/Common" -I"../../../../../../../../Drivers/BSP/Components/l6474" -I"../../../../../../../../Drivers/CMSIS/Include" -I"../../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"../../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc" -O0 -ffunction-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example-2f-User

clean-Example-2f-User:
	-$(RM) ./Example/User/clock_f4.cyclo ./Example/User/clock_f4.d ./Example/User/clock_f4.o ./Example/User/clock_f4.su ./Example/User/edukit_system.cyclo ./Example/User/edukit_system.d ./Example/User/edukit_system.o ./Example/User/edukit_system.su ./Example/User/main.cyclo ./Example/User/main.d ./Example/User/main.o ./Example/User/main.su ./Example/User/stm32f4xx_hal_msp.cyclo ./Example/User/stm32f4xx_hal_msp.d ./Example/User/stm32f4xx_hal_msp.o ./Example/User/stm32f4xx_hal_msp.su ./Example/User/stm32f4xx_it.cyclo ./Example/User/stm32f4xx_it.d ./Example/User/stm32f4xx_it.o ./Example/User/stm32f4xx_it.su

.PHONY: clean-Example-2f-User

