################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.o: ../Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F4xx_Nucleo/stm32f4xx_nucleo.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

