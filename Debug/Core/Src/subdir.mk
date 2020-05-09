################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stm32f4xx_nucleo_bus.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/stm32f4xx_nucleo_bus.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/stm32f4xx_nucleo_bus.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/main.o: ../Core/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/stm32f4xx_hal_msp.o: ../Core/Src/stm32f4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/stm32f4xx_it.o: ../Core/Src/stm32f4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/stm32f4xx_nucleo_bus.o: ../Core/Src/stm32f4xx_nucleo_bus.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_nucleo_bus.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Core/Src/system_stm32f4xx.o: ../Core/Src/system_stm32f4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/includes -I../MEMS/Target -I../Middlewares/ST/Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Template -I../Drivers/BSP/Components/hts221 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/BlueNRG-MS/utils -I../Drivers/BSP/STM32F4xx_Nucleo -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../BlueNRG_MS/Target -I../Drivers/BSP/Components/lps22hb -I../Middlewares/ST/STM32_MotionFX_Library/Inc -I../MEMS/App -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-MS/includes -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/IKS01A2 -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/BSP/Components/Common -I../BlueNRG_MS/App -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
