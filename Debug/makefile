################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ST/BlueNRG-MS/utils/subdir.mk
-include Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic/subdir.mk
-include Middlewares/ST/BlueNRG-MS/hci/controller/subdir.mk
-include Middlewares/ST/BlueNRG-MS/hci/subdir.mk
-include MEMS/Target/subdir.mk
-include MEMS/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32F4xx_Nucleo/subdir.mk
-include Drivers/BSP/IKS01A2/subdir.mk
-include Drivers/BSP/Components/lsm6dsl/subdir.mk
-include Drivers/BSP/Components/lsm303agr/subdir.mk
-include Drivers/BSP/Components/lps22hb/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include BlueNRG_MS/Target/subdir.mk
-include BlueNRG_MS/App/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
IMSI2020_ProjectFX.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
IMSI2020_ProjectFX.list \

OBJCOPY_BIN += \
IMSI2020_ProjectFX.bin \


# All Target
all: IMSI2020_ProjectFX.elf secondary-outputs

# Tool invocations
IMSI2020_ProjectFX.elf: $(OBJS) $(USER_OBJS) C:\Users\pasqu\STM32CubeIDE\workspace_1.3.0\IMSI2020_ProjectFX\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "IMSI2020_ProjectFX.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\pasqu\STM32CubeIDE\workspace_1.3.0\IMSI2020_ProjectFX\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="IMSI2020_ProjectFX.map" -Wl,--gc-sections -static -L"C:\Users\pasqu\STM32CubeIDE\workspace_1.3.0\IMSI2020_ProjectFX\Middlewares\ST\STM32_MotionFX_Library\Lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

IMSI2020_ProjectFX.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "IMSI2020_ProjectFX.list"
	@echo 'Finished building: $@'
	@echo ' '

IMSI2020_ProjectFX.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "IMSI2020_ProjectFX.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
