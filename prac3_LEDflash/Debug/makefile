################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
prac3_LEDflash.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
prac3_LEDflash.list \

OBJCOPY_BIN += \
prac3_LEDflash.bin \


# All Target
all: prac3_LEDflash.elf secondary-outputs

# Tool invocations
prac3_LEDflash.elf: $(OBJS) $(USER_OBJS) C:\Users\benjr\STM32CubeIDE\workspace_1.3.0\prac3_LEDflash\STM32F411VETX_FLASH.ld
	arm-none-eabi-gcc -o "prac3_LEDflash.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\benjr\STM32CubeIDE\workspace_1.3.0\prac3_LEDflash\STM32F411VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="prac3_LEDflash.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

prac3_LEDflash.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "prac3_LEDflash.list"
	@echo 'Finished building: $@'
	@echo ' '

prac3_LEDflash.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "prac3_LEDflash.bin"
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
