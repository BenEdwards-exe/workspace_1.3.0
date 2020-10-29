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
a1-prep.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
a1-prep.list \

OBJCOPY_BIN += \
a1-prep.bin \


# All Target
all: a1-prep.elf secondary-outputs

# Tool invocations
a1-prep.elf: $(OBJS) $(USER_OBJS) C:\Users\benjr\STM32CubeIDE\workspace_1.3.0\a1-prep\STM32F411VETX_FLASH.ld
	arm-none-eabi-gcc -o "a1-prep.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\benjr\STM32CubeIDE\workspace_1.3.0\a1-prep\STM32F411VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="a1-prep.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

a1-prep.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "a1-prep.list"
	@echo 'Finished building: $@'
	@echo ' '

a1-prep.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "a1-prep.bin"
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
