################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include uart/subdir.mk
-include mpu6050/subdir.mk
-include i2chw/subdir.mk
-include bldc/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Prisma_Block.lss \

FLASH_IMAGE += \
Prisma_Block.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: Prisma_Block.elf secondary-outputs

# Tool invocations
Prisma_Block.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Prisma_Block.map -Wl,-u,vfprintf -mmcu=atmega328p -o "Prisma_Block.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Prisma_Block.lss: Prisma_Block.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Prisma_Block.elf  >"Prisma_Block.lss"
	@echo 'Finished building: $@'
	@echo ' '

Prisma_Block.hex: Prisma_Block.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Prisma_Block.elf  "Prisma_Block.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Prisma_Block.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Prisma_Block.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Prisma_Block.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
