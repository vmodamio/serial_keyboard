CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections -Wno-shadow
CFLAGS += -I. -Iinclude -Icmsis_core/CMSIS/Core/Include -Icmsis_l4/Include
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c syscalls.c 
SOURCES += cmsis_l4/Source/Templates/gcc/startup_stm32l412xx.s # ST startup file. Compiler-dependent!

ifeq ($(OS),Windows_NT)
  RM = cmd /C del /Q /F
else
  RM = rm -rf
endif

build: firmware.bin

firmware.elf: cmsis_core cmsis_l4 input-event-codes.h hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(CFLAGS_EXTRA) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x8000000

cmsis_core:
	git clone --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@

cmsis_l4:
	git clone --depth 1 -b v1.7.2 https://github.com/STMicroelectronics/cmsis_device_l4 $@

clean:
	$(RM) firmware.* cmsis_*

