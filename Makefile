##
## This file is part of the libopenstm32 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

# FAMILY=f1 is the STM32F1 bootloader and the default (`make`).
# `make f4` builds the STM32F407 image, stm32_loader_f407, linked into
# sector 0 only. HSE_HZ is only used by the F4 clock. 8000000 matches
# the F107 boards. Use 25000000 only when that crystal has been measured.
# Run `make clean` before switching HSE_HZ: object files are not rebuilt
# when only that value changes.
FAMILY ?= f1
HSE_HZ ?= 8000000

ifeq ($(FAMILY),f4)
CPUFLAGS        = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCHDEF         = -DSTM32F4 -DHSE_HZ=$(HSE_HZ)
BINARY          = stm32_loader_f407
OUT_DIR         = obj-f4
LDSCRIPT        = stm32_loader_f407.ld
OPENCM3_LIB     = opencm3_stm32f4
MAPFILE         = linker-f4.map
else ifeq ($(FAMILY),f1)
CPUFLAGS        = -mcpu=cortex-m3 -mthumb
ARCHDEF         =
BINARY          = stm32_loader
OUT_DIR         = obj
LDSCRIPT        = stm32_loader.ld
OPENCM3_LIB     = opencm3_stm32f1
MAPFILE         = linker.map
else
$(error FAMILY must be f1 or f4)
endif

PREFIX		?= arm-none-eabi
SIZE		= $(PREFIX)-size
CC		= $(PREFIX)-gcc
CPP		= $(PREFIX)-g++
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
CFLAGS		= -Os -Wall -Wextra -Ilibopencm3/include -fno-common -fno-builtin \
		  $(CPUFLAGS) $(ARCHDEF) -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
CPPFLAGS	= -Os -Wall -Wextra -Ilibopencm3/include -fno-common \
		  -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions \
		  -fno-unwind-tables $(CPUFLAGS) $(ARCHDEF) -ggdb3
LDFLAGS		= -Llibopencm3/lib -T$(LDSCRIPT) $(CPUFLAGS) -nostartfiles \
		  -Wl,--gc-sections,-Map,$(MAPFILE) -ggdb3
OBJS		= $(OUT_DIR)/stm32_loader.o

OPENOCD_BASE	= /usr
OPENOCD		= $(OPENOCD_BASE)/bin/openocd
OPENOCD_SCRIPTS	= $(OPENOCD_BASE)/share/openocd/scripts
OPENOCD_FLASHER	= $(OPENOCD_SCRIPTS)/interface/parport.cfg
OPENOCD_BOARD	= $(OPENOCD_SCRIPTS)/board/olimex_stm32_h103.cfg

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

# Test whether this binutils can suppress the rwx-segment note.
# The note is from the generic libopencm3 script and does not change the image.
comma := ,
link_command := -Wl$(comma)
try-run = $(shell set -e; if ($(1)) >/dev/null 2>&1; then echo "$(2)"; else echo "$(3)"; fi)
ld-option = $(call try-run, $(PREFIX)-ld $(1) -v,$(link_command)$(1))
LDFLAGS += $(call ld-option,--no-warn-rwx-segments)

all: images
Debug: images
cleanDebug: clean

images: $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	@printf "  OBJCOPY $(BINARY).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(BINARY) $(BINARY).hex
	$(Q)$(SIZE) $(BINARY)
ifeq ($(FAMILY),f4)
	@bytes=$$(wc -c < $(BINARY).bin); \
	 if [ "$$bytes" -gt 16384 ]; then \
	   echo "error: $(BINARY).bin is $$bytes bytes; sector 0 is 16384" >&2; \
	   exit 1; \
	 fi; \
	 echo "F407 bootloader fits in sector 0 ($$bytes / 16384 bytes)"
endif

$(OUT_DIR):
	$(Q)mkdir -p $(OUT_DIR)

libopencm3/Makefile:
	git clone https://github.com/jsphuebner/libopencm3.git libopencm3

libopencm3/lib/libopencm3_stm32f1.a: libopencm3/Makefile
	$(MAKE) -C libopencm3 TARGETS=stm32/f1

libopencm3/lib/libopencm3_stm32f4.a: libopencm3/Makefile
	$(MAKE) -C libopencm3 TARGETS=stm32/f4

$(BINARY): $(OBJS) $(LDSCRIPT) libopencm3/lib/lib$(OPENCM3_LIB).a
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -l$(OPENCM3_LIB)

$(OUT_DIR)/stm32_loader.o: stm32_loader.c Makefile | $(OUT_DIR)
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@printf "  CLEAN   objects and images\n"
	$(Q)rm -rf obj obj-f4
	$(Q)rm -f stm32_loader.o stm32_loader stm32_loader.bin stm32_loader.hex stm32_loader.srec stm32_loader.list linker.map
	$(Q)rm -f stm32_loader_f407 stm32_loader_f407.bin stm32_loader_f407.hex stm32_loader_f407.srec stm32_loader_f407.list linker-f4.map

# The OpenOCD board file below is the old F103 recipe. Do not point it
# at an F407. Program stm32_loader_f407.bin at 0x08000000 with a debugger.
ifneq ($(FAMILY),f4)
flash: images
	@printf "  FLASH   $(BINARY).bin\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OPENOCD) -s $(OPENOCD_SCRIPTS) \
		       -f $(OPENOCD_FLASHER) \
		       -f $(OPENOCD_BOARD) \
		       -c "init" -c "reset halt" \
		       -c "flash write_image erase $(BINARY).hex" \
		       -c "reset" \
		       -c "shutdown" $(NULL)
else
flash:
	@echo "This OpenOCD recipe is the F1 board file. It is not for the F407." >&2
	@echo "Program stm32_loader_f407.bin at 0x08000000 (sector 0 only)." >&2
	@false
endif

.PHONY: images clean get-deps f4 flash

# Build the F407 image without reusing F1 objects.
ifneq ($(FAMILY),f4)
f4:
	$(MAKE) FAMILY=f4 HSE_HZ=$(HSE_HZ) all
endif

get-deps: libopencm3/lib/libopencm3_stm32f1.a libopencm3/lib/libopencm3_stm32f4.a
