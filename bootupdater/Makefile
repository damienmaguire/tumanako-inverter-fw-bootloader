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

# `make` is the F1 bootloader updater, linked at 0x08001000.
# `make f4` is the F407 updater, linked at 0x08010000. It writes only
# sector 0 (16 KiB). HSE_HZ is 8000000 unless the crystal is measured
# at 25 MHz. Run `make clean` before switching HSE_HZ.
FAMILY ?= f1
HSE_HZ ?= 8000000

ifeq ($(FAMILY),f4)
CPUFLAGS        = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCHDEF         = -DSTM32F4 -DHSE_HZ=$(HSE_HZ)
BINARY          = stm32_bootupdater_f407
OUT_DIR         = obj-f4
LDSCRIPT        = stm32_bootupdater_f407.ld
OPENCM3_LIB     = opencm3_stm32f4
MAPFILE         = linker-f4.map
else ifeq ($(FAMILY),f1)
CPUFLAGS        = -mcpu=cortex-m3 -mthumb
ARCHDEF         =
BINARY          = stm32_bootupdater
OUT_DIR         = obj
LDSCRIPT        = stm32_bootupdater.ld
OPENCM3_LIB     = opencm3_stm32f1
MAPFILE         = linker.map
else
$(error FAMILY must be f1 or f4)
endif

PREFIX		?= arm-none-eabi
SIZE		= $(PREFIX)-size
CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
CFLAGS		= -Os -Wall -Wextra -I../libopencm3/include -fno-common -fno-builtin \
		  $(CPUFLAGS) $(ARCHDEF) -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
LDFLAGS		= -L../libopencm3/lib -T$(LDSCRIPT) $(CPUFLAGS) -nostartfiles \
		  -Wl,--gc-sections,-Map,$(MAPFILE) -ggdb3
OBJS		= $(OUT_DIR)/stm32_bootupdater.o

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

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
	 if [ "$$bytes" -gt 65536 ]; then \
	   echo "error: $(BINARY).bin is $$bytes bytes; it must stay in sector 4" >&2; \
	   exit 1; \
	 fi
endif

$(OUT_DIR):
	$(Q)mkdir -p $(OUT_DIR)

$(BINARY): $(OBJS) $(LDSCRIPT) ../libopencm3/lib/lib$(OPENCM3_LIB).a
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -l$(OPENCM3_LIB)

$(OUT_DIR)/stm32_bootupdater.o: stm32_bootupdater.c Makefile | $(OUT_DIR)
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@printf "  CLEAN   objects and images\n"
	$(Q)rm -rf obj obj-f4
	$(Q)rm -f stm32_bootupdater.o stm32_bootupdater stm32_bootupdater.bin stm32_bootupdater.hex linker.map
	$(Q)rm -f stm32_bootupdater_f407 stm32_bootupdater_f407.bin stm32_bootupdater_f407.hex linker-f4.map

.PHONY: images clean f4

ifneq ($(FAMILY),f4)
f4:
	$(MAKE) FAMILY=f4 HSE_HZ=$(HSE_HZ) all
endif
