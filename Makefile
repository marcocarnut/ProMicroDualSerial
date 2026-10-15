#
# ProMicroDualSerial - dual USB CDC <-> UART bridge for the ATmega32U4 (Pro Micro)
#
# Build:   make
# Flash:   make flash            (Caterina bootloader, via 1200 baud touch)
# Tests:   make -C test           (host-side unit tests)
#

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = NONE
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = ProMicroDualSerial
SRC          = src/main.c src/Descriptors.c src/clock.c src/hwuart.c src/softuart.c src/softuart_codec.c src/softuart_dec.S src/profiler.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = lufa/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -Isrc/ -Wextra -Wno-unused-parameter
# The soft UART's 256-byte aligned rings (.noinit) go at the start of RAM,
# which is aligned already, instead of after .bss with up to 255 bytes of
# padding; .data and .bss follow them.
LD_FLAGS     = -Wl,--section-start=.noinit=0x800100 -Wl,-Tdata=0x800300

# make BOARD_BEETLE=1: build for the Beetle board (see src/config.h)
ifdef BOARD_BEETLE
CC_FLAGS    += -DBOARD_BEETLE=$(BOARD_BEETLE)
endif

# extra compiler flags, e.g. make clean && make EXTRA=-DTX_ENCODE_BATCH=2
CC_FLAGS    += $(EXTRA)

# r2-r7 belong to the soft UART's ISRs (see src/softuart.c): nothing else
# may use them
CC_FLAGS    += -DSU_REGS_RESERVED $(foreach r,2 3 4 5 6 7,-ffixed-r$(r))

# Caterina (Arduino / SparkFun Pro Micro) bootloader
FLASH_PORT        ?= /dev/ttyACM0
AVRDUDE_PROGRAMMER = avr109
AVRDUDE_PORT      ?= $(FLASH_PORT)

# Default target
all:

flash: $(TARGET).hex
	tools/flash.sh $(FLASH_PORT) $(TARGET).hex

.PHONY: flash

# Include LUFA-specific DMBS extension modules
DMBS_LUFA_PATH ?= $(LUFA_PATH)/Build/LUFA
include $(DMBS_LUFA_PATH)/lufa-sources.mk
include $(DMBS_LUFA_PATH)/lufa-gcc.mk

# Include common DMBS build system modules
DMBS_PATH      ?= $(LUFA_PATH)/Build/DMBS/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/avrdude.mk
