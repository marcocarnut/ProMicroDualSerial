#
# ProMicroDualUART - dual USB CDC <-> UART bridge for the ATmega32U4 (Pro Micro)
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
TARGET       = ProMicroDualUART
SRC          = src/main.c src/Descriptors.c src/clock.c src/hwuart.c src/softuart.c src/softuart_codec.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = lufa/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -Isrc/ -Wextra -Wno-unused-parameter
LD_FLAGS     =

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
