# Cycle-level simulation test of the soft UART with simavr: the real
# firmware drivers (asm ISRs included) run in a simulated ATmega32U4 with
# the soft UART's TX looped back into its RX.
#
# Needs libsimavr and its headers (Debian/Ubuntu: libsimavr-dev), or point
# SIMAVR_INC at simavr's source tree "simavr/sim" directory.

SIMAVR_INC ?= /usr/include/simavr
SIMAVR_LIB ?= -l:libsimavr.so.2 -lelf

ROOT     = ../..
SRC      = $(ROOT)/src
LUFA     = $(ROOT)/lufa
BUILD    = build
MCUFLAGS = -mmcu=atmega32u4 -DF_CPU=16000000UL -DF_USB=16000000UL -DARCH=ARCH_AVR8 \
           -DUSE_LUFA_CONFIG_HEADER -I$(ROOT)/Config -I$(SRC) -I$(LUFA) -Os -std=gnu99 \
           -ffunction-sections -fdata-sections -Wl,--gc-sections -Wall
FW_SRC   = sim_fw.c $(SRC)/hwuart.c $(SRC)/softuart.c $(SRC)/softuart_codec.c $(SRC)/clock.c

# name:baud:bits:parity:stop:pattern:count
CASES = \
	115200_8N1_random:115200:8:0:0:random:2000 \
	115200_8N1_55:115200:8:0:0:55:2000 \
	115200_8N1_00:115200:8:0:0:00:500 \
	115200_8N1_ff:115200:8:0:0:ff:500 \
	115200_7E1_ascii:115200:7:2:0:ascii:1000 \
	115200_8O2_random:115200:8:1:2:random:1000 \
	57600_8M1_random:57600:8:3:0:random:1000 \
	38400_5S15_random:38400:5:4:1:random:1000 \
	9600_8N1_random:9600:8:0:0:random:300 \
	2400_8E1_random:2400:8:2:0:random:100 \
	300_7O2_random:300:7:1:2:random:30

all: run

$(BUILD)/harness: harness.c
	@mkdir -p $(BUILD)
	$(CC) -O2 -Wall -I$(SIMAVR_INC) -o $@ $< $(SIMAVR_LIB) -lm

# one firmware per line coding
define fw_rule
$(BUILD)/fw_$(1).elf: $(FW_SRC) $(wildcard $(SRC)/*.h)
	@mkdir -p $(BUILD)
	avr-gcc $(MCUFLAGS) -DSOFT_BAUD=$(2) -DSOFT_BITS=$(3) -DSOFT_PARITY=$(4) -DSOFT_STOP=$(5) -o $$@ $(FW_SRC)
endef
$(foreach c,$(CASES),$(eval $(call fw_rule,$(word 1,$(subst :, ,$(c))),$(word 2,$(subst :, ,$(c))),$(word 3,$(subst :, ,$(c))),$(word 4,$(subst :, ,$(c))),$(word 5,$(subst :, ,$(c))))))

run: $(BUILD)/harness $(foreach c,$(CASES),$(BUILD)/fw_$(word 1,$(subst :, ,$(c))).elf)
	@fail=0; for c in $(CASES); do \
		set -- $$(echo $$c | tr ':' ' '); \
		echo "== $$1"; \
		$(BUILD)/harness $(BUILD)/fw_$$1.elf $$2 $$3 $$4 $$5 $$6 $$7 || fail=1; \
	done; exit $$fail

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
