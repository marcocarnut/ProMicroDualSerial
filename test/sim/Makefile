# Cycle-level simulation test of the soft UART with simavr: the real
# firmware drivers (asm ISRs included) run in a simulated ATmega32U4 with
# the soft UART's TX looped back into its RX.
#
# Needs a recent simavr built from source. 1.6, as packaged by Debian/Ubuntu,
# models Timer1's compare output and multiple pending interrupts wrongly;
# upstream master still fires a compare at OCR1x <= ~2 right after an
# overflow one period late (rare, but this suite hits it). Recommended: the
# "fixes" branch below, which has that fixed and also charges the 4 cycles
# of interrupt entry like the silicon does:
#   git clone -b fixes https://github.com/marcocarnut/simavr
#   make -C simavr/simavr libsimavr
#   make -j SIMAVR_INC=simavr/simavr/sim \
#        SIMAVR_LIB="simavr/simavr/obj-x86_64-linux-gnu/libsimavr.a -lelf"

SIMAVR_INC ?= /usr/include/simavr
SIMAVR_LIB ?= -l:libsimavr.so.2 -lelf

ROOT     = ../..
SRC      = $(ROOT)/src
LUFA     = $(ROOT)/lufa
BUILD    = build$(if $(filter-out 0,$(SU_TX_MODE)),_tx$(SU_TX_MODE))
# SU_TX_MODE=3 tests the per-bit Timer3 transmitter (see src/config.h)
SU_TX_MODE ?= 0
RESERVED = $(if $(filter-out 0,$(SU_TX_MODE)),-DSU_REGS_RESERVED $(foreach r,2 3 4 5 6,-ffixed-r$(r)))
MCUFLAGS = $(EXTRA) -DSU_TX_MODE=$(SU_TX_MODE) $(RESERVED) -mmcu=atmega32u4 -DF_CPU=16000000UL -DF_USB=16000000UL -DARCH=ARCH_AVR8 \
           -DUSE_LUFA_CONFIG_HEADER -I$(ROOT)/Config -I$(SRC) -I$(LUFA) -Os -std=gnu99 \
           -ffunction-sections -fdata-sections -Wl,--gc-sections -Wall \
           -DP0_HW_FLOW=0 -DP1_HW_FLOW=2
FW_SRC   = sim_fw.c $(SRC)/hwuart.c $(SRC)/softuart.c $(SRC)/softuart_codec.c $(SRC)/softuart_dec.S $(SRC)/softuart_enc.S $(SRC)/clock.c

# name:baud:bits:parity:stop:pattern:count[:env,env...]
# Data is offered as fast as the firmware accepts it (LOAD=1, with the
# firmware's "ready" pin standing in for USB flow control). With RXGEN=1 an
# independent full-rate stream drives the soft RX pin instead of looping TX
# back, so RX has to keep up at 100% while TX gets what CPU is left.

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
	300_7O2_random:300:7:1:2:random:30 \
	rxgen_115200_8N1_random:115200:8:0:0:random:3000:RXGEN=1 \
	rxgen_115200_8N1_55:115200:8:0:0:55:3000:RXGEN=1 \
	rxgen_57600_7E1_ascii:57600:7:2:0:ascii:2000:RXGEN=1 \
	rxgen_115200_7O1_random:115200:7:1:0:random:3000:RXGEN=1 \
	rxgen_115200_8E1_random:115200:8:2:0:random:3000:RXGEN=1 \
	rxgen_115200_8S1_random:115200:8:4:0:random:3000:RXGEN=1 \
	rxgen_115200_7S1_random:115200:7:4:0:random:3000:RXGEN=1 \
	rxgen_117647_8N1_random:115200:8:0:0:random:3000:RXGEN=1,RXGEN_BAUD=117647 \
	rxgen_117647_8O1_random:115200:8:1:0:random:3000:RXGEN=1,RXGEN_BAUD=117647 \
	cts_115200_8N1_random:115200:8:0:0:random:2000:CTS_TOGGLE=1 \
	cts_9600_8E2_random:9600:8:2:2:random:300:CTS_TOGGLE=1 \
	cts_unconnected_115200:115200:8:0:0:random:1000:CTS_NEVER=1

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

comma := ,

# one result file per case, so `make -j` runs them in parallel
define case_rule
$(BUILD)/res_$(1).txt: $(BUILD)/harness $(BUILD)/fw_$(1).elf
	@env LOAD=1 TXTIMER=$(if $(filter 3,$(SU_TX_MODE)),3,1) $(subst $(comma), ,$(8)) $(BUILD)/harness $(BUILD)/fw_$(1).elf $(2) $(3) $(4) $(5) $(6) $(7) > $$@.tmp 2>&1; mv $$@.tmp $$@
endef
$(foreach c,$(CASES),$(eval $(call case_rule,$(word 1,$(subst :, ,$(c))),$(word 2,$(subst :, ,$(c))),$(word 3,$(subst :, ,$(c))),$(word 4,$(subst :, ,$(c))),$(word 5,$(subst :, ,$(c))),$(word 6,$(subst :, ,$(c))),$(word 7,$(subst :, ,$(c))),$(word 8,$(subst :, ,$(c))))))

RESULTS = $(BUILD)/res_decoder_asm_vs_c.txt \
          $(foreach c,$(CASES),$(BUILD)/res_$(word 1,$(subst :, ,$(c))).txt)

# the asm decoder against the C reference, both running on the simulated AVR
$(BUILD)/dec_harness: dec_harness.c
	@mkdir -p $(BUILD)
	$(CC) -O2 -Wall -I$(SIMAVR_INC) -o $@ $< $(SIMAVR_LIB) -lm

$(BUILD)/dec_test_fw.elf: dec_test_fw.c $(SRC)/softuart_codec.c $(SRC)/softuart_dec.S $(SRC)/softuart_enc.S $(SRC)/softuart_codec.h
	@mkdir -p $(BUILD)
	avr-gcc $(MCUFLAGS) -DSU_DEC_C -DSU_ENC_C -o $@ dec_test_fw.c $(SRC)/softuart_codec.c $(SRC)/softuart_dec.S $(SRC)/softuart_enc.S

$(BUILD)/res_decoder_asm_vs_c.txt: $(BUILD)/dec_harness $(BUILD)/dec_test_fw.elf
	@$(BUILD)/dec_harness $(BUILD)/dec_test_fw.elf > $@.tmp 2>&1; mv $@.tmp $@

run: $(RESULTS)
	@fail=0; for r in $(RESULTS); do \
		n=$$(basename $$r .txt); n=$${n#res_}; \
		v=$$(tail -n1 $$r); \
		printf '%-28s %-4s %s\n' "$$n" "$$v" "$$(grep -h 'throughput' $$r | sed 's/tx throughput: //')"; \
		[ "$$v" = PASS ] || { grep -E 'error flag|mismatch|violation' $$r | sed 's/^/    /'; fail=1; }; \
	done; exit $$fail

clean:
	rm -rf $(BUILD)

rerun:
	rm -f $(BUILD)/res_*.txt
	$(MAKE) run

.PHONY: all run rerun clean
