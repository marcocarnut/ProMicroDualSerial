# Host-side unit tests for the hardware independent parts of the firmware.
CC      ?= cc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=c99 -fsanitize=address,undefined
BUILD    = build

all: run

$(BUILD)/test_codec: test_codec.c ../src/softuart_codec.c ../src/softuart_codec.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I../src -o $@ test_codec.c ../src/softuart_codec.c

run: $(BUILD)/test_codec
	$(BUILD)/test_codec

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
