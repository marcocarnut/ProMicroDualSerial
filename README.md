# ProMicroDualUART

Dual USB CDC-ACM <-> UART bridge for the ATmega32U4 (SparkFun Pro Micro and
clones), built on [LUFA](https://github.com/abcminiuser/lufa).

* `/dev/ttyACM0` - port 0, hardware USART1
* `/dev/ttyACM1` - port 1, timer-based soft UART (Timer1 input capture / output compare)

## Building

    git submodule update --init
    make
    make flash          # 1200-baud touch + avrdude through the Caterina bootloader

Requires `avr-gcc`, `avr-libc` and `avrdude`.

## Wiring

| Signal       | Port 0 (ttyACM0, USART1) | Port 1 (ttyACM1, soft UART) | Direction |
|--------------|--------------------------|-----------------------------|-----------|
| TX           | TX0 (PD3)                | 9 (PB5, OC1A)               | out       |
| RX           | RX1 (PD2)                | 4 (PD4, ICP1)               | in        |
| RTS          | A3 (PF4)                 | A1 (PF6)                    | out       |
| CTS          | A2 (PF5)                 | A0 (PF7)                    | in        |
| DTR          | 15 (PB1)                 | 6 (PD7)                     | out       |
| DSR          | 14 (PB3)                 | 5 (PC6)                     | in        |
| DCD          | 16 (PB2)                 | 7 (PE6)                     | in        |
| RI           | 10 (PB6)                 | 8 (PB4)                     | in        |

All levels are 5 V TTL. The handshake lines are active low, like on an FTDI
adapter. Inputs have pull-ups, so unconnected inputs read as deasserted. Pins
and options are in `src/config.h`.

**Flow control is on by default (`P0_HW_FLOW`, `P1_HW_FLOW`).** A port only
transmits while its CTS is low, so tie CTS to GND if you don't use it.
RTS is asserted while the host has the port open (host RTS set) and the
receive buffer has room. With flow control off, RTS just follows the host.
Breaks can be sent even while CTS holds off data.

The RX and TX LEDs blink on UART activity.

## Testing

* `make -C test` runs host unit tests of the soft UART framing code (all
  line codings, baud mismatch, jitter, errors, break, glitches).
* `make -C test/sim SIMAVR_INC=... SIMAVR_LIB=...` runs the real drivers
  (asm ISRs included) in a simulated ATmega32U4: USART1 is bridged to the
  soft UART, whose TX is looped back to its RX (or, in the `rxgen` cases,
  RX is driven by an independent full-rate stream). It checks every byte,
  the error flags and the timing of every TX edge. Needs simavr >= 1.8 built
  from source; see `test/sim/Makefile`. `test/sim/profile.py` turns a
  `PROFILE=` dump into a per-function cycle profile.
* On hardware, wire TX0 -> pin 4 and pin 9 -> RX1 and run
  `tools/loopback_test.py`: data both ways through both UARTs at once for
  many line codings, plus error counters and breaks.

## Soft UART design

Timer1 runs free with a per-baud prescaler.

* **TX:** the main loop turns each byte into a list of edges (delays and
  levels). The compare ISR only does `OCR1A += delta` and sets COM1A, and
  the hardware switches the pin exactly on time.
* **RX:** the capture ISR stores the 16-bit timestamp and flips ICES1. The
  main loop rebuilds the bits by sampling mid-bit between edges.
* **Interrupt latency:** both ISRs are hand-written asm (55 and 63 cycles).
  The USART ISRs re-enable interrupts after about 20 cycles so they can't
  hold off the timer ISRs. At 115200 a bit is 139 CPU cycles, and the
  worst-case latency chain is about 118 cycles.
* **Priority:** when the CPU runs short, RX wins. TX encoding is paused while
  received edges pile up, so TX just leaves idle time between frames (legal
  on a UART line, and the host is held off by USB flow control), while RX
  doesn't lose data.

### Throughput (simulated, 16 MHz)

Each line transition costs an interrupt plus main loop work, so the load
depends on the data pattern. In these tests port 0 carries the same traffic
at the same time.

| Soft UART load                                   | Result                           |
|--------------------------------------------------|----------------------------------|
| <= 57600, full duplex, continuous                | 100% of line rate both ways      |
| 115200, RX continuous at 100%                    | lossless                         |
| 115200, RX at 100% + TX continuous offered       | RX lossless, TX ~50% of line rate|
| 115200, full duplex loopback, random data        | TX/RX ~55% of line rate          |
| 115200, full duplex loopback, 0x55 (worst case)  | ~41%                             |
| 230400, RX continuous                            | beyond the CPU: bursts only      |

## Status

All four milestones are implemented:

1. Dual CDC-ACM descriptors (two IADs). Linux enumerates ttyACM0/1.
2. Port 0 on USART1: all CDC line codings (5-8 data bits, none/odd/even/
   mark/space parity, 1/1.5/2 stop bits), 245 baud to 2 Mbaud, break both
   ways, framing/parity/overrun/break reported to the host.
3. Port 1 on the timer soft UART: the same line codings, 50 to 230400 baud
   with exact bit timing.
4. RTS/CTS flow control on both ports, plus DTR out and DSR/DCD/RI in.

Also included: a 1200-baud touch to enter the Caterina bootloader
(`BOOTLOADER_1200BAUD_TOUCH`) and activity LEDs.
