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

**Flow control** (`P0_HW_FLOW`, `P1_HW_FLOW` in `src/config.h`). CDC-ACM
has no way for the host to switch it at run time: minicom's "hardware flow
control" setting never reaches the device. So it's chosen at build time:

* `FLOW_AUTO` (default): CTS is obeyed once it has been seen asserted (low)
  since the host opened the port (raised DTR). An unconnected CTS never arms
  it, and a real peer arms it straight away. The catch: a peer that starts
  out with CTS deasserted is sent to until it first asserts CTS.
* `FLOW_ON`: the port only transmits while CTS is low, so an unconnected CTS
  blocks it.
* `FLOW_OFF`: CTS is ignored.

With `FLOW_AUTO` or `FLOW_ON`, RTS is asserted while the host has RTS set
(Linux does that while the port is open) and the receive buffer has room.
With `FLOW_OFF`, RTS just follows the host. CTS is checked before each
frame, so at most the frame already committed (plus one in the USART on
port 0) goes out after CTS drops. Breaks still go through while CTS holds
off data.

The RX and TX LEDs blink on UART activity.

## Testing

* `make -C test` runs host unit tests of the soft UART framing code (all
  line codings, baud mismatch, jitter, errors, break, glitches).
* `make -C test/sim SIMAVR_INC=... SIMAVR_LIB=...` runs the real drivers
  (asm ISRs included) in a simulated ATmega32U4: USART1 is bridged to the
  soft UART, whose TX is looped back to its RX (or, in the `rxgen` cases,
  RX is driven by an independent full-rate stream). It checks every byte,
  the error flags and the timing of every TX edge. Needs simavr >= 1.8 built
  from source (recommended: the `fixes` branch of github.com/marcocarnut/simavr,
  see `test/sim/Makefile`). `test/sim/profile.py` turns a
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
* **Codec:** the encoder (`softuart_enc.S`, ~230 cycles per random 8N1
  frame) and the per-edge decode loop (`softuart_dec.S`) are asm too. Both
  are checked against the C reference versions by running them side by side
  on the simulated AVR: the encoder on every byte in every line coding, the
  decoder on random edge streams.
* **Interrupt latency:** both ISRs are hand-written asm (53 and 68 cycles
  including interrupt entry). The USART ISRs re-enable interrupts after 20
  cycles so they can't hold off the timer ISRs. At 115200 a bit is 139 CPU
  cycles, and the worst-case latency chain is 125 cycles.
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
| 115200, RX at 100% (any framing) + TX offered    | RX lossless, TX ~58% of line rate|
| 115200, full duplex loopback, random data        | TX/RX ~68% of line rate          |
| 115200, full duplex loopback, 0x55 (worst case)  | ~50%                             |
| 230400, RX continuous                            | beyond the CPU: bursts only      |

Measured on a Pro Micro with port 0 looped to port 1 (both directions at
once, random data, host-side timing):

| 115200                         | hw -> soft (soft RX) | soft -> hw (soft TX) |
|--------------------------------|----------------------|----------------------|
| one direction at a time        | 100%                 | 100%                 |
| full duplex, 8N1 / 8E1 / 7O1   | 100%                 | 56% / 56% / 55%      |
| 57600 full duplex              | 100%                 | 100%                 |

## Status

All four milestones are implemented:

1. Dual CDC-ACM descriptors (two IADs). Linux enumerates ttyACM0/1.
2. Port 0 on USART1: all CDC line codings (5-8 data bits, none/odd/even/
   mark/space parity, 1/1.5/2 stop bits), 245 baud to 2 Mbaud, break both
   ways, framing/parity/overrun/break reported to the host.
3. Port 1 on the timer soft UART: the same line codings, 50 to 230400 baud
   with exact bit timing.
4. RTS/CTS flow control on both ports (auto, on or off), plus DTR out and
   DSR/DCD/RI in.

Also included: a 1200-baud touch to enter the Caterina bootloader
(`BOOTLOADER_1200BAUD_TOUCH`) and activity LEDs.
