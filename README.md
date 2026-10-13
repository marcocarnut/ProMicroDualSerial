# ProMicroDualUART

Dual USB CDC-ACM <-> UART bridge for the ATmega32U4 (SparkFun Pro Micro and
clones), built on [LUFA](https://github.com/abcminiuser/lufa). Both ports run
115200 baud full duplex at the full line rate, at the same time.

* `/dev/ttyACM0` - port 0, hardware USART1
* `/dev/ttyACM1` - port 1, timer-based soft UART (Timer1 input capture for
  RX, Timer3 output compare for TX)

## Building

    git submodule update --init
    make
    make flash          # 1200-baud touch + avrdude through the Caterina bootloader

Requires `avr-gcc`, `avr-libc` and `avrdude`.

Registers r2-r7 belong to the soft UART's interrupt handlers: everything is
compiled with `-ffixed-r2` ... `-ffixed-r7` (see the `Makefile`). Code built
outside it, or hand-written asm, must not touch them either; the prebuilt
libgcc / avr-libc routines the firmware links don't.

## Wiring

| Signal       | Port 0 (ttyACM0, USART1) | Port 1 (ttyACM1, soft UART) | Direction |
|--------------|--------------------------|-----------------------------|-----------|
| TX           | TX0 (PD3)                | 5 (PC6, OC3A)               | out       |
| RX           | RX1 (PD2)                | 4 (PD4, ICP1)               | in        |
| RTS          | A3 (PF4)                 | A1 (PF6)                    | out       |
| CTS          | A2 (PF5)                 | A0 (PF7)                    | in        |
| DTR          | 15 (PB1)                 | 6 (PD7)                     | out       |
| DSR          | 14 (PB3)                 | 9 (PB5)                     | in        |
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
  the error flags and the timing of every TX edge, and runs the asm decoder
  against the C one on random edge streams. Needs simavr >= 1.8 built from
  source (recommended: the `fixes` branch of github.com/marcocarnut/simavr,
  see `test/sim/Makefile`). `test/sim/profile.py` turns a `PROFILE=` dump
  into a per-function cycle profile.
* On hardware, wire the two ports to each other like a null modem cable
  (TX0 -> pin 4, pin 5 -> RX1, A3 -> A0, A1 -> A2; for `--modem` also
  15 -> 9 and 6 -> 14) and run `tools/loopback_test.py`: data both ways
  through both UARTs at once for many line codings, error counters, breaks,
  flow control and the modem lines.
* Profiling on the board: `make clean && make EXTRA=-DPROFILER=1`, flash,
  then `tools/fwprofile.py --mode both` runs full duplex traffic over the
  loopback rig and prints where the CPU spent its time (Timer1 compare B
  samples the PC; the share of samples held off by the non-nestable timer
  ISRs is reported separately).

## Soft UART design

All bit timing is done by timer hardware, so it doesn't depend on interrupt
latency; the interrupt handlers only have to keep up.

* **TX:** Timer3 runs in CTC mode with a period of one bit. The main loop
  turns each byte into a 16-bit frame word (start, data, parity and stop
  bits) and queues it; the compare ISR shifts the next bit of the word into
  OC3A's set/clear mode, so the pin switches on the next match, exactly on
  the bit grid. It keeps its state in the reserved registers and pushes
  nothing: 21 cycles per bit including interrupt entry, ~250 per 8N1 frame
  with fetching the next word. Bit times are whole timer ticks (0.08% slow
  at 115200), and 1.5 stop bits are sent as 2.
* **RX:** Timer1 runs free with a per-baud prescaler. The capture ISR (45
  cycles) stores the 16-bit timestamp of each edge and flips ICES1. The main
  loop rebuilds the bits by sampling mid-bit between edges. The decoder
  (`softuart_dec.S`) is a state machine whose state (idle or in a frame,
  line high or low) is the code position, so an edge that completes no
  sample costs ~18 cycles; it is checked against the C reference version
  on the simulated AVR.
* **Interrupt latency:** the USART ISRs re-enable interrupts after ~17
  cycles so they can't hold off the timer ISRs, and handle the common case
  (a clean byte in, the next byte out) in asm with three registers saved
  (~70 cycles); errors and emulated mark/space parity go to C. At 115200 a
  bit is 139 CPU cycles, and the worst case from a timer event to its ISR
  having done the time-critical part is under 90 cycles.
* **Priority:** when the CPU runs short, RX wins. TX stops queueing frames
  while received edges pile up, so TX just leaves idle time between frames
  (legal on a UART line, and the host is held off by USB flow control),
  while RX doesn't lose data.

### Throughput

Measured on a Pro Micro with port 0 looped to port 1, random data, both
directions at once and sustained (the soft TX figure is taken while soft RX
runs at 100% throughout), host-side timing:

| 115200                         | hw -> soft (soft RX) | soft -> hw (soft TX) |
|--------------------------------|----------------------|----------------------|
| one direction at a time        | 100%                 | 100%                 |
| full duplex, 8N1 / 8E1 / 7O1   | 100%                 | 100% / 100% / 100%   |

In the simulator (port 0 carries the same traffic at the same time):

| Soft UART load                                   | Result                               |
|--------------------------------------------------|--------------------------------------|
| <= 57600, full duplex, continuous                | 100% of line rate both ways          |
| 115200, RX at 100% (any framing) + TX offered    | RX lossless, TX ~100%                |
| 115200, full duplex loopback, random data        | ~100%                                |
| 115200, full duplex loopback, 0x55 (worst case)  | ~79% (every bit boundary is an edge) |
| 230400, RX continuous                            | beyond the CPU: bursts only          |

## Status

All four milestones are implemented:

1. Dual CDC-ACM descriptors (two IADs). Linux enumerates ttyACM0/1.
2. Port 0 on USART1: all CDC line codings (5-8 data bits, none/odd/even/
   mark/space parity, 1/1.5/2 stop bits), 245 baud to 2 Mbaud, break both
   ways, framing/parity/overrun/break reported to the host.
3. Port 1 on the timer soft UART: the same line codings, 50 to 230400 baud.
4. RTS/CTS flow control on both ports (auto, on or off), plus DTR out and
   DSR/DCD/RI in.

Also included: a 1200-baud touch to enter the Caterina bootloader
(`BOOTLOADER_1200BAUD_TOUCH`) and activity LEDs.
