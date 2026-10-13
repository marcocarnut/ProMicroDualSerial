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

| Signal        | Port 0 (ttyACM0, USART1) | Port 1 (ttyACM1, soft UART) |
|---------------|--------------------------|-----------------------------|
| TX            | TX0 (PD3)                | 9 (PB5, OC1A)               |
| RX            | RX1 (PD2)                | 4 (PD4, ICP1)               |

The RX and TX LEDs blink on UART activity.

## Testing

* `make -C test` runs host unit tests of the soft UART framing code.
* `make -C test/sim SIMAVR_INC=<simavr>/simavr/sim` runs the soft UART
  drivers (asm ISRs included) in simavr with TX looped back to RX. simavr
  1.6's Timer1 output compare model has bugs in normal mode, so this is
  a work in progress; see the comments in `test/sim/harness.c`.
* On hardware, wire TX0 -> pin 4 and pin 9 -> RX1 and run
  `tools/loopback_test.py`. It sends data both ways through both UARTs at
  once, for many line codings, and checks error counters and breaks.

## Soft UART design

Timer1 runs free with a per-baud prescaler. TX: the main loop turns each
byte into a list of edges (delays + levels). The compare ISR only does
`OCR1A += delta` and sets COM1A, and the hardware toggles the pin exactly on
time. RX: the capture ISR stores the 16-bit timestamp and flips ICES1. The
main loop rebuilds the bits by sampling mid-bit between edges. Both ISRs are
hand-written asm (55 and 63 cycles). The USART ISRs re-enable interrupts
after about 20 cycles so they can't hold off the timer ISRs. At 115200
baud a bit is 139 CPU cycles, and the worst-case latency chain is about
118 cycles.

CPU budget: each line transition costs one ISR. Random data averages 5.5
edges per frame. At 115200 full duplex on the soft port, the timer ISRs use
roughly half the CPU. A continuous 0x55 pattern in both directions at once
is close to the limit.

## Status

Milestone 3: port 1 is the timer soft UART (5-8 data bits, none/odd/even/
mark/space parity, 1/1.5/2 stop bits, break, framing/parity/overrun
reporting, 50..230400 baud).
