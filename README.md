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

## Status

Milestone 2: port 0 bridges to the hardware USART (all CDC line codings incl.
mark/space parity, break, error reporting); port 1 is a loopback placeholder.
