/*
 * Pin access by "<port letter>,<bit>" pairs as used in config.h, e.g.
 * GPIO_SET(P0_RTS_PIN) -> PORTF |= _BV(4). The extra macro level makes the
 * pair expand into two arguments.
 */
#ifndef GPIO_H
#define GPIO_H

#include <avr/io.h>

#define GPIO_PORT_(p, b)   PORT##p
#define GPIO_PIN_(p, b)    PIN##p
#define GPIO_DDR_(p, b)    DDR##p
#define GPIO_BIT_(p, b)    (b)

/* variadic so that a pin macro may be passed through several levels */
#define GPIO_PORT(...)     GPIO_PORT_(__VA_ARGS__)
#define GPIO_PINREG(...)   GPIO_PIN_(__VA_ARGS__)
#define GPIO_DDR(...)      GPIO_DDR_(__VA_ARGS__)
#define GPIO_BIT(...)      GPIO_BIT_(__VA_ARGS__)

#define GPIO_OUTPUT(...)   (GPIO_DDR(__VA_ARGS__) |= _BV(GPIO_BIT(__VA_ARGS__)))
#define GPIO_INPUT_PU(...) do { GPIO_DDR(__VA_ARGS__) &= ~_BV(GPIO_BIT(__VA_ARGS__)); \
                                GPIO_PORT(__VA_ARGS__) |= _BV(GPIO_BIT(__VA_ARGS__)); } while (0)
#define GPIO_SET(...)      (GPIO_PORT(__VA_ARGS__) |= _BV(GPIO_BIT(__VA_ARGS__)))
#define GPIO_CLR(...)      (GPIO_PORT(__VA_ARGS__) &= ~_BV(GPIO_BIT(__VA_ARGS__)))
#define GPIO_READ(...)     ((GPIO_PINREG(__VA_ARGS__) >> GPIO_BIT(__VA_ARGS__)) & 1)

/* active low signals: drive (pin, asserted?) / read */
#define GPIO_ASSERT(pin, on) do { if (on) GPIO_CLR(pin); else GPIO_SET(pin); } while (0)
#define GPIO_ASSERTED(...)   (!GPIO_READ(__VA_ARGS__))

#endif
