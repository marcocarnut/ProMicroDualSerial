/*
 * LUFA library compile-time configuration for ProMicroDualSerial.
 */
#ifndef _LUFA_CONFIG_H_
#define _LUFA_CONFIG_H_

#if (ARCH != ARCH_AVR8)
	#error Unsupported architecture for this LUFA configuration file.
#endif

/* General USB driver tokens */
#define USE_STATIC_OPTIONS               (USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)
#define USB_DEVICE_ONLY
#define NO_SOF_EVENTS                    /* keep USB_GEN_vect quiet: it can delay the soft UART ISRs */

/* Device mode tokens. Control requests are deliberately processed from the
 * main loop (no INTERRUPT_CONTROL_ENDPOINT) so that USB never runs long
 * interrupt handlers that would disturb the soft UART timing. */
#define USE_FLASH_DESCRIPTORS
#define FIXED_CONTROL_ENDPOINT_SIZE      64
#define FIXED_NUM_CONFIGURATIONS         1
#define NO_DEVICE_REMOTE_WAKEUP
#define NO_DEVICE_SELF_POWER

#endif
