/*
 * Build-time configuration for ProMicroDualUART.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- USB identity ------------------------------------------------------
 * 03EB:204E is the VID/PID LUFA uses for its dual CDC demo. Fine for
 * personal use; get your own (e.g. from pid.codes) before distributing. */
#define USB_VID                  0x03EB
#define USB_PID                  0x204E
#define USB_MANUFACTURER_STRING  L"ProMicroDualUART"
#define USB_PRODUCT_STRING       L"Pro Micro Dual UART"
#define USB_CDC0_STRING          L"UART0 (hardware USART)"
#define USB_CDC1_STRING          L"UART1 (timer soft UART)"

/* ---- Bootloader --------------------------------------------------------
 * Arduino-style "1200 baud touch": when port 0 is set to 1200 baud and DTR
 * is dropped (i.e. the port is opened at 1200 baud and closed), jump to the
 * Caterina bootloader so the board can be reflashed without touching RST.
 * Side effect: port 0 can still be used at 1200 baud, but closing it at
 * that speed reboots the board. Set to 0 to disable. */
#define BOOTLOADER_1200BAUD_TOUCH  1

/* ---- Soft UART (port 1) ----------------------------------------------
 * Line codings outside this range are clamped. 115200 is the design target;
 * at higher rates the margin for interrupt latency (one bit time) gets thin. */
#define SOFTUART_MIN_BAUD          50
#define SOFTUART_MAX_BAUD          230400

#endif
