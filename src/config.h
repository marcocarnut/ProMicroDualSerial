/*
 * Build-time configuration for ProMicroDualSerial.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Board -------------------------------------------------------------
 * 0: SparkFun Pro Micro (and clones). 1: "Beetle" (DFRobot style, USB-A
 * plug), which breaks out fewer pins: port 1 moves to A0 (RX) / D10 (TX),
 * and there are no modem lines. Selected with make BOARD_BEETLE=1 (after a
 * make clean). */
#ifndef BOARD_BEETLE
#define BOARD_BEETLE               0
#endif

/* ---- USB identity ------------------------------------------------------
 * 03EB:204E is the VID/PID LUFA uses for its dual CDC demo. Fine for
 * personal use; get your own (e.g. from pid.codes) before distributing. */
#define USB_VID                  0x03EB
#define USB_PID                  0x204E
#define USB_MANUFACTURER_STRING  L"ProMicroDualSerial"
#define USB_PRODUCT_STRING       L"Pro Micro Dual Serial"
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
 * Rates outside this range are clamped. Up to 115200 it runs full duplex at
 * the full line rate. Above that a bit is shorter than the worst case
 * interrupt latency: TX alone still works, but RX loses edges (measured at
 * 250000), so faster rates are not offered. */
#ifndef SOFTUART_MIN_BAUD
#define SOFTUART_MIN_BAUD          50
#endif
#ifndef SOFTUART_MAX_BAUD
#define SOFTUART_MAX_BAUD          115200
#endif

/* ---- Flow control and modem control lines ------------------------------
 * Pins are given as <port letter>,<bit>. All of these signals are active
 * low on the pins, like on any TTL-level UART adapter (e.g. FTDI). Inputs
 * have the internal pull-up enabled, so an unconnected input reads as
 * deasserted.
 *
 * Pro Micro:
 *
 *            Port 0 (ttyACM0)     Port 1 (ttyACM1)
 *   RTS out  F,4  (A3)            F,6  (A1)
 *   CTS in   F,5  (A2)            F,7  (A0)
 *   DTR out  B,1  (15)            D,7  (6)
 *   DSR in   B,3  (14)            B,5  (9)
 *   DCD in   B,2  (16)            E,6  (7)
 *   RI  in   B,6  (10)            B,4  (8)
 *
 * Beetle (TX/RX: port 0 on TX/RX, port 1 on D10/A0):
 *
 *            Port 0 (ttyACM0)     Port 1 (ttyACM1)
 *   RTS out  D,0  (SCL)           F,6  (A1)
 *   CTS in   D,1  (SDA)           F,5  (A2)
 *
 * It has no modem lines (MODEM_LINES is 0); their pins below only keep the
 * code compiling. D9, D11, MI, MO and SCK are left free.
 */
#if BOARD_BEETLE
#define P0_RTS_PIN   D,0
#define P0_CTS_PIN   D,1
#define P1_RTS_PIN   F,6
#define P1_CTS_PIN   F,5
#define P0_DTR_PIN   B,5
#define P0_DSR_PIN   B,7
#define P0_DCD_PIN   B,7
#define P0_RI_PIN    B,7
#define P1_DTR_PIN   B,5
#define P1_DSR_PIN   B,7
#define P1_DCD_PIN   B,7
#define P1_RI_PIN    B,7
#ifndef MODEM_LINES
#define MODEM_LINES  0
#endif
#else
#define P0_RTS_PIN   F,4
#define P0_CTS_PIN   F,5
#define P0_DTR_PIN   B,1
#define P0_DSR_PIN   B,3
#define P0_DCD_PIN   B,2
#define P0_RI_PIN    B,6
#define P1_RTS_PIN   F,6
#define P1_CTS_PIN   F,7
#define P1_DTR_PIN   D,7
#define P1_DSR_PIN   B,5
#define P1_DCD_PIN   E,6
#define P1_RI_PIN    B,4
#endif

/* Hardware (RTS/CTS) flow control, per port. CDC-ACM has no request to
 * switch it at run time (the host's CRTSCTS setting never reaches the
 * device), so it is chosen here:
 *
 *  FLOW_OFF  CTS is ignored; RTS follows the host's RTS.
 *  FLOW_ON   The transmitter only starts a frame while CTS is asserted
 *            (low). An unconnected CTS reads as deasserted (pull-up), so
 *            nothing is sent until CTS is pulled low.
 *  FLOW_AUTO Like FLOW_ON, but CTS is only obeyed once it has been seen
 *            asserted since the host opened the port (raised DTR). An
 *            unconnected CTS never arms it, a real peer arms it at once.
 *            Caveat: a peer that starts out with CTS deasserted is sent to
 *            until it first asserts CTS.
 *
 * With FLOW_ON / FLOW_AUTO, RTS is asserted while the host has RTS set
 * (Linux does while the port is open) and the receive buffer has room. */
#define FLOW_OFF     0
#define FLOW_ON      1
#define FLOW_AUTO    2

#ifndef P0_HW_FLOW
#define P0_HW_FLOW   FLOW_AUTO
#endif
#ifndef P1_HW_FLOW
#define P1_HW_FLOW   FLOW_AUTO
#endif

/* DTR output following the host, and DSR / DCD / RI inputs reported to the
 * host (CDC SERIAL_STATE: TIOCM_DSR / TIOCM_CD / TIOCM_RI and the
 * TIOCMIWAIT / TIOCGICOUNT counters on Linux). Note that Linux hangs up a
 * port whose DCD drops unless CLOCAL is set (it is by default). */
#ifndef MODEM_LINES
#define MODEM_LINES  1
#endif

/* RX buffer level (bytes free) at which RTS is dropped / raised again */
#define RTS_OFF_FREE 32
#define RTS_ON_FREE  64

/* Debug: sampling profiler on Timer1 compare B (see profiler.h, tools/fwprofile.py) */
#ifndef PROFILER
#define PROFILER     0
#endif

#endif
