/*
 * USB descriptors: a composite device with two CDC-ACM functions (each one
 * an IAD + control interface + data interface), enumerating on Linux as
 * /dev/ttyACM0 (port 0, hardware USART) and /dev/ttyACM1 (port 1, soft UART).
 */
#ifndef DESCRIPTORS_H
#define DESCRIPTORS_H

#include <avr/pgmspace.h>
#include <LUFA/Drivers/USB/USB.h>

/* Endpoint addresses. EP0 is control; the ATmega32U4 has EP1..EP6. */
#define CDC0_NOTIFICATION_EPADDR   (ENDPOINT_DIR_IN  | 1)
#define CDC0_TX_EPADDR             (ENDPOINT_DIR_IN  | 2)
#define CDC0_RX_EPADDR             (ENDPOINT_DIR_OUT | 3)
#define CDC1_NOTIFICATION_EPADDR   (ENDPOINT_DIR_IN  | 4)
#define CDC1_TX_EPADDR             (ENDPOINT_DIR_IN  | 5)
#define CDC1_RX_EPADDR             (ENDPOINT_DIR_OUT | 6)

/* A SERIAL_STATE notification is 10 bytes, so 16 sends it in one packet. */
#define CDC_NOTIFICATION_EPSIZE    16
#define CDC_NOTIFICATION_INTERVAL  2       /* ms */
#define CDC_TXRX_EPSIZE            64
#define CDC_TXRX_BANKS             2       /* double buffered data endpoints */

typedef struct
{
	USB_Descriptor_Configuration_Header_t  Config;

	USB_Descriptor_Interface_Association_t CDC0_IAD;
	USB_Descriptor_Interface_t             CDC0_CCI_Interface;
	USB_CDC_Descriptor_FunctionalHeader_t  CDC0_Functional_Header;
	USB_CDC_Descriptor_FunctionalACM_t     CDC0_Functional_ACM;
	USB_CDC_Descriptor_FunctionalUnion_t   CDC0_Functional_Union;
	USB_Descriptor_Endpoint_t              CDC0_ManagementEndpoint;
	USB_Descriptor_Interface_t             CDC0_DCI_Interface;
	USB_Descriptor_Endpoint_t              CDC0_DataOutEndpoint;
	USB_Descriptor_Endpoint_t              CDC0_DataInEndpoint;

	USB_Descriptor_Interface_Association_t CDC1_IAD;
	USB_Descriptor_Interface_t             CDC1_CCI_Interface;
	USB_CDC_Descriptor_FunctionalHeader_t  CDC1_Functional_Header;
	USB_CDC_Descriptor_FunctionalACM_t     CDC1_Functional_ACM;
	USB_CDC_Descriptor_FunctionalUnion_t   CDC1_Functional_Union;
	USB_Descriptor_Endpoint_t              CDC1_ManagementEndpoint;
	USB_Descriptor_Interface_t             CDC1_DCI_Interface;
	USB_Descriptor_Endpoint_t              CDC1_DataOutEndpoint;
	USB_Descriptor_Endpoint_t              CDC1_DataInEndpoint;
} USB_Descriptor_Configuration_t;

enum InterfaceDescriptors_t
{
	INTERFACE_ID_CDC0_CCI = 0,
	INTERFACE_ID_CDC0_DCI = 1,
	INTERFACE_ID_CDC1_CCI = 2,
	INTERFACE_ID_CDC1_DCI = 3,
};

enum StringDescriptors_t
{
	STRING_ID_Language     = 0,
	STRING_ID_Manufacturer = 1,
	STRING_ID_Product      = 2,
	STRING_ID_CDC0         = 3,
	STRING_ID_CDC1         = 4,
};

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif
