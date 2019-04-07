/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define _USB_DEVICE_DESCRIPTOR_C_
#include <string.h>
#include "usb_device_config.h"
#include "../usb/include/usb.h"

#include "../usb/device/usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"

#include "usb_device_descriptor.h"

/* >>>
 Because we allow different composition dynamically,
 configuration desc is generated dynamically by linking parts ;
 interface definition related variables are also fixed, including
	 g_xxxEndpoints[#].byte0: endpoint ndx
	 g_xxxInterfaces[#].byte4: interface ndx
  <<< */
/*******************************************************************************
* Variables
******************************************************************************/
USBDCfgFix_t g_cfgFix;

/* msc disk information */
/* Define endpoint for MSC class */
usb_device_endpoint_struct_t g_mscDiskEndpoints[USB_MSC_DISK_ENDPOINT_COUNT] = {
	{
		USB_MSC_DISK_BULK_IN_ENDPOINT | (USB_IN << 7U), // fixed during desc linkage
		USB_ENDPOINT_BULK, FS_MSC_DISK_BULK_IN_PACKET_SIZE,
	},
	{
		USB_MSC_DISK_BULK_OUT_ENDPOINT | (USB_OUT << 7U), // fixed during desc linkage
		USB_ENDPOINT_BULK, FS_MSC_DISK_BULK_OUT_PACKET_SIZE,
	}
};

/* Define interface for MSC class */
usb_device_interface_struct_t g_mscDiskInterface[] = {
	{
		0,
		{
			USB_MSC_DISK_ENDPOINT_COUNT, g_mscDiskEndpoints,
		},
		NULL
	}
};

/* Define interfaces for MSC disk */
usb_device_interfaces_struct_t g_mscDiskInterfaces[1] = {
	{
		USB_MSC_DISK_CLASS, USB_MSC_DISK_SUBCLASS, USB_MSC_DISK_PROTOCOL, 
		USB_MSC_DISK_INTERFACE_INDEX, // fixed during desc linkage
		g_mscDiskInterface,
		sizeof(g_mscDiskInterface) / sizeof(usb_device_interfaces_struct_t),
	}
};

/* Define configurations for MSC disk */
usb_device_interface_list_t g_mscDiskInterfaceList[USB_DEVICE_CONFIGURATION_COUNT] = {
	{
		1,
		g_mscDiskInterfaces,
	},
};

/* Define class information for MSC disk */
usb_device_class_struct_t g_usbdMscConfig = {
	g_mscDiskInterfaceList, kUSB_DeviceClassTypeMsc, USB_DEVICE_CONFIGURATION_COUNT,
};

/* cdc virtual com information */
/* Define endpoint for communication class */
usb_device_endpoint_struct_t g_cdcVcomCicEndpoints[USB_CDC_VCOM_CIC_ENDPOINT_COUNT] = {
	{
		USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT | (USB_IN << 7U), // fixed during desc linkage
		USB_ENDPOINT_INTERRUPT,
		FS_CDC_VCOM_BULK_IN_PACKET_SIZE,
	},
};

/* Define endpoint for data class */
usb_device_endpoint_struct_t g_cdcVcomDicEndpoints[USB_CDC_VCOM_DIC_ENDPOINT_COUNT] = {
	{
		USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT | (USB_IN << 7U), // fixed during desc linkage
		USB_ENDPOINT_BULK, FS_CDC_VCOM_BULK_IN_PACKET_SIZE,
	},
	{
		USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT | (USB_OUT << 7U), // fixed during desc linkage
		USB_ENDPOINT_BULK, FS_CDC_VCOM_BULK_OUT_PACKET_SIZE,
	},
};

/* Define interface for communication class */
usb_device_interface_struct_t g_cdcVcomCicInterface[] = {
	{
		0,
		{
			USB_CDC_VCOM_CIC_ENDPOINT_COUNT, g_cdcVcomCicEndpoints,
		},
		NULL
	}
};

/* Define interface for data class */
usb_device_interface_struct_t g_cdcVcomDicInterface[] = {
	{
		0,
		{
			USB_CDC_VCOM_DIC_ENDPOINT_COUNT, g_cdcVcomDicEndpoints,
		},
		NULL
	}
};

/* Define interfaces for virtual com */
usb_device_interfaces_struct_t g_cdcVcomInterfaces[2] = {
	{
		USB_CDC_VCOM_CIC_CLASS,
		USB_CDC_VCOM_CIC_SUBCLASS,
		USB_CDC_VCOM_CIC_PROTOCOL,
		USB_CDC_VCOM_CIC_INTERFACE_INDEX, // fixed during desc linkage
		g_cdcVcomCicInterface,
		sizeof(g_cdcVcomCicInterface) / sizeof(usb_device_interfaces_struct_t)
	},

	{
		USB_CDC_VCOM_DIC_CLASS,
		USB_CDC_VCOM_DIC_SUBCLASS,
		USB_CDC_VCOM_DIC_PROTOCOL,
		USB_CDC_VCOM_DIC_INTERFACE_INDEX, // fixed during desc linkage
		g_cdcVcomDicInterface,
		sizeof(g_cdcVcomDicInterface) / sizeof(usb_device_interfaces_struct_t)
	},
};

/* Define configurations for virtual com */
usb_device_interface_list_t g_UsbDeviceCdcVcomInterfaceList[USB_DEVICE_CONFIGURATION_COUNT] = {
	{2, g_cdcVcomInterfaces,},
};

/* Define class information for virtual com */
usb_device_class_struct_t g_usbdCdcVcomConfig = {
	g_UsbDeviceCdcVcomInterfaceList,
	kUSB_DeviceClassTypeCdc,
	USB_DEVICE_CONFIGURATION_COUNT,
};

/* Define device descriptor */
uint8_t g_UsbDeviceDescriptor[USB_DESCRIPTOR_LENGTH_DEVICE] = {
	0x12, // USB_DESCRIPTOR_LENGTH_DEVICE,
	0x01, // USB_DESCRIPTOR_TYPE_DEVICE,
	/* USB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H). */
	USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION), USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION),
	0xEf, // USB_DEVICE_CLASS, set to 0xEF for composite device
	USB_DEVICE_SUBCLASS,
	USB_DEVICE_PROTOCOL,
	USB_CONTROL_MAX_PACKET_SIZE,
	/* default vendor ID (assigned by the USB-IF) */
	0xC9U, 0x1FU, // vendor ID can be conigured by user
	/* default Product ID (assigned by the manufacturer) */
	0x9E, 0x00,   // product ID can be conigured by user
	/* Device release number in binary-coded decimal */
	USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION), USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION),

	0x01, /* Index of string descriptor describing manufacturer */
	0x02, /* Index of string descriptor describing product */
	0x03, /* Index of string descriptor describing the device's serial number */
	USB_DEVICE_CONFIGURATION_COUNT,
};

uint8_t g_UsbDevCfgDesc[256]; // generated cfg desc according to user selected itf
// common header for a composite device
uint8_t s_UsbDevCfgDescHdr[9] =
{
	0x09U, // USB_DESCRIPTOR_LENGTH_CONFIGURE,
	0x02U, // USB_DESCRIPTOR_TYPE_CONFIGURE,
	0, // LSB of total len of cur cfg, fixed during desc linkage
	0, // MSB of total len of cur cfg, fixed during desc linkage
	0, // ItfCnt, fixed during desc linkage,
	/* Value to use as an argument to the SetConfiguration() request to select this configuration */
	1, // Value to use as an argument to the SetConfiguration() request to select this configuration
	2, /* Index of string descriptor describing this configuration */
	/* Configuration characteristics D7: Reserved (set to one) D6: Self-powered D5: Remote Wakeup D4...0: Reserved
		(reset to zero) */
	(USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
		(USB_DEVICE_CONFIG_SELF_POWER << 6) |
		(USB_DEVICE_CONFIG_REMOTE_WAKEUP << 5),
	/* Maximum power consumption of the USB * device from the bus in this specific * configuration when the device is
		fully * operational. Expressed in 2 mA units *  (i.e., 50 = 100 mA).  */
	USB_DEVICE_MAX_POWER,
};
uint8_t s_UsbDevCfgDescMscPart[] =
{
	/* MSC Interface Descriptor */
	0x09, // 0 USB_DESCRIPTOR_LENGTH_INTERFACE,
	0x04, // 1 USB_DESCRIPTOR_TYPE_INTERFACE,
	0x00, // 2 USB_MSC_DISK_INTERFACE_INDEX, // fixed during desc linkage
	0x00, // 3 Alternate setting
	USB_MSC_DISK_ENDPOINT_COUNT, USB_MSC_DISK_CLASS, USB_MSC_DISK_SUBCLASS, USB_MSC_DISK_PROTOCOL,
	5, /*ofs=8 Interface Description String Index*/ // fixed during desc linkage str5=iFlash, str6=SPI Flash, str7=SD Card

	/*Bulk IN Endpoint descriptor, ofs = 9 */
	USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
	0 | (USB_IN << 7U), // ofs = 11, fixed during desc linkage
	USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(HS_MSC_DISK_BULK_IN_PACKET_SIZE),
	USB_SHORT_GET_HIGH(HS_MSC_DISK_BULK_IN_PACKET_SIZE),
	0x00, /* The polling interval value is every 0 Frames */

	/*Bulk OUT Endpoint descriptor ofs = 16 */
	USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
	0 | (USB_OUT << 7U),  // ofs = 18 fixed during desc linkage
	USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(HS_MSC_DISK_BULK_OUT_PACKET_SIZE),
	USB_SHORT_GET_HIGH(HS_MSC_DISK_BULK_OUT_PACKET_SIZE),
	0x00 /* The polling interval value is every 0 Frames */
};

uint8_t s_UsbDevCfgDescCdcPart[] = 
{
	/* Interface Association Descriptor */
	8, // USB_IAD_DESC_SIZE,
	0xb, // desc type = IAD type,
	0x00, /* ofs = 2 The first interface number associated with this function */ // fixed during desc linkage
	0x02, /* The number of contiguous interfaces associated with this function */
	USB_CDC_VCOM_CIC_CLASS, /* The function belongs to the Communication Device/Interface Class  */
	0x03,
	0x00, /* The function uses the No class specific protocol required Protocol  */
	0x02, /* The Function string descriptor index */
	/************************************************************/
	/* ofs = 8 Interface Descriptor */
	0x09, // USB_DESCRIPTOR_LENGTH_INTERFACE,
	0x04, // USB_DESCRIPTOR_TYPE_INTERFACE,
	0, // ofs = 10 USB_CDC_VCOM_CIC_INTERFACE_INDEX, fixed during desc linkage
	0x00,
	USB_CDC_VCOM_CIC_ENDPOINT_COUNT,
	USB_CDC_VCOM_CIC_CLASS,
	USB_CDC_VCOM_CIC_SUBCLASS,
	USB_CDC_VCOM_CIC_PROTOCOL,
	3, // stringDescNdx

	/* ofs = 17 CDC Class-Specific descriptor */
	USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC, /* Size of this descriptor in bytes */
	USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE,  /* CS_INTERFACE Descriptor Type */
	USB_CDC_HEADER_FUNC_DESC, 0x10,
	0x01, /* USB Class Definitions for Communications the Communication specification version 1.10 */

	// ofs = 22 Call Management Functional Descriptor
	USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG, /* Size of this descriptor in bytes */
	USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
	USB_CDC_CALL_MANAGEMENT_FUNC_DESC,
	0x01, /*Bit 0: Whether device handle call management itself 1, Bit 1: Whether device can send/receive call
			management information over a Data Class Interface 0 */
	0x01, /* Indicates multiplexed commands are handled via data interface */

	// ofs = 27 ACM Functional Descriptor
	USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT,   /* Size of this descriptor in bytes */
	USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
	USB_CDC_ABSTRACT_CONTROL_FUNC_DESC,
	0x06, /* Bit 0: Whether device supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and
			Get_Comm_Feature 0, Bit 1: Whether device supports the request combination of Set_Line_Coding,
			Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State 1, Bit ...  */

	// ofs = 31 Union Functional Descriptor
	USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC, /* Size of this descriptor in bytes */
	USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
	USB_CDC_UNION_FUNC_DESC, 0x00,        /* The interface number of the Communications or Data Class interface  */
	0x01,                                 /* Interface number of subordinate interface in the Union  */

	/* ofs = 36 Notification Endpoint descriptor */
	USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
	0 | (USB_IN << 7U), // ofs = 38 fixed during desc linkage
	USB_ENDPOINT_INTERRUPT,
	USB_SHORT_GET_LOW(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE),
	FS_CDC_VCOM_INTERRUPT_IN_INTERVAL,

	/* ofs = 43 Data Interface Descriptor */
	0x09, // USB_DESCRIPTOR_LENGTH_INTERFACE,
	0x04, // USB_DESCRIPTOR_TYPE_INTERFACE,
	0, // USB_CDC_VCOM_DIC_INTERFACE_INDEX, // ofs = 45 fixed during desc linkage
	0x00, // alternate setting
	2, // USB_CDC_VCOM_DIC_ENDPOINT_COUNT,
	USB_CDC_VCOM_DIC_CLASS,
	USB_CDC_VCOM_DIC_SUBCLASS,
	USB_CDC_VCOM_DIC_PROTOCOL,
	0x00, /* Interface Description String Index*/

	/* ofs = 52 Bulk IN Endpoint descriptor */
	USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
	0 | (USB_IN << 7U), // ofs = 54 fixed during desc linkage
	USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_IN_PACKET_SIZE),
	USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_IN_PACKET_SIZE),
	0x00, /* The polling interval value is every 0 Frames */

	/* ofs = 59 Bulk OUT Endpoint descriptor */
	USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
	0 | (USB_OUT << 7U), // ofs = 61 fixed during desc linkage
	USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE),
	USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE),
	0x00, /* The polling interval value is every 0 Frames */
};

/* Define string descriptor */
uint8_t g_UsbDeviceString0[4] = {
	0, USB_DESCRIPTOR_TYPE_STRING,
	0x09, 0x04
};

uint8_t g_UsbDeviceString1[] = {
	0,
	USB_DESCRIPTOR_TYPE_STRING,
	'2', 0, '0', 0, '1', 0, '8',0,
	'X', 0, 'M', 0, 'B', 0,
	' ', 0, 'A', 0, 'C', 0, 'E', 0,
};

uint8_t g_UsbDeviceString2[] = {
	0,
	USB_DESCRIPTOR_TYPE_STRING,
	'm', 0, 'p', 0, 'y', 0, ' ', 0,
	'U', 0, 'S', 0, 'B', 0, ' ', 0,
	'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
};

uint8_t g_UsbDeviceString3[] = {
	0,
	USB_DESCRIPTOR_TYPE_STRING,
	'F', 0, 'e', 0, 'e', 0, 'd', 0,
	'F', 0, 'a', 0, 'c', 0, 'e', 0, ' ', 0,
	'8', 0, '6', 0, '8', 0, '0', 0, '8', 0,
};

uint8_t g_UsbDeviceString4[] = {
	0,
	USB_DESCRIPTOR_TYPE_STRING,
	'm', 0, 'p', 0, 'y', 0, ' ', 0,
	'V', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0,
	'P', 0, 'o', 0, 'r', 0, 't', 0, ' ', 0,
};

uint8_t g_UsbDeviceString5[] = {
	0,
	USB_DESCRIPTOR_TYPE_STRING,
	'm', 0, 'p', 0, 'y', 0, ' ', 0,
	'S', 0, 'D', 0, ' ', 0,
	'C', 0, 'a', 0, 'r', 0, 'd', 0, ' ', 0,
};

/* Define string descriptor size */
uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
	sizeof(g_UsbDeviceString0), sizeof(g_UsbDeviceString1),
	sizeof(g_UsbDeviceString2), sizeof(g_UsbDeviceString3),
	sizeof(g_UsbDeviceString4), sizeof(g_UsbDeviceString5),
};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {
	g_UsbDeviceString0, g_UsbDeviceString1,
	g_UsbDeviceString2, g_UsbDeviceString3,
	g_UsbDeviceString4, g_UsbDeviceString5,
};

usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {
	{
		g_UsbDeviceStringDescriptorArray,
		g_UsbDeviceStringDescriptorLength, (uint16_t)0x0409,
	}
};

usb_language_list_t g_UsbDeviceLanguageList = {
	g_UsbDeviceString0, sizeof(g_UsbDeviceString0),
	g_UsbDeviceLanguage, USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
* Code
******************************************************************************/
void usbd_vpid_release(uint16_t vid, uint16_t pid,
	uint16_t device_release_num, int cdc_only)
{
	if (cdc_only) {
		// Make it look like a Communications device if we're only
		// using CDC. Otherwise, windows gets confused when we tell it that
		// its a composite device with only a cdc serial interface.
		g_UsbDeviceDescriptor[4] = 0x02;
		g_UsbDeviceDescriptor[5] = 0x00;
		g_UsbDeviceDescriptor[6] = 0x00;
	} else {
		// For the other modes, we make this look like a composite device.
		g_UsbDeviceDescriptor[4] = 0xef;
		g_UsbDeviceDescriptor[5] = 0x02;
		g_UsbDeviceDescriptor[6] = 0x01;
	}
	g_UsbDeviceDescriptor[8] = USB_SHORT_GET_LOW(vid);
	g_UsbDeviceDescriptor[9] = USB_SHORT_GET_HIGH(vid);
	g_UsbDeviceDescriptor[10] = USB_SHORT_GET_LOW(pid);
	g_UsbDeviceDescriptor[11] = USB_SHORT_GET_HIGH(pid);
	g_UsbDeviceDescriptor[12] = USB_SHORT_GET_LOW(device_release_num);
	g_UsbDeviceDescriptor[13] = USB_SHORT_GET_HIGH(device_release_num);
}



USBDCfgFix_t g_cfgFix;

#define FIX_STR_DESC_SIZE(ndx) g_UsbDeviceString##ndx[0] = sizeof(g_UsbDeviceString##ndx)
void USBD_BeginFixCfgData(void)
{
	g_cfgFix.d32 = 0;
	FIX_STR_DESC_SIZE(0);
	FIX_STR_DESC_SIZE(1);
	FIX_STR_DESC_SIZE(2);
	FIX_STR_DESC_SIZE(3);
	FIX_STR_DESC_SIZE(4);
	FIX_STR_DESC_SIZE(5);

	g_cfgFix.epInNdx = g_cfgFix.epOutNdx = 1;
	memcpy(g_UsbDevCfgDesc, s_UsbDevCfgDescHdr, sizeof(s_UsbDevCfgDescHdr));
	g_cfgFix.cfgDescSize = sizeof(s_UsbDevCfgDescHdr);
}

void USBD_EndFixCfgData(void)
{
	g_UsbDevCfgDesc[2] = USB_SHORT_GET_LOW(g_cfgFix.cfgDescSize);
	g_UsbDevCfgDesc[3] = USB_SHORT_GET_HIGH(g_cfgFix.cfgDescSize);
	g_UsbDevCfgDesc[4] = g_cfgFix.itfNdx;
}

// note that EP0 is control, can't be dedicated for user classes
#define MAX_IN_EP_CNT     5
#define MAX_OUT_EP_CNT    5
#define APPEND_ITF_DESC(a) do{ \
	memcpy(g_UsbDevCfgDesc + g_cfgFix.cfgDescSize, a, sizeof(a));\
	g_cfgFix.cfgDescSize += sizeof(a);	\
	}while(0)

int USBD_AddItf_MSC(void)
{
	if (g_cfgFix.epInNdx + 1 > MAX_IN_EP_CNT || g_cfgFix.epOutNdx + 1 > MAX_OUT_EP_CNT)
		return -1L;  // not enough free endpoints!
	g_mscDiskEndpoints[0].endpointAddress = g_cfgFix.epInNdx | (USB_IN << 7U);
	g_mscDiskEndpoints[1].endpointAddress = g_cfgFix.epOutNdx | (USB_OUT << 7U);
	g_mscDiskInterfaces[0].interfaceNumber = g_cfgFix.itfNdx;
	s_UsbDevCfgDescMscPart[2] = g_cfgFix.itfNdx;
	s_UsbDevCfgDescMscPart[11] = g_cfgFix.epInNdx | (USB_IN << 7U);
	s_UsbDevCfgDescMscPart[18] = g_cfgFix.epOutNdx| (USB_OUT << 7U);
	g_cfgFix.roMscEpInNdx = g_cfgFix.epInNdx++;
	g_cfgFix.roMscEpOutNdx = g_cfgFix.epOutNdx++;
	g_cfgFix.roMscItfNdx = g_cfgFix.itfNdx++;

	APPEND_ITF_DESC(s_UsbDevCfgDescMscPart);
	return 0;
}

int USBD_AddItf_VCP(void)
{
	if (g_cfgFix.epInNdx + 2 > MAX_IN_EP_CNT || g_cfgFix.epOutNdx + 1 > MAX_OUT_EP_CNT)
		return -1L;  // not enough free endpoints!
	g_cdcVcomCicEndpoints[0].endpointAddress = g_cfgFix.epInNdx       | (USB_IN << 7U);  // cic.epin
	g_cdcVcomDicEndpoints[0].endpointAddress = (g_cfgFix.epInNdx + 1) | (USB_IN << 7U);  // dic.epin
	g_cdcVcomDicEndpoints[1].endpointAddress = g_cfgFix.epOutNdx      | (USB_OUT << 7U); // dic.epout
	g_cdcVcomInterfaces[0].interfaceNumber = g_cfgFix.itfNdx;   // CIC itf
	g_cdcVcomInterfaces[1].interfaceNumber = g_cfgFix.itfNdx + 1;	// DIC itf

	/* offset to fix in cdc itf desc
	2, 1stItfNdx
	10, cicItfNdx
	38, cicEpInNdx
	45, dicItfNdx
	54, dicEpInNdx
	61, dicEpOutNdx
	*/
	s_UsbDevCfgDescCdcPart[ 2] = g_cfgFix.itfNdx;
	s_UsbDevCfgDescCdcPart[10] = g_cfgFix.itfNdx;
	s_UsbDevCfgDescCdcPart[38] = g_cfgFix.epInNdx | (USB_IN << 7U);
	s_UsbDevCfgDescCdcPart[45] = g_cfgFix.itfNdx + 1;
	s_UsbDevCfgDescCdcPart[54] = (g_cfgFix.epInNdx + 1) | (USB_IN << 7U);
	s_UsbDevCfgDescCdcPart[61] = g_cfgFix.epOutNdx | (USB_OUT << 7U);

	g_cfgFix.roCdcCicItfNdx = g_cfgFix.itfNdx++;
	g_cfgFix.roCdcCicEpNdx = g_cfgFix.epInNdx++;
	g_cfgFix.roCdcDicItfNdx = g_cfgFix.itfNdx++;
	g_cfgFix.roCdcDicEpInNdx = g_cfgFix.epInNdx++;
	g_cfgFix.roCdcDicEpOutNdx = g_cfgFix.epOutNdx++;

	APPEND_ITF_DESC(s_UsbDevCfgDescCdcPart);
	return 0;
}

#define HID_DESC_OFFSET_SUBCLASS (6)
#define HID_DESC_OFFSET_PROTOCOL (7)
#define HID_DESC_OFFSET_SUBDESC (9)
#define HID_DESC_OFFSET_REPORT_DESC_LEN (16)
#define HID_DESC_OFFSET_MAX_PACKET_LO (22)
#define HID_DESC_OFFSET_MAX_PACKET_HI (23)
#define HID_DESC_OFFSET_POLLING_INTERVAL (24)
#define HID_DESC_OFFSET_MAX_PACKET_OUT_LO (29)
#define HID_DESC_OFFSET_MAX_PACKET_OUT_HI (30)
#define HID_DESC_OFFSET_POLLING_INTERVAL_OUT (31)

uint8_t usbd_mode;

uint8_t usbd_get_mode(void)
{
	return usbd_mode;
}


int usbd_select_mode(uint32_t mode)
{
	int ret = 0;

	usbd_mode = mode;
	USBD_BeginFixCfgData();
	// construct config desc
	if (mode & USBD_MODE_CDC)
		ret |= USBD_AddItf_VCP();
	if (mode & USBD_MODE_MSC)
		ret |= USBD_AddItf_MSC();

	USBD_EndFixCfgData();
	return ret;
}


/*!
 * @brief USB device get device descriptor function.
 *
 * This function gets the device descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param deviceDescriptor The pointer to the device descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
	usb_device_get_device_descriptor_struct_t *deviceDescriptor)
{
	deviceDescriptor->buffer = g_UsbDeviceDescriptor;
	deviceDescriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
	return kStatus_USB_Success;
}

/*!
 * @brief USB device get configuration descriptor function.
 *
 * This function gets the configuration descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param configurationDescriptor The pointer to the configuration descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetConfigurationDescriptor(
	usb_device_handle handle,
	usb_device_get_configuration_descriptor_struct_t *configurationDescriptor)
{
	if (USB_COMPOSITE_CONFIGURE_INDEX > configurationDescriptor->configuration) {
		configurationDescriptor->buffer = g_UsbDevCfgDesc;
		configurationDescriptor->length = g_cfgFix.cfgDescSize;
		return kStatus_USB_Success;
	}
	return kStatus_USB_InvalidRequest;
}

/*!
 * @brief USB device get string descriptor function.
 *
 * This function gets the string descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param stringDescriptor Pointer to the string descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
	usb_device_get_string_descriptor_struct_t *stringDescriptor)
{
	if (stringDescriptor->stringIndex == 0) {
		stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
		stringDescriptor->length = g_UsbDeviceLanguageList.stringLength;
	} else {
		uint8_t langId = 0;
		uint8_t langIndex = USB_DEVICE_STRING_COUNT;

		for (; langId < USB_DEVICE_LANGUAGE_COUNT; langId++) {
			if (stringDescriptor->languageId ==
				g_UsbDeviceLanguageList.languageList[langId].languageId) {
				if (stringDescriptor->stringIndex < USB_DEVICE_STRING_COUNT)
					langIndex = stringDescriptor->stringIndex;
				break;
			}
		}

		if (USB_DEVICE_STRING_COUNT == langIndex)
			return kStatus_USB_InvalidRequest;
		stringDescriptor->buffer =
			(uint8_t *)g_UsbDeviceLanguageList.languageList[langId].string[langIndex];
		stringDescriptor->length =
			g_UsbDeviceLanguageList.languageList[langId].length[langIndex];
	}
	return kStatus_USB_Success;
}

/*!
 * @brief USB device set speed function.
 *
 * This function sets the speed of the USB device.
 *
 * Due to the difference of HS and FS descriptors, the device descriptors and configurations need to be updated to match
 * current speed.
 * As the default, the device descriptors and configurations are configured by using FS parameters for both EHCI and
 * KHCI.
 * When the EHCI is enabled, the application needs to call this function to update device by using current speed.
 * The updated information includes endpoint max packet size, endpoint interval, etc.
 *
 * @param handle The USB device handle.
 * @param speed Speed type. USB_SPEED_HIGH/USB_SPEED_FULL/USB_SPEED_LOW.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed)
{
	usb_descriptor_union_t *ptr1;
	usb_descriptor_union_t *ptr2;
	ptr1 = (usb_descriptor_union_t *)(&g_UsbDevCfgDesc[0]);
	ptr2 = (usb_descriptor_union_t *)(&g_UsbDevCfgDesc[g_cfgFix.cfgDescSize - 1]);
	int i;

	while (ptr1 < ptr2) {
		if (ptr1->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT) {
			if (USB_SPEED_HIGH == speed) {
				if (g_cfgFix.roCdcCicEpNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					ptr1->endpoint.bInterval = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				}
				else if (g_cfgFix.roCdcDicEpInNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_BULK_IN_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
				} else if (USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_BULK_OUT_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roMscEpInNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_MSC_DISK_BULK_IN_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roMscEpOutNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_MSC_DISK_BULK_OUT_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
				}
				// todo: if add more classes, need to adjust as above
			} else {
				if (g_cfgFix.roCdcCicEpNdx ==
					(ptr1->endpoint.bEndpointAddress &
					USB_ENDPOINT_NUMBER_MASK)) {
					ptr1->endpoint.bInterval = FS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roCdcDicEpInNdx ==
					(ptr1->endpoint.bEndpointAddress &
					USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_BULK_IN_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roCdcDicEpOutNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roMscEpInNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_MSC_DISK_BULK_IN_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				} else if (g_cfgFix.roMscEpOutNdx ==
					(ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)) {
					USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_MSC_DISK_BULK_OUT_PACKET_SIZE,
						ptr1->endpoint.wMaxPacketSize);
				}
			}
		}
		ptr1 = (usb_descriptor_union_t *)((uint8_t *)ptr1 + ptr1->common.bLength);
	}

	for (i = 0; i < USB_CDC_VCOM_CIC_ENDPOINT_COUNT; i++) {
		if (USB_SPEED_HIGH == speed)
			g_cdcVcomCicEndpoints[i].maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
		else
			g_cdcVcomCicEndpoints[i].maxPacketSize = FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
	}

	for (i = 0; i < USB_CDC_VCOM_DIC_ENDPOINT_COUNT; i++) {
		if (USB_SPEED_HIGH == speed)
			g_cdcVcomDicEndpoints[i].maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
		else
			g_cdcVcomDicEndpoints[i].maxPacketSize = FS_CDC_VCOM_BULK_IN_PACKET_SIZE;
	}

	for (int i = 0; i < USB_MSC_DISK_ENDPOINT_COUNT; i++) {
		if (USB_SPEED_HIGH == speed)
			g_mscDiskEndpoints[i].maxPacketSize = HS_MSC_DISK_BULK_IN_PACKET_SIZE;
		else
			g_mscDiskEndpoints[i].maxPacketSize = FS_MSC_DISK_BULK_IN_PACKET_SIZE;
	}
	// todo: if add more classes, need to adjust as above
	return kStatus_USB_Success;
}
