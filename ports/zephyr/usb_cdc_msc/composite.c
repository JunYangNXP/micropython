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
#include <stdio.h>
#include <stdlib.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
#include "irq.h"
#include "usb_device_config.h"
#include "../usb/include/usb.h"
#include "../usb/device/usb_device.h"

#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "fsl_device_registers.h"
//#include "clock_config.h"
//#include "board.h"
//#include "fsl_debug_console.h"
#include "composite.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#include "../usb/phy/usb_phy.h"
#endif

//#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
* Definitions
******************************************************************************/
#undef usb_echo
#define usb_echo(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)

/* USB clock source and frequency*/
#define USB_FS_CLK_SRC kCLOCK_UsbSrcFro
#define USB_FS_CLK_FREQ CLOCK_GetFreq(kCLOCK_FroHf)
#define USB_HS_CLK_SRC kCLOCK_UsbSrcUsbPll
#define USB_HS_CLK_FREQ 0U

/*******************************************************************************
* Prototypes
******************************************************************************/
/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usbd_cb(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
* Variables
******************************************************************************/
/* Composite device structure. */
usb_device_composite_struct_t g_composite;
extern usb_device_class_struct_t g_usbdCdcVcomConfig;
extern usb_device_class_struct_t g_usbdMscConfig;

/* USB device class information, support at most 5 classes */
usb_device_class_config_struct_t g_classes[class_ndx_end] = {
	// IMPORTANT! Class orders must be corresponding to "class_ndx_enum"
	{
		usbd_msc_cb, (class_handle_t)NULL, &g_usbdMscConfig
	},
	{
		usbd_vcom_cb, (class_handle_t)NULL, &g_usbdCdcVcomConfig
	},
};

/* USB device class configuration information */
usb_device_class_config_list_struct_t g_classesConfigList = {
	g_classes, usbd_cb, 2,
};

/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usbd_cb(usb_device_handle handle, uint32_t event, void *param)
{
	usb_status_t error = kStatus_USB_Error;
	uint16_t *temp16 = (uint16_t *)param;
	uint8_t *temp8 = (uint8_t *)param;

	switch (event)
	{
		case kUSB_DeviceEventBusReset:
			g_composite.attach = 0;
			error = kStatus_USB_Success;

			/* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
			if (kStatus_USB_Success ==
				USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_composite.speed))
				USB_DeviceSetSpeed(handle, g_composite.speed);
			break;
		case kUSB_DeviceEventSetConfiguration:
			if (param) {
				g_composite.attach = 1;
				g_composite.currentConfiguration = *temp8;
				usbd_vcom_cfg(g_composite.cdcVcom.cdcAcmHandle, *temp8);
				usbd_msc_cfg(g_composite.mscDisk.mscHandle, *temp8);
				error = kStatus_USB_Success;
			}
			break;
		case kUSB_DeviceEventSetInterface:
			if (g_composite.attach) {
				uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
				uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);

				if (interface < USB_INTERFACE_COUNT) {
					g_composite.currentInterfaceAlternateSetting[interface] = alternateSetting;
					error = kStatus_USB_Success;
				}
			}
			break;
		case kUSB_DeviceEventGetConfiguration:
			if (param) {
				*temp8 = g_composite.currentConfiguration;
				error = kStatus_USB_Success;
			}
			break;
		case kUSB_DeviceEventGetInterface:
			if (param) {
				uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
				if (interface < USB_INTERFACE_COUNT) {
					*temp16 = (*temp16 & 0xFF00U) |
						g_composite.currentInterfaceAlternateSetting[interface];
					error = kStatus_USB_Success;
				} else {
					error = kStatus_USB_InvalidRequest;
				}
			}
			break;
		case kUSB_DeviceEventGetDeviceDescriptor:
			if (param)
				error = USB_DeviceGetDeviceDescriptor(handle,
					(usb_device_get_device_descriptor_struct_t *)param);
			break;
		case kUSB_DeviceEventGetConfigurationDescriptor:
			if (param)
				error = USB_DeviceGetConfigurationDescriptor(handle,
					(usb_device_get_configuration_descriptor_struct_t *)param);
			break;
		case kUSB_DeviceEventGetStringDescriptor:
			if (param)
				error = USB_DeviceGetStringDescriptor(handle,
					(usb_device_get_string_descriptor_struct_t *)param);
			break;
		default:
			break;
	}

	return error;
}


/*!
 * @brief USB Interrupt service routine.
 *
 * This function serves as the USB interrupt service routine.
 *
 * @return None.
 */
extern usb_msc_struct_t g_msc;

#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
void USB_OTG1_IRQHandler(void)
{
	USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
}
#endif
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
void USB_OTG2_IRQHandler(void)
{
	USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
}
#endif

#include "irq.h"

static int g_usb_irq_connected;
static void usb_irq_connect(void)
{
	if (g_usb_irq_connected)
		return;
	g_usb_irq_connected = 1;

	IRQ_CONNECT(112, 0, USB_OTG1_IRQHandler, 0, 0);
	IRQ_CONNECT(113, 0, USB_OTG2_IRQHandler, 0, 0);
}
#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)

#define BOARD_XTAL0_CLK_HZ 24000000U

void USB_DeviceClockInit(void)
{

    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };

    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);

}

#define IRQ_PRI_USB_OTG1 14

void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif
/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, IRQ_PRI_USB_OTG1);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, /*3*/IRQ_PRI_USB_OTG1);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */
void USBAPP_Init(void)
{
	USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
	SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
	g_composite.speed = USB_SPEED_FULL;
	g_composite.attach = 0;
	g_composite.cdcVcom.cdcAcmHandle = (class_handle_t)NULL;
	g_composite.mscDisk.mscHandle = (class_handle_t)NULL;
	g_composite.deviceHandle = NULL;

	if (kStatus_USB_Success !=
		USB_DeviceClassInit(CONTROLLER_ID, &g_classesConfigList, &g_composite.deviceHandle)) {
		return;
	} else {
		g_composite.mscDisk.mscHandle = g_classesConfigList.config[0].classHandle;
		g_composite.cdcVcom.cdcAcmHandle = g_classesConfigList.config[1].classHandle;

		usbd_vcom_init(&g_composite);
		usbd_msc_init(&g_composite);
	}
	usb_irq_connect();
	USB_DeviceIsrEnable();
	USB_DeviceRun(g_composite.deviceHandle);
	//while(1);
}

int32_t USBAPP_Deinit(void)
{
	usb_status_t st = st;
	if (g_composite.deviceHandle) {
		if (USB_DeviceStop(g_composite.deviceHandle)== kStatus_USB_Success)
		{
			st = USB_DeviceDeinit(g_composite.deviceHandle);
			assert(st == kStatus_USB_Success);
			return 0;
		} else {
			return -1;
		}
	}
	return -1;
}

