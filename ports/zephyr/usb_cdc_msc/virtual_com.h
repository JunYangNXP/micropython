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
#ifndef _USB_CDC_VCOM_H_
#define _USB_CDC_VCOM_H_ 1
#include "../usb/include/usb.h"
#include "../usb/device/usb_device.h"
#include "usb_device_class.h"
#include "usb_device_descriptor.h"

#define VCP_RINGBLK_SIZE	128
#define VCP_OUTEPBUF_CNT 	4
#define VCP_INEPBUF_CNT 	3

/*******************************************************************************
* Definitions
******************************************************************************/
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#define CONTROLLER_ID kUSB_ControllerEhci0
#define DATA_BUFF_SIZE (HS_CDC_VCOM_BULK_OUT_PACKET_SIZE)
#endif

/* Currently configured line coding */
#define LINE_CODING_SIZE (0x07)
#define LINE_CODING_DTERATE (115200)
#define LINE_CODING_CHARFORMAT (0x00)
#define LINE_CODING_PARITYTYPE (0x00)
#define LINE_CODING_DATABITS (0x08)

/* Communications feature */
#define COMM_FEATURE_DATA_SIZE (0x02)
#define STATUS_ABSTRACT_STATE (0x0000)
#define COUNTRY_SETTING (0x0000)

/* Notification of serial state */
#define NOTIF_PACKET_SIZE (0x08)
#define UART_BITMAP_SIZE (0x02)
#define NOTIF_REQUEST_TYPE (0xA1)

/* Define the types for application */
typedef struct _usb_cdc_vcom_struct
{
	usb_device_handle deviceHandle; /* USB device handle. */
	class_handle_t cdcAcmHandle; /* USB CDC ACM class handle.                                                         */
	uint8_t attach;              /* A flag to indicate whether a usb device is attached. 1: attached, 0: not attached */
	uint8_t speed;               /* Speed of USB device. USB_SPEED_FULL/USB_SPEED_LOW/USB_SPEED_HIGH.                 */
	uint8_t startTransactions;   /* A flag to indicate whether a CDC device is ready to transmit and receive data.    */
	uint8_t currentConfiguration; /* Current configuration value. */
	uint8_t currentInterfaceAlternateSetting[2]; /* Current alternate setting value for CIC and DIC itfs */
} usb_cdc_vcom_struct_t;

/* Define the infomation relates to abstract control model */
typedef struct _usb_cdc_acm_info
{
	uint8_t serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE]; /* Serial state buffer of the CDC device to notify the
                                                                     serial state to host. */
	bool dtePresent;          /* A flag to indicate whether DTE is present.         */
	uint16_t breakDuration;   /* Length of time in milliseconds of the break signal */
	uint8_t dteStatus;        /* Status of data terminal equipment                  */
	uint8_t currentInterface; /* Current interface index.                           */
	uint16_t uartState;       /* UART state of the CDC device.                      */
} usb_cdc_acm_info_t;

int vcom_read(uint8_t *buf, uint32_t len/*, uint32_t timeout*/);
int vcom_write(const uint8_t *buf, uint32_t len);
void vcom_write_always(const uint8_t *buf, uint32_t len);
uint32_t usbd_vcom_rx_buf_used(void);
int usbd_vcom_tx_buf_half_empty(void);

bool vcom_omvide_connected(void);
uint32_t vcom_omv_tx_len(void);
int vcom_omv_txblk(uint8_t *pBuf, uint32_t bufSize);
void vcom_omv_write(const uint8_t *buf, uint32_t len);
#endif /* _USB_CDC_VCOM_H_ */
