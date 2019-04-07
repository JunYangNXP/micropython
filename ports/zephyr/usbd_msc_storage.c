/*
 * This file is part of the Micro Python project, http://micropython.org/
 */

/**
  ******************************************************************************
  * @file    usbd_storage_msd.c
  * @author  MCD application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the disk operations functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * Heavily modified by dpgeorge for Micro Python.
  *
  ******************************************************************************
  */

#include <stdint.h>
#include "fsl_device_registers.h"
//#include "clock_config.h"
//#include "board.h"
//#include "fsl_debug_console.h"

#include "usb_cdc_msc/usb_device_config.h"
#include "usb/include/usb.h"
#include "usb/device/usb_device.h"

#include "usb_cdc_msc/usb_device_class.h"
#include "usb_cdc_msc/usb_device_msc.h"
#include "usb_cdc_msc/usb_device_ch9.h"
#include "usb_cdc_msc/usb_device_descriptor.h"

#include <stdio.h>
#include <stdlib.h>

#include "usb_cdc_msc/composite.h"

//#include "mpconfigboard.h"
#include "usbd_msc_storage.h"
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
//#include "storage.h"
#include "fsl_sdmmc/fsl_sd.h"

//#include "sdcard.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#undef usb_echo
#define usb_echo(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)

/*******************************************************************************
* Variables
******************************************************************************/

static usb_device_composite_struct_t *g_deviceComposite;

USB_DATA_ALIGNMENT usb_device_inquiry_data_fromat_struct_t g_InquiryInfo = {
	(USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
		USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
	(uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
	USB_DEVICE_MSC_UFI_VERSIONS,
	0x02,
	USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
	{0x00, 0x00, 0x00},
	{'N', 'X', 'P', ' ', 'S', 'E', 'M', 'I'},
	{'N', 'X', 'P', ' ', 'M', 'A', 'S', 'S', ' ', 'S', 'T', 'O', 'R', 'A', 'G', 'E'},
	{'0', '0', '0', '1'}
};

USB_DATA_ALIGNMENT usb_device_mode_parameters_header_struct_t g_ModeParametersHeader = {
	/*refer to ufi spec mode parameter header*/
	0x0000, /*!< Mode Data Length*/
	0x00,   /*!<Default medium type (current mounted medium type)*/
	0x00,   /*!MODE SENSE command, a Write Protected bit of zero indicates the medium is write enabled*/
	{0x00, 0x00, 0x00, 0x00} /*!<This bit should be set to zero*/
};

USB_DMA_DATA_NONINIT_SUB uint32_t g_mscReadRequestBuffer[USB_DEVICE_MSC_READ_BUFF_SIZE >> 2] __aligned(512);

USB_DMA_DATA_NONINIT_SUB uint32_t g_mscWriteRequestBuffer[USB_DEVICE_MSC_WRITE_BUFF_SIZE >> 2]__aligned(512);

/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief device msc callback function.
 *
 * This function handle the disk class specified event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */

#define USB_MSC_BLOCK_SIZE_LOG2	9

static void lba_cfg(usb_device_lba_information_struct_t *lbaInf)
{
	assert(sdcard_is_present());

	lbaInf->totalLbaNumberSupports = sdcard_get_lba_count();

	lbaInf->lengthOfEachLba = 512;
	lbaInf->bulkInBufferSize = sizeof(g_mscReadRequestBuffer);
	lbaInf->bulkOutBufferSize = sizeof(g_mscWriteRequestBuffer);
	lbaInf->logicalUnitNumberSupported = LOGICAL_UNIT_SUPPORTED;
}

#define USB_DEVICE_SDCARD_BLOCK_SIZE_POWER (9U)
#define USB_DEVICE_MSC_ADMA_TABLE_WORDS (8U)

extern status_t ums_sd_read_blocks(void *card, uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);
//extern sd_card_t g_sd;

usb_status_t usbd_msc_cb(class_handle_t handle, uint32_t event, void *param)
{
	usb_status_t error = kStatus_USB_Success;
	usb_device_lba_app_struct_t *lba;
	usb_device_ufi_app_struct_t *ufi;
	mp_uint_t t1 = kStatus_Fail;

	switch (event)
	{
	case kUSB_DeviceMscEventReadResponse:
		lba = (usb_device_lba_app_struct_t *)param;
		break;
	case kUSB_DeviceMscEventWriteResponse:
		lba = (usb_device_lba_app_struct_t *)param;

		/*write the data to sd card*/
		if (0 != lba->size) {
			if (sdcard_is_present()) {
				t1 = sdcard_write_blocks(lba->buffer,
					lba->offset, lba->size >> USB_MSC_BLOCK_SIZE_LOG2);
			} else {
				error = kStatus_USB_Error;
				break;
			}
			if (0 != t1)
				error = kStatus_USB_Error;
		}
		break;
	case kUSB_DeviceMscEventWriteRequest:
		lba = (usb_device_lba_app_struct_t *)param;
		/*get a buffer to store the data from host*/

		lba->buffer = (uint8_t *)&g_mscWriteRequestBuffer[0];
		break;
	case kUSB_DeviceMscEventReadRequest:
		lba = (usb_device_lba_app_struct_t *)param;
		lba->buffer = (uint8_t *)&g_mscReadRequestBuffer[0];
		if (sdcard_is_present())
			t1 = ums_sd_read_blocks(0, lba->buffer, lba->offset, lba->size >> USB_DEVICE_SDCARD_BLOCK_SIZE_POWER);
		if (0 != t1) {
			g_deviceComposite->mscDisk.readWriteError = 1;
			usb_echo(
				"Read error, error = 0xx%x \t Please check read request buffer size(must be less than 128 "
				"sectors)\r\n",
				t1);
			error = kStatus_USB_Error;
		}

		break;
	case kUSB_DeviceMscEventGetLbaInformation:
		lba_cfg((usb_device_lba_information_struct_t *)param);
		break;
	case kUSB_DeviceMscEventTestUnitReady:
		/*change the test unit ready command's sense data if need, be careful to modify*/
		ufi = (usb_device_ufi_app_struct_t *)param;
		break;
	case kUSB_DeviceMscEventInquiry:
		ufi = (usb_device_ufi_app_struct_t *)param;
		ufi->size = sizeof(usb_device_inquiry_data_fromat_struct_t);
		ufi->buffer = (uint8_t *)&g_InquiryInfo;
		break;
	case kUSB_DeviceMscEventModeSense:
		ufi = (usb_device_ufi_app_struct_t *)param;
		ufi->size = sizeof(usb_device_mode_parameters_header_struct_t);
		ufi->buffer = (uint8_t *)&g_ModeParametersHeader;
		break;
	case kUSB_DeviceMscEventModeSelect:
		break;
	case kUSB_DeviceMscEventModeSelectResponse:
		ufi = (usb_device_ufi_app_struct_t *)param;
		break;
	case kUSB_DeviceMscEventFormatComplete:
		break;
	case kUSB_DeviceMscEventRemovalRequest:
		break;
	default:
		break;
	}
	return error;
}

/*!
 * @brief msc device set configuration function.
 *
 * This function sets configuration for msc class.
 *
 * @param handle The msc class handle.
 * @param configure The msc class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usbd_msc_cfg(class_handle_t handle, uint8_t configure)
{
	return kStatus_USB_Error;
}
/*!
 * @brief device msc init function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param deviceComposite          The pointer to the composite device structure.
 * @return kStatus_USB_Success .
 */
usb_status_t usbd_msc_init(usb_device_composite_struct_t *deviceComposite)
{
	g_deviceComposite = deviceComposite;
	return kStatus_USB_Success;
}
