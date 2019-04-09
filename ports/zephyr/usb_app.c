/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014, 2015 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdarg.h>
#include <string.h>
#include "usb_cdc_msc/usb_device_config.h"
#include "usb/include/usb.h"
#include "usb/device/usb_device.h"

#include "usb_cdc_msc/usb_device_class.h"
#include "usb_cdc_msc/usb_device_msc.h"
#include "usb_cdc_msc/usb_device_cdc_acm.h"
#include "usb_cdc_msc/usb_device_ch9.h"
#include "usb_cdc_msc/usb_device_descriptor.h"

#include "py/objstr.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "usb_app.h"
#include "usb_cdc_msc/composite.h"

static void pyb_buf_get_for_send(mp_obj_t o, mp_buffer_info_t *bufinfo, byte *tmp_data)
{
	if (MP_OBJ_IS_INT(o)) {
		tmp_data[0] = mp_obj_get_int(o);
		bufinfo->buf = tmp_data;
		bufinfo->len = 1;
		bufinfo->typecode = 'B';
	} else {
		mp_get_buffer_raise(o, bufinfo, MP_BUFFER_READ);
	}
}

static mp_obj_t pyb_buf_get_for_recv(mp_obj_t o, vstr_t *vstr)
{
	if (MP_OBJ_IS_INT(o)) {
		vstr_init_len(vstr, mp_obj_get_int(o));
		return MP_OBJ_NULL;
	} else {
		mp_buffer_info_t bufinfo;

		mp_get_buffer_raise(o, &bufinfo, MP_BUFFER_WRITE);
		vstr->buf = bufinfo.buf;
		vstr->len = bufinfo.len;
		return o;
	}
}

// this will be persistent across a soft-reset
mp_uint_t pyb_usb_flags = 0;

// predefined hid keyboard data
void pyb_usb_init0(void)
{
	mp_hal_set_interrupt_char(-1);
}

bool pyb_usb_dev_init(uint16_t vid, uint16_t pid, usb_device_mode_t mode)
{
#ifdef USE_DEVICE_MODE
	if (!(pyb_usb_flags & PYB_USB_FLAG_DEV_ENABLED)) {
		// only init USB once in the device's power-lifetime
		usbd_vpid_release(vid, pid, 0x0200, mode == USBD_MODE_CDC);
		if (usbd_select_mode(mode) < 0)
			return false;
		USBAPP_Init();
	}
	pyb_usb_flags |= PYB_USB_FLAG_DEV_ENABLED;
#endif
	printk("pyb_usb_dev_init successfully\r\n");
	return true;
}

void pyb_usb_dev_deinit(void)
{
	if (pyb_usb_flags & PYB_USB_FLAG_DEV_ENABLED) {
		USBAPP_Deinit();
		pyb_usb_flags &= ~PYB_USB_FLAG_DEV_ENABLED;
	}
}

bool usb_vcp_is_enabled(void)
{
	return (pyb_usb_flags & PYB_USB_FLAG_DEV_ENABLED) != 0;
}

void usb_vcp_send_strn(const char *str, int len)
{
#ifdef USE_DEVICE_MODE
	bool omvUsbDbg = vcom_omvide_connected();

	if (pyb_usb_flags & PYB_USB_FLAG_DEV_ENABLED) {
		if (omvUsbDbg) {
			vcom_omv_write((const uint8_t*)str, len);
		} else {
			vcom_write_always((const uint8_t*)str, len);
		}
	}
#endif
}

/******************************************************************************/
// Micro Python bindings for USB

/*
  Philosophy of USB driver and Python API: pyb.usb_mode(...) configures the USB
  on the board.  The USB itself is not an entity, rather the interfaces are, and
  can be accessed by creating objects, such as pyb.USB_VCP() and pyb.USB_HID().

  We have:

    pyb.usb_mode()          # return the current usb mode
    pyb.usb_mode(None)      # disable USB
    pyb.usb_mode('VCP')     # enable with VCP interface
    pyb.usb_mode('VCP+MSC') # enable with VCP and MSC interfaces
    pyb.usb_mode('VCP+HID') # enable with VCP and HID, defaulting to mouse protocol
    pyb.usb_mode('VCP+HID', vid=0xf055, pid=0x9800) # specify VID and PID
    pyb.usb_mode('VCP+HID', hid=pyb.hid_mouse)
    pyb.usb_mode('VCP+HID', hid=pyb.hid_keyboard)
    pyb.usb_mode('VCP+HID', pid=0x1234, hid=(subclass, protocol, max_packet_len, polling_interval, report_desc))

    vcp = pyb.USB_VCP() # get the VCP device for read/write
    hid = pyb.USB_HID() # get the HID device for write/poll

  Possible extensions:
    pyb.usb_mode('host', ...)
    pyb.usb_mode('OTG', ...)
    pyb.usb_mode(..., port=2) # for second USB port
*/

STATIC mp_obj_t pyb_usb_mode(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
	static const mp_arg_t allowed_args[] = {
		{
			MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}
		},
		{
			MP_QSTR_vid, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = USBD_VID}
		},
		{
			MP_QSTR_pid, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}
		},
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
	uint8_t mode;
	const char *mode_str;
#ifdef USE_DEVICE_MODE
	uint16_t vid;
	uint16_t pid;
#endif

	// fetch the current usb mode -> pyb.usb_mode()
	if (n_args == 0) {
#if defined(USE_HOST_MODE)
		return MP_OBJ_NEW_QSTR(MP_QSTR_host);
#elif defined(USE_DEVICE_MODE)
		mode = usbd_get_mode();
		switch (mode) {
		case USBD_MODE_CDC:
			return MP_OBJ_NEW_QSTR(MP_QSTR_VCP);
		case USBD_MODE_MSC:
			return MP_OBJ_NEW_QSTR(MP_QSTR_MSC);
		case USBD_MODE_CDC_MSC:
			return MP_OBJ_NEW_QSTR(MP_QSTR_VCP_plus_MSC);
		default:
			return mp_const_none;
		}
#endif
	}

	// parse args
	mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
	mode_str = mp_obj_str_get_str(args[0].u_obj);

	// record the fact that the usb has been explicitly configured
	pyb_usb_flags |= PYB_USB_FLAG_USB_MODE_CALLED;

	// check if user wants to disable the USB
	if (args[0].u_obj == mp_const_none) {
		// disable usb
#if defined(USE_DEVICE_MODE)
		pyb_usb_dev_deinit();
#endif
		return mp_const_none;
	}

#if defined(USE_HOST_MODE)
	// hardware configured for USB host mode

	if (strcmp(mode_str, "host") == 0)
		pyb_usb_host_init();
	else
		goto bad_mode;

#elif defined(USE_DEVICE_MODE)
	// hardware configured for USB device mode

	// get the VID, PID and USB mode
	// note: PID=-1 means select PID based on mode
	// note: we support CDC as a synonym for VCP for backward compatibility
	vid = args[1].u_int;
	pid = args[2].u_int;
	if (strcmp(mode_str, "CDC+MSC") == 0 || strcmp(mode_str, "VCP+MSC") == 0) {
		if (args[2].u_int == -1)
			pid = USBD_PID_CDC_MSC;
		mode = USBD_MODE_CDC_MSC;
	} else if (strcmp(mode_str, "CDC+HID") == 0 || strcmp(mode_str, "VCP+HID") == 0) {
		if (args[2].u_int == -1)
			pid = USBD_PID_CDC_HID;
		mode = USBD_MODE_CDC_HID;
	} else if (strcmp(mode_str, "CDC") == 0 || strcmp(mode_str, "VCP") == 0) {
		if (args[2].u_int == -1)
			pid = USBD_PID_CDC;
		mode = USBD_MODE_CDC;
	} else
		goto bad_mode;

	// init the USB device
	if (!pyb_usb_dev_init(vid, pid, mode))
		goto bad_mode;

#else
	// hardware not configured for USB
	goto bad_mode;
#endif

	return mp_const_none;

bad_mode:
	nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "bad USB mode"));
}
MP_DEFINE_CONST_FUN_OBJ_KW(pyb_usb_mode_obj, 0, pyb_usb_mode);

/******************************************************************************/
// Micro Python bindings for USB VCP

/// \moduleref pyb
/// \class USB_VCP - USB virtual comm port
///
/// The USB_VCP class allows creation of an object representing the USB
/// virtual comm port.  It can be used to read and write data over USB to
/// the connected host.

typedef struct _pyb_usb_vcp_obj_t {
	mp_obj_base_t base;
} pyb_usb_vcp_obj_t;

STATIC const pyb_usb_vcp_obj_t pyb_usb_vcp_obj = {
	{&pyb_usb_vcp_type}
};

STATIC void pyb_usb_vcp_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
	mp_print_str(print, "USB_VCP()");
}

/// \classmethod \constructor()
/// Create a new USB_VCP object.
STATIC mp_obj_t pyb_usb_vcp_make_new(const mp_obj_type_t *type,
	size_t n_args, size_t n_kw, const mp_obj_t *args)
{
	mp_arg_check_num(n_args, n_kw, 0, 0, false);
	return (mp_obj_t)&pyb_usb_vcp_obj;
}

STATIC mp_obj_t pyb_usb_vcp_isconnected(mp_obj_t self_in)
{
	return mp_obj_new_bool(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_vcp_isconnected_obj, pyb_usb_vcp_isconnected);

// deprecated in favour of USB_VCP.isconnected
STATIC mp_obj_t pyb_have_cdc(void)
{
	return pyb_usb_vcp_isconnected(MP_OBJ_NULL);
}
MP_DEFINE_CONST_FUN_OBJ_0(pyb_have_cdc_obj, pyb_have_cdc);

/// \method any()
/// Return `True` if any characters waiting, else `False`.
STATIC mp_obj_t pyb_usb_vcp_any(mp_obj_t self_in)
{
	if (usbd_vcom_rx_buf_used() > 0)
		return mp_const_true;
	else
		return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_vcp_any_obj, pyb_usb_vcp_any);

/// \method send(data, *, timeout=5000)
/// Send data over the USB VCP:
///
///   - `data` is the data to send (an integer to send, or a buffer object).
///   - `timeout` is the timeout in milliseconds to wait for the send.
///
/// Return value: number of bytes sent.
STATIC const mp_arg_t pyb_usb_vcp_send_args[] = {
	{
		MP_QSTR_data,
		MP_ARG_REQUIRED | MP_ARG_OBJ,
		{.u_obj = MP_OBJ_NULL}
	},
	{
		MP_QSTR_timeout,
		MP_ARG_KW_ONLY | MP_ARG_INT,
		{.u_int = 5000}
	},
};
#define PYB_USB_VCP_SEND_NUM_ARGS MP_ARRAY_SIZE(pyb_usb_vcp_send_args)

STATIC mp_obj_t pyb_usb_vcp_send(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
	mp_buffer_info_t bufinfo;
	uint8_t data[1];
	mp_arg_val_t vals[PYB_USB_VCP_SEND_NUM_ARGS];

	mp_arg_parse_all(n_args - 1, args + 1, kw_args,
		PYB_USB_VCP_SEND_NUM_ARGS, pyb_usb_vcp_send_args, vals);

	pyb_buf_get_for_send(vals[0].u_obj, &bufinfo, data);
	return mp_obj_new_int(0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_usb_vcp_send_obj, 1, pyb_usb_vcp_send);

/// \method recv(data, *, timeout=5000)
///
/// Receive data on the bus:
///
///   - `data` can be an integer, which is the number of bytes to receive,
///     or a mutable buffer, which will be filled with received bytes.
///   - `timeout` is the timeout in milliseconds to wait for the receive.
///
/// Return value: if `data` is an integer then a new buffer of the bytes received,
/// otherwise the number of bytes read into `data` is returned.
STATIC mp_obj_t pyb_usb_vcp_recv(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
	vstr_t vstr;
	mp_obj_t o_ret;
	mp_arg_val_t vals[PYB_USB_VCP_SEND_NUM_ARGS];

	mp_arg_parse_all(n_args - 1, args + 1, kw_args,
		PYB_USB_VCP_SEND_NUM_ARGS, pyb_usb_vcp_send_args, vals);

	o_ret = pyb_buf_get_for_recv(vals[0].u_obj, &vstr);

	if (o_ret != MP_OBJ_NULL) {
		return mp_obj_new_int(0); // number of bytes read into given buffer
	} else {
		vstr.len = 0; // set actual number of bytes read
		return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr); // create a new buffer
	}
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_usb_vcp_recv_obj, 1, pyb_usb_vcp_recv);

mp_obj_t pyb_usb_vcp___exit__(mp_uint_t n_args, const mp_obj_t *args)
{
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_usb_vcp___exit___obj, 4, 4, pyb_usb_vcp___exit__);

STATIC const mp_rom_map_elem_t pyb_usb_vcp_locals_dict_table[] = {
	{
		MP_ROM_QSTR(MP_QSTR_isconnected),
		MP_ROM_PTR(&pyb_usb_vcp_isconnected_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_any),
		MP_ROM_PTR(&pyb_usb_vcp_any_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_send),
		MP_ROM_PTR(&pyb_usb_vcp_send_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_recv),
		MP_ROM_PTR(&pyb_usb_vcp_recv_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_read),
		MP_ROM_PTR(&mp_stream_read_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_readinto),
		MP_ROM_PTR(&mp_stream_readinto_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_readline),
		MP_ROM_PTR(&mp_stream_unbuffered_readline_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_readlines),
		MP_ROM_PTR(&mp_stream_unbuffered_readlines_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_write),
		MP_ROM_PTR(&mp_stream_write_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_close),
		MP_ROM_PTR(&mp_identity_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR___del__),
		MP_ROM_PTR(&mp_identity_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR___enter__),
		MP_ROM_PTR(&mp_identity_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR___exit__),
		MP_ROM_PTR(&pyb_usb_vcp___exit___obj)
	},
};

STATIC MP_DEFINE_CONST_DICT(pyb_usb_vcp_locals_dict, pyb_usb_vcp_locals_dict_table);

STATIC mp_uint_t pyb_usb_vcp_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode)
{
	int ret = vcom_read(buf, size);

	if (ret == 0) {
		*errcode = MP_EAGAIN;
		return MP_STREAM_ERROR;
	}
	return ret;
}

STATIC mp_uint_t pyb_usb_vcp_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode)
{
	int ret = vcom_write(buf, size);

	if (ret == 0) {
		*errcode = MP_EAGAIN;
		return MP_STREAM_ERROR;
	}
	return ret;
}

STATIC mp_uint_t pyb_usb_vcp_ioctl(mp_obj_t self_in,
	mp_uint_t request, mp_uint_t arg, int *errcode)
{
	mp_uint_t ret;

	if (request == MP_STREAM_POLL) {
		mp_uint_t flags = arg;
		ret = 0;
		if ((flags & MP_STREAM_POLL_RD) && usbd_vcom_rx_buf_used() > 0) {
			ret |= MP_STREAM_POLL_RD;
		}
		if ((flags & MP_STREAM_POLL_WR) && usbd_vcom_tx_buf_half_empty()) {
			ret |= MP_STREAM_POLL_WR;
		}
	} else {
		*errcode = MP_EINVAL;
		ret = MP_STREAM_ERROR;
	}
	return ret;
}

STATIC const mp_stream_p_t pyb_usb_vcp_stream_p = {
	.read = pyb_usb_vcp_read,
	.write = pyb_usb_vcp_write,
	.ioctl = pyb_usb_vcp_ioctl,
};

const mp_obj_type_t pyb_usb_vcp_type = {
	{ &mp_type_type },
	.name = MP_QSTR_USB_VCP,
	.print = pyb_usb_vcp_print,
	.make_new = pyb_usb_vcp_make_new,
	.getiter = mp_identity_getiter,
	.iternext = mp_stream_unbuffered_iter,
	.protocol = &pyb_usb_vcp_stream_p,
	.locals_dict = (mp_obj_dict_t*)&pyb_usb_vcp_locals_dict,
};
