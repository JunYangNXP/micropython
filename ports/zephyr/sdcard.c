/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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

#include <string.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include "ff.h"
#include "extmod/vfs.h"
#else
#include "extmod/vfs_fat.h"
#endif
#include "fsl_sdmmc/fsl_sd.h"
#include "sdcard.h"

#undef usb_echo
#define usb_echo(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)

#if MICROPY_HW_HAS_SDCARD

/******************************************************************************/
/*Micro Python bindings

Expose the SD card as an object with the block protocol.

there is a singleton SDCard object*/
const mp_obj_base_t pyb_sdcard_obj = {&pyb_sdcard_type};

STATIC mp_obj_t pyb_sdcard_make_new(const mp_obj_type_t *type,
	size_t n_args, size_t n_kw, const mp_obj_t *args)
{
	mp_arg_check_num(n_args, n_kw, 0, 0, false);

	return (mp_obj_t)&pyb_sdcard_obj;
}

STATIC mp_obj_t sd_present(mp_obj_t self)
{
	return mp_obj_new_bool(sdcard_is_present());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_present_obj, sd_present);

STATIC mp_obj_t sd_power(mp_obj_t self, mp_obj_t state)
{
	bool result;

	if (mp_obj_is_true(state)) {
		result = sdcard_power_on();
	} else {
		sdcard_power_off();
		result = true;
	}
	return mp_obj_new_bool(result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_power_obj, sd_power);

STATIC mp_obj_t sd_info(mp_obj_t self)
{
	int ret;
	uint32_t total_size, block_size, busclk_hz;
	mp_obj_t tuple[3];

	ret = sdcard_info(&total_size, &block_size, &busclk_hz);
	if (ret)
		return mp_const_none;

	tuple[0] = mp_obj_new_int_from_ull(total_size);
	tuple[1] = mp_obj_new_int_from_uint(block_size);
	tuple[2] = mp_obj_new_int(busclk_hz / 1000);

	return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_info_obj, sd_info);

STATIC mp_obj_t sd_read(mp_obj_t self, mp_obj_t block_num)
{
	uint8_t *dest = m_new(uint8_t, SDCARD_BLOCK_SIZE);
	mp_uint_t ret = sdcard_read_blocks(dest, mp_obj_get_int(block_num), 1);

	if (ret != 0) {
		m_del(uint8_t, dest, SDCARD_BLOCK_SIZE);
		nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception, "sdcard_read_blocks failed [%u]", ret));
	}

	return mp_obj_new_bytearray_by_ref(SDCARD_BLOCK_SIZE, dest);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_read_obj, sd_read);

STATIC mp_obj_t sd_write(mp_obj_t self, mp_obj_t block_num, mp_obj_t data)
{
	mp_buffer_info_t bufinfo;
	mp_uint_t ret;

	mp_get_buffer_raise(data, &bufinfo, MP_BUFFER_READ);
	if (bufinfo.len % SDCARD_BLOCK_SIZE != 0)
		nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
			"writes must be a multiple of %d bytes", SDCARD_BLOCK_SIZE));

	ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num),
		bufinfo.len / SDCARD_BLOCK_SIZE);

	if (ret != 0)
		nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception,
			"sdcard_write_blocks failed [%u]", ret));

	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sd_write_obj, sd_write);

#ifndef CONFIG_FAT_FILESYSTEM_ELM
STATIC mp_obj_t pyb_sdcard_readblocks(mp_obj_t self,
	mp_obj_t block_num, mp_obj_t buf)
{
	mp_buffer_info_t bufinfo;
	mp_uint_t ret;

	mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
	ret = sdcard_read_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
	return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_readblocks_obj, pyb_sdcard_readblocks);

STATIC mp_obj_t pyb_sdcard_writeblocks(mp_obj_t self,
	mp_obj_t block_num, mp_obj_t buf)
{
	mp_buffer_info_t bufinfo;
	mp_uint_t ret;

	mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
	ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
	return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_writeblocks_obj, pyb_sdcard_writeblocks);
#endif

STATIC mp_obj_t pyb_sdcard_ioctl(mp_obj_t self, mp_obj_t cmd_in,
	mp_obj_t arg_in)
{
	mp_int_t cmd = mp_obj_get_int(cmd_in);

	switch (cmd) {
	case BP_IOCTL_INIT:
		if (!sdcard_power_on())
			return MP_OBJ_NEW_SMALL_INT(-1);
		return MP_OBJ_NEW_SMALL_INT(0);

	case BP_IOCTL_DEINIT:
		sdcard_power_off();
		return MP_OBJ_NEW_SMALL_INT(0);

	case BP_IOCTL_SYNC:
		return MP_OBJ_NEW_SMALL_INT(0);

	case BP_IOCTL_SEC_COUNT:
		return MP_OBJ_NEW_SMALL_INT(0);

	case BP_IOCTL_SEC_SIZE:
		return MP_OBJ_NEW_SMALL_INT(SDCARD_BLOCK_SIZE);

	default:
		return MP_OBJ_NEW_SMALL_INT(-1);
	}
}

STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_ioctl_obj, pyb_sdcard_ioctl);

STATIC const mp_rom_map_elem_t pyb_sdcard_locals_dict_table[] = {
	{
		MP_ROM_QSTR(MP_QSTR_present),
		MP_ROM_PTR(&sd_present_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_power),
		MP_ROM_PTR(&sd_power_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_info),
		MP_ROM_PTR(&sd_info_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_read),
		MP_ROM_PTR(&sd_read_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_write),
		MP_ROM_PTR(&sd_write_obj)
	},
#ifndef CONFIG_FAT_FILESYSTEM_ELM
	{
		MP_ROM_QSTR(MP_QSTR_readblocks),
		MP_ROM_PTR(&pyb_sdcard_readblocks_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_writeblocks),
		MP_ROM_PTR(&pyb_sdcard_writeblocks_obj)
	},
#endif
	{
		MP_ROM_QSTR(MP_QSTR_ioctl),
		MP_ROM_PTR(&pyb_sdcard_ioctl_obj)
	},
};

STATIC MP_DEFINE_CONST_DICT(pyb_sdcard_locals_dict, pyb_sdcard_locals_dict_table);

const mp_obj_type_t pyb_sdcard_type = {
	{ &mp_type_type },
	.name = MP_QSTR_SDCard,
	.make_new = pyb_sdcard_make_new,
	.locals_dict = (mp_obj_dict_t *)&pyb_sdcard_locals_dict,
};

#ifndef CONFIG_FAT_FILESYSTEM_ELM
void sdcard_init_vfs(fs_user_mount_t *vfs, int drv)
{
	vfs->base.type = &mp_fat_vfs_type;
	vfs->flags |= FSUSER_NATIVE | FSUSER_HAVE_IOCTL;
#ifdef CONFIG_FAT_FILESYSTEM_ELM
	vfs->fatfs.drv = drv;
#else
	vfs->fatfs.drv = vfs;
	vfs->fatfs.part = drv;
#endif
	vfs->readblocks[0] = (mp_obj_t)&pyb_sdcard_readblocks_obj;
	vfs->readblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
	vfs->readblocks[2] = (mp_obj_t)sdcard_read_blocks; /* native version*/
	vfs->writeblocks[0] = (mp_obj_t)&pyb_sdcard_writeblocks_obj;
	vfs->writeblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
	vfs->writeblocks[2] = (mp_obj_t)sdcard_write_blocks; /* native version*/
	vfs->u.ioctl[0] = (mp_obj_t)&pyb_sdcard_ioctl_obj;
	vfs->u.ioctl[1] = (mp_obj_t)&pyb_sdcard_obj;
}
#endif
#endif
