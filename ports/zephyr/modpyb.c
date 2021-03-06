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

#include <stdint.h>
#include <stdio.h>
#include "fsl_common.h"

#include "py/mpstate.h"
#include "py/nlr.h"
#include "py/obj.h"
#include "py/gc.h"
#include "py/builtin.h"
#include "py/mphal.h"
#include "lib/utils/pyexec.h"
#include "ff.h"
#include "irq.h"
#include "led.h"
#include "sdcard.h"
#include "usb_app.h"
#include "modmachine.h"
#include "extmod/vfs.h"
#include "extmod/utime_mphal.h"
extern int pyb_hard_fault_debug;

STATIC mp_obj_t machine_freq(mp_uint_t n_args, const mp_obj_t *args)
{
	return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj, 0, 4, machine_freq);

STATIC mp_obj_t machine_sleep(void)
{
	return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_0(machine_sleep_obj, machine_sleep);

STATIC mp_obj_t machine_deepsleep(void)
{
	printk("machine.deepsleep not supported yet\n");
	return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_0(machine_deepsleep_obj, machine_deepsleep);


STATIC mp_obj_t pyb_fault_debug(mp_obj_t val)
{
	pyb_hard_fault_debug = mp_obj_is_true(val);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_fault_debug_obj, pyb_fault_debug);

/*\function elapsed_millis(start)
Returns the number of milliseconds which have elapsed since `start`.

This function takes care of counter wrap, and always returns a positive
number. This means it can be used to measure periods upto about 12.4 days.

Example:
start = pyb.millis()
while pyb.elapsed_millis(start) < 1000:
# Perform some operation*/
STATIC mp_obj_t pyb_elapsed_millis(mp_obj_t start)
{
	uint32_t startMillis = mp_obj_get_int(start);
	uint32_t currMillis = mp_hal_ticks_ms();

	return MP_OBJ_NEW_SMALL_INT((currMillis - startMillis) & 0x3fffffff);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_elapsed_millis_obj, pyb_elapsed_millis);

STATIC mp_obj_t pyb_elapsed_micros(mp_obj_t start)
{
	uint32_t startMicros = mp_obj_get_int(start);
	uint32_t currMicros = mp_hal_ticks_us();

	return MP_OBJ_NEW_SMALL_INT((currMicros - startMicros) & 0x3fffffff);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_elapsed_micros_obj, pyb_elapsed_micros);

MP_DECLARE_CONST_FUN_OBJ_KW(pyb_main_obj);

MP_DECLARE_CONST_FUN_OBJ_0(pyb_wfi_obj);
MP_DECLARE_CONST_FUN_OBJ_0(pyb_disable_irq_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_enable_irq_obj);
#if IRQ_ENABLE_STATS
MP_DECLARE_CONST_FUN_OBJ_0(pyb_irq_stats_obj);
#endif

STATIC const mp_rom_map_elem_t pyb_module_globals_table[] = {
	{
		MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pyb)
	},

	{
		MP_ROM_QSTR(MP_QSTR_fault_debug), MP_ROM_PTR(&pyb_fault_debug_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&machine_info_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&machine_freq_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_repl_info), MP_ROM_PTR(&pyb_set_repl_info_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_disable_irq), MP_ROM_PTR(&pyb_disable_irq_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_enable_irq), MP_ROM_PTR(&pyb_enable_irq_obj)
	},
#if IRQ_ENABLE_STATS
	{
		MP_ROM_QSTR(MP_QSTR_irq_stats), MP_ROM_PTR(&pyb_irq_stats_obj)
	},
#endif
	{
		MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&machine_sleep_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_standby), MP_ROM_PTR(&machine_deepsleep_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_main), MP_ROM_PTR(&pyb_main_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_usb_mode), MP_ROM_PTR(&pyb_usb_mode_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_USB_VCP), MP_ROM_PTR(&pyb_usb_vcp_type)
	},
	{
		MP_ROM_QSTR(MP_QSTR_have_cdc), MP_ROM_PTR(&pyb_have_cdc_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_millis), MP_ROM_PTR(&mp_utime_ticks_ms_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_elapsed_millis), MP_ROM_PTR(&pyb_elapsed_millis_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_micros), MP_ROM_PTR(&mp_utime_ticks_us_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_elapsed_micros), MP_ROM_PTR(&pyb_elapsed_micros_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_delay), MP_ROM_PTR(&mp_utime_sleep_ms_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_udelay), MP_ROM_PTR(&mp_utime_sleep_us_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_mount), MP_ROM_PTR(&mp_vfs_mount_obj)
	},

#if MICROPY_HW_ENABLE_RTC
	{
		MP_ROM_QSTR(MP_QSTR_RTC), MP_ROM_PTR(&pyb_rtc_type)
	},
#endif
#if MICROPY_HW_HAS_SDCARD
	{
		MP_ROM_QSTR(MP_QSTR_SD), MP_ROM_PTR(&pyb_sdcard_obj)
	},
	{
		MP_ROM_QSTR(MP_QSTR_SDCard), MP_ROM_PTR(&pyb_sdcard_type)
	},
#endif
#if MICROPY_HW_HAS_LCD
	{
		MP_ROM_QSTR(MP_QSTR_LCD), MP_ROM_PTR(&pyb_lcd_type)
	},
#endif
};

STATIC MP_DEFINE_CONST_DICT(pyb_module_globals, pyb_module_globals_table);

const mp_obj_module_t pyb_module = {
	.base = { &mp_type_module },
	.globals = (mp_obj_dict_t*)&pyb_module_globals,
};
