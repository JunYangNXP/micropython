/*
 * This file is part of the MicroPython project, http://micropython.org/
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
#ifndef MICROPY_INCLUDED_ZEPHYR_PIN_H
#define MICROPY_INCLUDED_ZEPHYR_PIN_H

#include "py/obj.h"

typedef struct {
	mp_obj_base_t base;
	qstr name;
	uint8_t idx;
	uint8_t fn;
	uint8_t unit;
	uint8_t type;
	void *reg;
} pin_af_obj_t;

typedef struct {
	mp_obj_base_t base;
	qstr name;
	uint32_t port   : 4;
	uint32_t pin    : 5;
	uint32_t num_af : 4;
	uint32_t adc_channel : 5;
	uint32_t adc_num  : 3;
	uint32_t pin_mask;
	void *gpio;
	const pin_af_obj_t *af;
} pin_obj_t;

typedef struct {
	const char *name;
	const pin_obj_t *pin;
} pin_named_pin_t;

typedef struct {
	mp_obj_base_t base;
	qstr name;
	const pin_named_pin_t *named_pins;
} pin_named_pins_obj_t;

MP_DECLARE_CONST_FUN_OBJ_KW(pin_init_obj);

static inline void pin_init0(void)
{
	return;
}
static inline uint32_t pin_get_mode(const pin_obj_t *pin)
{
	return 0;
}
static inline uint32_t pin_get_pull(const pin_obj_t *pin)
{
	return 0;
}
static inline uint32_t pin_get_af(const pin_obj_t *pin)
{
	return 0;
}
static inline const pin_obj_t *pin_find(mp_obj_t user_obj)
{
	return 0;
}
static inline const pin_obj_t *pin_find_named_pin(const mp_obj_dict_t *named_pins,
	mp_obj_t name)
{
	return 0;
}
static inline const pin_af_obj_t *pin_find_af(const pin_obj_t *pin,
	uint8_t fn, uint8_t unit)
{
	return 0;
}
static inline const pin_af_obj_t *pin_find_af_by_index(const pin_obj_t *pin,
	mp_uint_t af_idx)
{
	return 0;
}
static inline const pin_af_obj_t *pin_find_af_by_name(const pin_obj_t *pin,
	const char *name)
{
	return 0;
}

#endif
