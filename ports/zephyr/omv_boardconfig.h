/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Board configuration and pin definitions.
 *
 */
#ifndef __OMV_BOARDCONFIG_H__
#define __OMV_BOARDCONFIG_H__

// Architecture info
#define OMV_ARCH_STR            "OMV3 F7 512" // 33 chars max
#define OMV_BOARD_TYPE          "M7"
#define OMV_UNIQUE_ID_ADDR      0x1FF0F420

#define OMV_IMG_SENSOR_I2C_ADDR MT9M114_I2C_ADDR

#define OMV_JPEG_BUF_SIZE   (10 * 1024)

#define JPEG_QUALITY_THRESH     (480 * 272 * 2)

#define JPEG_QUALITY_LOW        35
#define JPEG_QUALITY_HIGH       60

#include "autoconf.h"

int printf(const char *fmt, ...);

#endif //__OMV_BOARDCONFIG_H__
