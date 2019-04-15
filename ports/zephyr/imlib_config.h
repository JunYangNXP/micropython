/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Image library configuration.
 *
 */
#ifndef __IMLIB_CONFIG_H__
#define __IMLIB_CONFIG_H__

typedef unsigned char byte;
typedef unsigned int uint;

#define IMLIB_ENABLE_BINARY_OPS

#define IMLIB_ENABLE_MATH_OPS

#define IMLIB_ENABLE_FLOOD_FILL

#define IMLIB_ENABLE_MEAN

#define IMLIB_ENABLE_MEDIAN

#define IMLIB_ENABLE_MODE

#define IMLIB_ENABLE_MIDPOINT

#define IMLIB_ENABLE_MORPH

#define IMLIB_ENABLE_GAUSSIAN

#define IMLIB_ENABLE_LAPLACIAN

#define IMLIB_ENABLE_BILATERAL

#define IMLIB_ENABLE_CARTOON

#define IMLIB_ENABLE_REMOVE_SHADOWS

#define IMLIB_ENABLE_LINPOLAR

#define IMLIB_ENABLE_LOGPOLAR

#define IMLIB_ENABLE_CHROMINVAR

#define IMLIB_ENABLE_ILLUMINVAR

#define IMLIB_ENABLE_ROTATION_CORR

#define IMLIB_ENABLE_FIND_DISPLACEMENT

#if defined(IMLIB_ENABLE_FIND_DISPLACEMENT)\
	&& !defined(IMLIB_ENABLE_ROTATION_CORR)
#define IMLIB_ENABLE_ROTATION_CORR
#endif

#define IMLIB_ENABLE_GET_SIMILARITY

#define IMLIB_ENABLE_FIND_LINES

#define IMLIB_ENABLE_FIND_LINE_SEGMENTS

#if defined(IMLIB_ENABLE_FIND_LINE_SEGMENTS)\
	&& !defined(IMLIB_ENABLE_FIND_LINES)
#define IMLIB_ENABLE_FIND_LINES
#endif

#define IMLIB_ENABLE_FIND_CIRCLES

#define IMLIB_ENABLE_FIND_RECTS

#define IMLIB_ENABLE_QRCODES

#define IMLIB_ENABLE_APRILTAGS

#define IMLIB_ENABLE_DATAMATRICES

#define IMLIB_ENABLE_BARCODES

#define IMLIB_ENABLE_LENET

#define IMLIB_ENABLE_CNN

#define IMLIB_ENABLE_FAST

#define IMLIB_ENABLE_HOG

#define IMLIB_ENABLE_YUV_LUT

#endif
