#
# This is the main Makefile, which uses MicroPython build system,
# but Zephyr arch-specific toolchain and target-specific flags.
# This Makefile builds MicroPython as a library, and then calls
# recursively Makefile.zephyr to build complete application binary
# using Zephyr build system.
#
# To build a "minimal" configuration, use "make-minimal" wrapper.

BOARD ?= mimxrt1050_evk
OMV ?= 1
TFLIGHT ?= 0
CONF_FILE = prj_$(BOARD)_merged.conf
OUTDIR_PREFIX = $(BOARD)

# Default heap size is 16KB, which is on conservative side, to let
# it build for smaller boards, but it won't be enough for larger
# applications, and will need to be increased.
MICROPY_HEAP_SIZE = 16384
FROZEN_DIR = scripts

ifeq ($(OMV), 1)
PWD = $(shell pwd)
OMV_DIR = $(OPENMV_BASE)/src/omv
OMV_QSTR_DEFS = $(OPENMV_BASE)/src/omv/py/qstrdefsomv.h
QSTR_DEFS = $(OMV_QSTR_DEFS)
MP_SRC_DIR = $(PWD)/../..
endif

# Default target
all:

include ../../py/mkenv.mk
include $(TOP)/py/py.mk
ifeq ($(OMV), 1)
include $(OPENMV_BASE)/src/omv/omv_zephyr.mk
endif

ifeq ($(TFLIGHT), 1)
include $(TFLIGHT_BASE)/tensorflow/contrib/lite/tflight_zephyr.mk
endif

# Zephyr (generated) config files - must be defined before include below
Z_EXPORTS = outdir/$(OUTDIR_PREFIX)/Makefile.export
ifneq ($(MAKECMDGOALS), clean)
include $(Z_EXPORTS)
endif

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(ZEPHYR_BASE)/net/ip
INC += -I$(ZEPHYR_BASE)/net/ip/contiki
INC += -I$(ZEPHYR_BASE)/net/ip/contiki/os

ifeq ($(OMV), 1)
SRC_USBD_MSC_C = usbd_msc_storage.c

SRC_CDC_MSC_C = $(addprefix usb_cdc_msc/,\
	usb_device_ch9.c \
	usb_device_msc.c \
	usb_device_msc_ufi.c \
	usb_device_class.c \
	usb_device_descriptor.c \
	usb_device_cdc_acm.c \
	virtual_com.c \
	ring_buffer.c \
	composite.c \
	)

SRC_USBD_C = $(addprefix usb/device/usb_device_,\
	ehci.c \
	dci.c \
	)

SRC_USBD_C += $(wildcard usb/osa/usb_osa_bm.c)

SRC_USBD_C += $(wildcard usb/phy/usb_phy.c)

SRC_SD_C += sdcard.c

SRC_USBAPP_C = usb_app.c

SRC_C_ZEPHYR = main.c \
        help.c \
        modusocket.c \
        modutime.c \
        modzephyr.c \
        modzsensor.c \
        modmachine.c \
        machine_i2c.c \
        machine_pin.c \
        modpyb.c    \
        uart_core.c \
	$(SRC_MOD)

SRC_QSTR += $(SRC_C_ZEPHYR)

MP_ZEPHYR_DRIVER_C = $(SRC_USBD_MSC_C) $(SRC_CDC_MSC_C) $(SRC_USBD_C) $(SRC_SD_C) $(SRC_USBAPP_C)

MP_ZEPHYR_DRIVER_O = $(addprefix $(BUILD)/, $(MP_ZEPHYR_DRIVER_C:.c=.o))

MP_ZEPHYR_DIRS = $(sort $(dir $(MP_ZEPHYR_DRIVER_O)))

SRC_C = $(SRC_C_ZEPHYR) $(MP_ZEPHYR_DRIVER_C)

MP_ZEPHYR_O = $(addprefix $(BUILD)/, $(SRC_C:.c=.o))

OBJ = $(PY_O) $(OMV_OBJS) $(MP_ZEPHYR_O)

else
SRC_C = main.c \
	help.c \
	modusocket.c \
	modutime.c \
	modzephyr.c \
	modzsensor.c \
	modmachine.c \
	machine_i2c.c \
	machine_pin.c \
	uart_core.c \
	lib/utils/stdout_helpers.c \
	lib/utils/printf.c \
	lib/utils/pyexec.c \
	lib/utils/interrupt_char.c \
	lib/mp-readline/readline.c \
	$(SRC_MOD)

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C)

OBJ = $(PY_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
endif

ifeq ($(TFLIGHT), 1)
OBJ += $(TFLIGHT_OBJS)
endif

CFLAGS = $(Z_CFLAGS) \
	 -std=gnu99 -fomit-frame-pointer -DNDEBUG -DMICROPY_HEAP_SIZE=$(MICROPY_HEAP_SIZE) $(CFLAGS_EXTRA) $(INC)

ifeq ($(OMV), 1)
include $(OPENMV_BASE)/src/omv/mkrules.mk
endif

ifeq ($(TFLIGHT), 1)
include $(TFLIGHT_BASE)/tensorflow/contrib/lite/mkrules.mk
endif

include $(TOP)/py/mkrules.mk

ifeq ($(OMV), 1)
$(MP_ZEPHYR_O): | $(MP_ZEPHYR_DIRS)
$(MP_ZEPHYR_DIRS):
	$(MKDIR) -p $@
endif

GENERIC_TARGETS = all zephyr run qemu qemugdb flash debug debugserver \
	ram_report rom_report
KCONFIG_TARGETS = \
	initconfig config nconfig menuconfig xconfig gconfig \
	oldconfig silentoldconfig defconfig savedefconfig \
	allnoconfig allyesconfig alldefconfig randconfig \
	listnewconfig olddefconfig
CLEAN_TARGETS = pristine mrproper

ifeq ($(OMV), 1)
ifeq ($(TFLIGHT), 1)
$(GENERIC_TARGETS): $(LIBMICROPYTHON) $(LIBOPENMV) $(LIBTFLIGHT)
else
$(GENERIC_TARGETS): $(LIBMICROPYTHON) $(LIBOPENMV)
endif
else
ifeq ($(TFLIGHT), 1)
$(GENERIC_TARGETS): $(LIBMICROPYTHON) $(LIBTFLIGHT)
else
$(GENERIC_TARGETS): $(LIBMICROPYTHON)
endif
endif

$(CLEAN_TARGETS):  clean

$(GENERIC_TARGETS) $(KCONFIG_TARGETS) $(CLEAN_TARGETS):
	$(MAKE) -C outdir/$(BOARD) $@

$(LIBMICROPYTHON): | $(Z_EXPORTS)
build/genhdr/qstr.i.last: | $(Z_EXPORTS)

# If we recreate libmicropython, also cause zephyr.bin relink
LIBMICROPYTHON_EXTRA_CMD = -$(RM) -f outdir/$(OUTDIR_PREFIX)/zephyr.lnk

# MicroPython's global clean cleans everything, fast
CLEAN_EXTRA = outdir libmicropython.a prj_*_merged.conf

# Clean Zephyr things in Zephyr way
z_clean:
	$(MAKE) -f Makefile.zephyr BOARD=$(BOARD) clean

# This rule is for prj_$(BOARD)_merged.conf, not $(CONF_FILE), which
# can be overriden.
# prj_$(BOARD).conf is optional, that's why it's resolved with $(wildcard)
# function.
prj_$(BOARD)_merged.conf: prj_base.conf $(wildcard prj_$(BOARD).conf)
	$(PYTHON) makeprj.py prj_base.conf prj_$(BOARD).conf $@

test:
	cd $(TOP)/tests && ./run-tests --target minimal --device "execpty:make -C ../ports/zephyr run BOARD=$(BOARD) QEMU_PTY=1"

cmake: outdir/$(BOARD)/Makefile

outdir/$(BOARD)/Makefile: $(CONF_FILE)
	mkdir -p outdir/$(BOARD) && cmake -DBOARD=$(BOARD) -DCONF_FILE=$(CONF_FILE) -Boutdir/$(BOARD) -H.

$(Z_EXPORTS): outdir/$(BOARD)/Makefile
	make --no-print-directory -C outdir/$(BOARD) outputexports CMAKE_COMMAND=: >$@
	make -C outdir/$(BOARD) syscall_macros_h_target syscall_list_h_target kobj_types_h_target

ifeq ($(OMV), 1)
CFLAGS += $(CFLAGS_MOD)
CFLAGS += -DARM_MATH_CM7
CFLAGS += -D__FPU_PRESENT=1
CFLAGS += -I$(OPENMV_BASE)/src/omv
CFLAGS += -I$(OPENMV_BASE)/src/omv/img
CFLAGS += -I$(OPENMV_BASE)/src/omv/nn
CFLAGS += -I$(OPENMV_BASE)/src/omv/py
CFLAGS += -I$(ZEPHYR_BASE)/ext/fs/fat/include/
CFLAGS += -I$(MP_SRC_DIR)/py
CFLAGS += -I$(MP_SRC_DIR)/lib/mp-readline
CFLAGS += -I$(PWD)/usb/include
CFLAGS:=$(filter-out -I$(ZEPHYR_BASE)/ext/hal/cmsis/Include, $(CFLAGS))
#CFLAGS -= -I$(ZEPHYR_BASE)/ext/hal/cmsis/Include
CFLAGS += -I$(OPENMV_BASE)/src/cmsis/include
CFLAGS += -DZEPHYR_BSP
CFLAGS += -DOMV_SUPPORT
CFLAGS += -DUSE_DEVICE_MODE
#workaround for openmv to include STE header
HAL_INC = '"omv_zephyr_hal.h"'
CFLAGS += -DSTM32_HAL_H=$(HAL_INC)
endif

ifeq ($(TFLIGHT), 1)
TF_CPPFLAGS = $(Z_CPPFLAGS)

$(warning TF_CPPFLAGS: $(TF_CPPFLAGS))
endif
