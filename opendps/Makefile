BINARY = opendps

# Build with commandline interface rather than serial interface
COMMANDLINE := 0

# The baudrate used for serial communications, defaults to 115200
BAUDRATE ?= 115200

# Set build model, defaults to DPS5005
MODEL := DPS5005

# Include splash screen
SPLASH_SCREEN := 0

# Build wifi version (only change being that the wifi icon will start flashing
# on power up)
WIFI := 1

# Maximum current your DPS model can provide. Eg 5000mA for the DPS5005
# This is usually set by MODEL but you can override it here
#MAX_CURRENT := 5000
# Please note that the UI currently does not handle settings larger that 9.99A

# Print debug information on the serial output
DEBUG ?= 0

# Font file
METER_FONT_FILE ?= gfx/Ubuntu-C.ttf
METER_FONT_SMALL_SIZE ?= 18
METER_FONT_MEDIUM_SIZE ?= 24
METER_FONT_LARGE_SIZE ?= 48
FULL_FONT_FILE ?= gfx/ProggyClean.ttf
FULL_FONT_SMALL_SIZE ?= 16

# Color space for the DPS display. Most units use GBR but RGB has been spotted in the wild
COLORSPACE ?= 0

# Colors for the main UI elements -- see ili9163c.h for list of colors
COLOR_VOLTAGE ?= WHITE
COLOR_AMPERAGE ?= WHITE
COLOR_INPUT ?= WHITE

# Optional tinting for UI elements
TINT ?= ffffff

# Enable CC mode
CC_ENABLE ?= 1

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags)
CFLAGS = -I. -DGIT_VERSION=\"$(GIT_VERSION)\" -Wno-missing-braces

# Output voltage and current limit are persisted in flash,
# this is the default setting
CFLAGS += \
          -DCONFIG_DEFAULT_VOUT=5000 \
          -DCONFIG_DEFAULT_ILIMIT=500 \
          -DCONFIG_BAUDRATE=$(BAUDRATE) \
          -DCOLORSPACE=$(COLORSPACE) \
          -DCOLOR_VOLTAGE=$(COLOR_VOLTAGE) \
          -DCOLOR_AMPERAGE=$(COLOR_AMPERAGE) \
          -DCOLOR_INPUT=$(COLOR_INPUT) \
          -D$(MODEL)

# Application linker script
LDSCRIPT = stm32f100_app.ld

OBJS = \
    flashlock.o \
    bootcom.o \
    crc16.o \
    uui.o \
    uui_number.o \
    func_cv.o \
    settings_calibration.o \
    hw.o \
    pwrctl.o \
    event.o \
    past.o \
    tick.o \
    tft.o \
    spi_driver.o \
    ringbuf.o \
    ili9163c.o \
    mini-printf.o \
    gfx_lookup.o \
    font-full_small.o \
    font-meter_small.o \
    font-meter_medium.o \
    font-meter_large.o \
    gfx-cc.o \
    gfx-crosshair.o \
    gfx-cv.o \
    gfx-padlock.o \
    gfx-power.o \
    gfx-thermometer.o \
    gfx-wifi.o

ifdef MAX_CURRENT
	CFLAGS +=-DCONFIG_DPS_MAX_CURRENT=$(MAX_CURRENT)
endif

ifeq ($(DEBUG),1)
	CFLAGS +=-DCONFIG_DEBUG
	OBJS += dbg_printf.o
endif

ifeq ($(CC_ENABLE),1)
	CFLAGS +=-DCONFIG_CC_ENABLE
	OBJS += func_cc.o
endif

ifeq ($(SPLASH_SCREEN),1)
	CFLAGS +=-DCONFIG_SPLASH_SCREEN
endif

ifeq ($(WIFI),1)
	CFLAGS +=-DCONFIG_WIFI
endif

ifeq ($(COMMANDLINE),1)
	CFLAGS +=-DCONFIG_COMMANDLINE
	OBJS += cli.o command_handler.o
else
	CFLAGS +=-DCONFIG_SERIAL_PROTOCOL
	OBJS += uframe.o protocol.o protocol_handler.o
endif

include ../libopencm3.target.mk

fonts:
	@python ./gen_lookup.py -l -o gfx_lookup
	@python ./gen_lookup.py -f $(FULL_FONT_FILE) -s $(FULL_FONT_SMALL_SIZE) -o full_small -a
	@python ./gen_lookup.py -f $(METER_FONT_FILE) -s $(METER_FONT_SMALL_SIZE) -o meter_small
	@python ./gen_lookup.py -f $(METER_FONT_FILE) -s $(METER_FONT_MEDIUM_SIZE) -o meter_medium
	@python ./gen_lookup.py -f $(METER_FONT_FILE) -s $(METER_FONT_LARGE_SIZE) -o meter_large

test:
	@make -C tests

doxygen:
	@doxygen config.dox
