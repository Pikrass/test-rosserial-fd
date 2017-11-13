ARDUINO_DIR   = /usr/share/arduino
AVR_TOOLS_DIR = /usr
AVRDUDE_CONF  = /usr/share/arduino/hardware/tools/avrdude.conf

BOARD_TAG = mega2560
ARDUINO_SKETCHBOOK = $(realpath ..)
USER_LIB_PATH = $(realpath ..)
ARDUINO_LIBS = ros_lib \

DEBUG = 1
OPTIMIZATION_LEVEL = 0 -gdwarf-4

-include ./overrides.mk
include ../makefiles/Arduino.mk
