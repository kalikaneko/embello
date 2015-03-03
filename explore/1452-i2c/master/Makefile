# This makefile reuses the serial one, but runs it in a sibling directory
include ../common/rules.mk

term:
	lpc21isp -termonly -control x /dev/ttyUSB* 115200 0
