CC=gcc
CPP=g++
LD=ld
LDFLAGS=-ludev
#LDFLAGS=-lpthread  -lusb-1.0
CFLAGS=-I../hidapi/hidapi -std=c++14 -I/usr/include/libusb-1.0 -g
OUTPUT=run
C_SOURCES=hid.c
CPP_SOURCES=main.cpp control.cpp spi_bridge.cpp motor_driver.cpp io_extender.cpp analog_digital_converter.cpp

all:
	scons BUILD_TYPE=d
manual:
	${CC} -c ${C_SOURCES} ${CFLAGS}
	#${CPP} -c ${CPP_SOURCES} ${CFLAGS}
	${CPP} ${CPP_SOURCES} hid.o ${CFLAGS} ${LDFLAGS} -o ${OUTPUT}
