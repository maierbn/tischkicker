CC=gcc
CPP=g++
LD=ld
LDFLAGS=-ludev
#LDFLAGS=-lpthread  -lusb-1.0
CFLAGS=-I../hidapi/hidapi -std=c++11 -I/usr/include/libusb-1.0
OUTPUT=run
C_SOURCES=hid.c
CPP_SOURCES=main.cpp spi_bridge.cpp

all:
	${CC} -c ${C_SOURCES} ${CFLAGS}
	#${CPP} -c ${CPP_SOURCES} ${CFLAGS}
	${CPP} ${CPP_SOURCES} hid.o ${CFLAGS} ${LDFLAGS} -o ${OUTPUT}
