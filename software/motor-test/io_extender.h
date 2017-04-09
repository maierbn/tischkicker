#pragma once

#include <iostream>

class Control;

/** A class that gives access to the MCP23S17 input/output extension
*/
class IOExtender
{
public:
  IOExtender(Control &control, int hardwareAddress=0);

  ///! drive a chasing light
  void showChasingLight();

  ///! set the value of a single output led
  void setOutput(int ledNumber, bool on);

  ///! set the value of all output leds
  void setOutput(bool on[16]);

  ///! save the value of LED with no. ledNumber to on in cache, applied after call to applyOutputValues
  void setOutputCached(int ledNumber, bool on);

  ///! set the values of the LEDs to those previously set by setOutputCached
  void applyOutputValues();

private:

  enum RegisterAddress
  {
    IODIRA = 0x00,    // direction
    IODIRB = 0x01,    // direction
    IPOLA = 0x02,     // polarity
    IPOLB = 0x03,     // polarity
    GPINTENA = 0x04,  // interrupt enable
    GPINTENB = 0x05,  // interrupt enable
    DEFVALA = 0x06,   // interrupt compare default value
    DEFVALB = 0x07,   // interrupt compare default value
    INTCONA = 0x08,   // interrupt control
    INTCONB = 0x09,   // interrupt control
    IOCON = 0x0A,     // configuration
    // 0x0B is also mapped to IOCON
    GPPUA = 0x0C,     // pull-up resistor settings
    GPPUB = 0x0D,     // pull-up resistor settings
    INTFA = 0x0E,     // interrupt flag
    INTFB = 0x0F,     // interrupt flag
    INTCAPA = 0x10,   // interrupt capture
    INTCAPB = 0x11,   // interrupt capture
    GPIOA = 0x12,     // port register
    GPIOB = 0x13,     // port register
    OLATA = 0x14,     // output latch register
    OLATB = 0x15,     // output latch register
  };

  const std::string registerAddressName[22]
    = {"IODIRA", "IODIRB", "IPOLA", "IPOLB", "GPINTENA", "GPINTENB", "DEFVALA", "DEFVALB", "INTCONA", "INTCONB",
     "IOCON", "IOCON", "GPPUA", "GPPUB", "INTFA", "INTFB", "INTCAPA", "INTCAPB", "GPIOA", "GPIOB", "OLATA", "OLATB"};

  enum RegisterCommand
  {
    read = 0x01,    // Read command mask:  0000 0001
    write = 0x00,   // Write command mask: 0000 0000
  };

  ///! modify the on-chip configuration
  void configure();

  bool currentOutput_[16];  ///< current state of the LEDs (true=on, false=off)

  Control &control_;   ///< the control object that has access to all components
  int hardwareAddress_;    ///< the hardware address of the chip, given by the externally biased A2,A1 and A0 pins
  unsigned char deviceOpcode_;    ///< the first byte to send in a SPI transfer
};

