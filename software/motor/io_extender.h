#pragma once

#include <iostream>
#include "spi_component.h"

class Control;

/** A class that gives access to the MCP23S17 input/output extension
*/
class IOExtender
{
public:
  IOExtender(Control &control, SPIComponent spiComponent, int hardwareAddress=0);

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

  ///! read all input values in GPIO A0:8 and B0:8, return as 16 bits
  uint16_t readInputValues(bool debug=false);

  ///! returns the maximum bitrate that is possible for SPI communication
  static uint32_t maximumSPIBitrate();

  void debug();

  ///! retrieves all current register values from the IC and writes them to std output
  void outputAllSettings(bool withLegend=true);

  ///! if the chip is not yet configured, run configure
  void assertConfigured();

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

  enum IODirection
  {
    out = 0,
    in = 1
  };

  ///! modify the on-chip configuration
  void configure(bool debug=false);

  ///! read the value of a register
  unsigned char readRegister(RegisterAddress registerAddress, bool debug=false);

  ///! write a value to a register
  void writeRegister(RegisterAddress registerAddress, unsigned char value, bool debug=false);

  bool currentOutput_[16];  ///< current state of the LEDs (true=on, false=off)

  Control &control_;   ///< the control object that has access to all components
  SPIComponent spiComponent_;	///< the SPI component of control
  int hardwareAddress_;    ///< the hardware address of the chip, given by the externally biased A2,A1 and A0 pins
  unsigned char deviceOpcode_;    ///< the first byte to send in a SPI transfer
  bool configured_;         ///< if the settings were initially stored on the chip, this has to be done after the constructor, but before use of the chip

  // define I/O direction
  IODirection ioDirectionA_[8];
  IODirection ioDirectionB_[8];

};

