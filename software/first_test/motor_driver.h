#pragma once

#include "spi_bridge.h"
/** A class that encapsules the stepper motor driver
* which is in hardware connected to chip select line chipSelectNumber_
*/
class MotorDriver
{
public:
  MotorDriver(SPIBridge &spiBridge, int chipSelectNumber=0);

  ///! test function to check if SPI bridge hardware works correctly
  void debugChasingLight();

  ///! test function to check if motor driver chip works as expected
  void debugMotor();

private:

  ///! load all settings that are required for the SPI bridge to work with the motor driver
  void configureSPIBridge();

  ///! fill a 2-byte buffer with the values of the coil current to be send as SPI datagram
  ///! @param buffer: 2 byte buffer to be filled
  ///! @param coil: the current amount for both coils, where 0=0%, -15=min, 15=max each
  ///! @param mixedDecay: whether mixed decay feature is enabled for each coil (see datasheet)
  void setBufferCoilCurrent(char *buffer, int coil[2], bool mixedDecay[2]);

  void parseFlags(char *buffer, int length);

  void outputBuffer(char buffer[2]);

  SPIBridge &spiBridge_;   ///< the USB to SPI-interface driver
  int chipSelectNumber_;  ///< the chip select line where the motor driver IC connects to the SPI Bridge
};
