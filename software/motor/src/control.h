#pragma once

#include <chrono>
#include <functional>

#include "boards/spi_bridge.h"
#include "boards/io_extender.h"
#include "boards/motor_driver.h"
#include "boards/analog_digital_converter.h"
#include "spi_component.h"


/** The main class that controls all components
*/
class Control
{
public:
  Control();

  ///! show a chasing light using the LED panel
  void testLEDs();

  ///! selects the requested chip for SPI transfer and sets the spiTransactionLength
  ///! this includes setting the correct DEMUX address and configuring the SPI bridge to enable the correct CS lines
  ///! if everything is already set up, nothing is done
  void selectChip(SPIComponent spiComponent, int spiTransactionLength);

  ///! transfer data via SPI bus to one of the components
  ///! the spi bridge is configured to select the wanted component if not already done by calling selectChip
  ///! \param spiComponent: which component to send to
  ///! \param length: bytes per packet (until SPI bus is released)
  ///! \param buffer: buffer to send, on return received data
  void spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length);

  ///! trigger rising edge on NXTi with i=motorNumber
  void triggerNXT(int motorNumber, int count=1);

  ///! debugging routine
  void debug();

  ///! collects current settings on SPIBridge and print them
  void collectAndPrintSettings();

private:

  ///! names for the spi bridge general purpose i/o pins (GPIO)
  enum PinName
  {
    PinCSMotor1 = 0,      // nCS for motor 1
    PinNxt1 = 1,          // NXT for motor 1
  };

  ///! set all settings on the SPI bridge for the first time
  void initializeSPIBridge();

  /// objects
  SPIBridge spiBridge_;   ///< the USB to SPI-interface driver
  MotorDriver motorDriver_[1]; ///< the motor driver objects
  
  /// member variables
  SPIComponent currentlySelectedComponent_ = None;   ///< the component to which spiBridge_ with current configuration can send data
  int currentSPITransferLength_ = 0;
  int SPIChipSelectAddress_ = 0;   ///< the currently set values of lines C,B,A
  uint32_t bitRate_;      ///< the bit rate with which the chip communicates over SPI

  bool idleChipSelect_[9];    ///< logical values of the chip select lines when a chip is not selected
  bool activeChipSelect_[9]; ///< logical values of the chip select lines when a chip is selected

};

