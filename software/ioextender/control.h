#pragma once

#include <chrono>
#include <functional>

#include "spi_bridge.h"
#include "io_extender.h"
#include "spi_component.h"

/** The main class that controls all components
*/
class Control
{
public:
  Control();

  ///! show a chasing light using the LED panel
  void testLEDs();

  ///! transfer data via SPI bus to one of the components
  ///! the spi bridge is configured to select the wanted component if not already done by calling selectChip
  ///! \param spiComponent: which component to send to
  ///! \param length: bytes per packet (until SPI bus is released)
  ///! \param buffer: buffer to send, on return received data
  void spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length);

  ///! collects current settings on SPIBridge and print them
  void collectAndPrintSettings();

private:

  ///! names for the spi bridge general purpose i/o pins (GPIO)
  enum PinName
  {
    PinGPIO0 = 0,
    PinGPIO1 = 1,
    PinGPIO2 = 2,
    PinGPIO3 = 3,
    PinGPIO4 = 4,
    PinGPIO5 = 5,
    PinGPIO6 = 6,
    PinGPIO7 = 7,
    PinGPIO8 = 8
  };

  ///! set all settings on the SPI bridge for the first time
  void initializeSPIBridge();

  /// objects
  SPIBridge spiBridge_;   ///< the USB to SPI-interface driver
  IOExtender ioextender_;   ///< the io extender driver on the output board with 16 output LEDs

  /// member variables
  int currentSPITransferLength_ = 0;
  uint32_t bitRate_;      ///< the bit rate with which the chip communicates over SPI

  bool idleChipSelect_[9];    ///< logical values of the chip select lines when a chip is not selected
  bool activeChipSelect_[9]; ///< logical values of the chip select lines when a chip is selected

};

