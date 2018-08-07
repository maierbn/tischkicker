#pragma once

#include <chrono>
#include <functional>

#include "spi_bridge.h"
#include "io_extender.h"
#include "motor_driver.h"
#include "analog_digital_converter.h"
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
    PinA = 0,             // DEMUX address 0
    PinB = 1,             // DEMUX address 1
    PinC = 2,             // DEMUX address 2
    PinSPI_Transfer = 3,  // output ¬SPI transfer
    PinNxt1 = 4,          // Nxt line for motor drivers 0:7
    PinNxt2 = 5,          // Nxt line for motor drivers 8:15
    PinCSMotor1 = 6,           // chip select for motor drivers 0:7
    PinCSMotor2 = 7,           // chip select for motor drivers 8:15
    PinCSAux = 8          // chip select for auxiliary components (0=I/O extender, 1:2=Bemf-DAC)
  };

  ///! set all settings on the SPI bridge for the first time
  void initializeSPIBridge();

  /// objects
  SPIBridge spiBridge_;   ///< the USB to SPI-interface driver
  MotorDriver motorDriver_[16]; ///< the motor driver objects
  IOExtender ledOutput_;   ///< the io extender driver on the output board with 16 output LEDs
  IOExtender mainboardIO_;   ///< the io extender on the main board driving 2 push buttons and 4 LEDs
  AnalogDigitalConverter analogDigitalConverter_[2];    ///< the Bemf-ADCs A and B (analogDigitalConverter_[0] and analogDigitalConverter_[1])

  /// member variables
  SPIComponent currentlySelectedComponent_ = None;   ///< the component to which spiBridge_ with current configuration can send data
  int currentSPITransferLength_ = 0;
  int SPIChipSelectAddress_ = 0;   ///< the currently set values of lines C,B,A
  uint32_t bitRate_;      ///< the bit rate with which the chip communicates over SPI

  bool idleChipSelect_[9];    ///< logical values of the chip select lines when a chip is not selected
  bool activeChipSelect_[9]; ///< logical values of the chip select lines when a chip is selected

};

