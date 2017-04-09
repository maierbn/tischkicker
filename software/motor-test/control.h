#pragma once

#include "spi_bridge.h"
#include "io_extender.h"
#include "motor_driver.h"

/** The main class that controls all components of the tischkicker
*/
class Control
{
public:
  Control();

  ///! all components with which communication via SPI is possible
  enum SPIComponent
  {
    None,
    Motor0,   ///< motor driver
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
    Motor7,
    Motor8,
    Motor9,
    Motor10,
    Motor11,
    Motor12,
    Motor13,
    Motor14,
    Motor15,
    LEDOutput   ///< output panel
  };

  ///! show a chasing light using the LED panel
  void testLEDs();

  ///! transfer data via SPI bus to one of the components
  ///! the spi bridge is configured to select the wanted component if not already done
  ///! \param spiComponent: which component to send to
  ///! \param length: bytes per packet (until SPI bus is released)
  ///! \param buffer: buffer to send, on return received data
  void spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length);

  ///! trigger rising edge on NXTi with i=motorNumber
  void triggerNXT(int motorNumber, int count=1);

  ///! debugging routine
  void debug();

private:

  ///! names for the spi bridge general purpose i/o pins (GPIO)
  enum PinName
  {
    PinA = 0,
    PinB = 1,
    PinC = 2,
    PinSPI_Transfer = 3,
    PinCS1 = 4,
    PinCS2 = 5,
    PinD1 = 6,
    PinD2 = 7,
    PinCSOutput = 8
  };

  ///! set all settings on the SPI bridge for the first time
  void initializeSPIBridge();

  /// objects
  SPIBridge spiBridge_;   ///< the USB to SPI-interface driver
  MotorDriver motorDriver_[16]; ///< the motor driver objects
  IOExtender ledOutput_;   ///< the 16 output LEDs

  /// member variables
  SPIComponent currentlySelectedComponent_ = None;   ///< the component to which spiBridge_ with current configuration can send data
  int currentSPITransferLength_ = 0;
  int selectedChannel_ = 0;   ///< the currently set values of lines C,B,A
};

