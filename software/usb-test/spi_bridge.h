#pragma once

#include <cstdint>
#include <iostream>
#include "hidapi.h"

// intaction with motor driver:
// SPI mode 0
// max. 4 MHz = max. 4e6 bps
// per message 12 data bits, 4 dummy bits

/** A class that encapsules the SPI interface.
* It communicates with the USB to SPI chip.
* Connection to HID (USB) is achieved using hidapi.
*/
class SPIBridge
{
public:
  SPIBridge();
  ~SPIBridge();
private:

  //! wait for HIDs to enumerate (blocking) and open first device as hidDevice_
  void initDevice();

  /** what general purpose I/O (GPIO) pins are used for*/
  enum PinDesignation
  {
    GPIO = 0x00,              ///< use pin as general purpose I/O
    ChipSelect = 0x01,        ///< use pin as SPI chip select line
    DedicatedFunction = 0x02  ///< use pin for other special function (see datasheet)
  };
  enum InterruptPinMode
  {
    countHighPulses = 4, // b100
    countLowPulses = 3,  // b011
    countRisingEdges = 2, // b010
    countFallingEdges = 1, // b001
    noInterruptCounting = 0, // b000
  };
  enum SPIMode    // mode0 is needed for motor driver
  {
    mode0 = 0x00,   //CPOL = 0, CPHA = 0, CPOL = clock polarity, 0: rising=L->H, 1: rising=H->L
    mode1 = 0x01,   //CPOL = 0, CPHA = 1, CPHA = clock phase, 0: read on rising, 1: read on falling
    mode2 = 0x02,   //CPOL = 1, CPHA = 0
    mode3 = 0x03    //CPOL = 1, CPHA = 1
  };

  /** All settings that are stored on the chip */
  struct ChipSettings
  {
    PinDesignation pinDesignation[9];   ///< mode of pin
    bool defaultOutput[9];              ///< default output value for GPIO output pin
    bool defaultDirection[9];           ///< default direction (input/output) of GPIO pin
    bool enableRemoteWakeUp;            ///< if remote wakeup is enabled
    InterruptPinMode interruptPinMode;  ///< which interrupts should be counted for GP4 pin
    bool enableSPIBusRelease;           ///< 0=SPI Bus is released between transfers, 1=SPI Bus is not released between transfers
  }
  chipSettings_;

  //! set default settings values on power-up on the chip
  void setPowerUpSettings(PinDesignation pinDesignation[9], bool defaultOutput[9], bool defaultDirection[9],
    bool enableRemoteWakeUp, InterruptPinMode interruptPinMode, bool enableSPIBusRelease);

  //! set default transfer values on power-up on the chip
  void setPowerUpTransferValues(uint32_t bitRate, bool idleChipSelect[9], bool activeChipSelect[9],
    double delayCSToData, double delayLastByteToCS, double delayDataToData, int32_t nextSPIMessageLength, SPIMode spiMode);

  //! set key parameters on power up
  void setPowerUpKeyParameters(int16_t vendorId, int16_t productId, bool hostPowered, bool remoteWakeUpCapable, int currentAmountFromHost);

  //! set the manufacturer string
  void setManufacturerName(std::string manufacturerName);

  //! set the product string
  void setProductName(std::string productName);

	hid_device *hidDevice_ = NULL;    ///< the handle of the interfaced HID of hidapi
	unsigned char buffer[1024];               ///< a binary buffer to be used for sending and receiving
};
