#pragma once

#include <cstdint>
#include <iostream>
#include "hidapi.h"
#include <locale>

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
  //! set default settings values on power-up on the chip
  void setSettings(bool currentValues, PinDesignation pinDesignation[9], bool ioValue[9], bool ioDirection[9],
    bool enableRemoteWakeUp, InterruptPinMode interruptPinMode, bool enableSPIBusRelease);

  //! set default transfer values on power-up on the chip
  void setTransferSettings(bool currentValues, uint32_t bitRate, bool idleChipSelect[9], bool activeChipSelect[9],
    double delayCSToData, double delayLastByteToCS, double delayDataToData, int32_t spiTransactionLength, SPIMode spiMode);

  //! set key parameters on power up
  void setUSBKeyParameters(int16_t vendorId, int16_t productId, bool hostPowered, bool remoteWakeUpCapable, int currentAmountFromHost);

  //! set the manufacturer string
  void setManufacturerName(std::wstring manufacturerName);

  //! set the product string
  void setProductName(std::wstring productName);

  //! retrieve the power-up transfer settings from the chip and store them in currentChipSettings_
  //! @param currentValues: true=the currently used values in RAM should be retrieved,
  //!                       false=the default values in NVRAM should be retrieved
  void getTransferSettings(bool currentValues);

  //! retrieve the power-up chip settings from the chip and store them in currentChipSettings_
  void getSettings(bool currentValues);

  //! retrieve USB key parameters
  void getUSBKeyParameters();

  //! retrive the manufacturer name that is stored on the chip
  void getManufacturerName();

  //! retrive the product name that is stored on the chip
  void getProductName();

  //! set current value of all GPIO pins that are output pins, read in the value of all input pins and store in currentChipSettings_
  void setCurrentPinValue(bool ioValue[9]);

  //! retrieve all values of GPIO pins
  void getCurrentPinValue();

  //! set the direction of GPIO pins
  void setCurrentPinDirection(bool ioDirection[9]);

  //! retrieve the stored directions of GPIO pins
  void getCurrentPinDirection();

  //! perform SPI data transfer (sending and receiving), data contains data to be send and the new received data, where length is
  //! the length of the received data
  void spiTransfer(char *data, size_t &length);

  //! request chip status
  void getChipStatus();

  //! retrieve all retrievable settings from the chip
  void getAllSettings();

  //! output all currently known settings of the chip to std output
  void outputSettings();

  //! set the value of one GPIO pin
  void setGPOutputPin(int number, bool on);

private:

  //! wait for HIDs to enumerate (blocking) and open first device as hidDevice_
  void initDevice();


  /** All settings that are stored on the chip */
  struct ChipSettings
  {
    PinDesignation pinDesignation[9];   ///< the function of each general purpose pin ('GPIO', 'chip select' or 'dedicated function')
    bool ioValue[9];              ///< the default value when pin is a GPIO pin for output
    bool ioDirection[9];           ///< direction (input/output) of GPIO pin
    bool enableRemoteWakeUp;            ///< if remote wakeup is enabled (1=enabled)
    InterruptPinMode interruptPinMode;  ///< which interrupts should be counted for GP4 pin
    bool enableSPIBusRelease;           ///< 0=SPI Bus is released between transfers, 1=SPI Bus is not released between transfers

    uint32_t bitRate;                   ///< the bitrate with which SPI data is transmitted
    bool idleChipSelect[9];             ///< the configuration of Chip Select Lines when SPI bus is released
    bool activeChipSelect[9];           ///< the configuration of Chip Select Lines when SPI bus is controlled
    double delayCSToData;               ///< the delay in seconds between acquiring of SPI bus via Chip Select and begin of data transfer
    double delayLastByteToCS;           ///< the delay in seconds between end of data transfer and release of SPI bus
    double delayDataToData;             ///< the delay in seconds between subsequent data bytes
    int32_t spiTransactionLength;       ///< the number of bytes to transfer per SPI transaction
    SPIMode spiMode;                    ///< the SPI Mode
  };
  ChipSettings currentChipSettings_;    ///< the currently used chip settings that are stored on the chip in RAM
  ChipSettings powerUpChipSettings_;    ///< the default chip settings that are loaded into RAM at power up

  struct USBSettings
  {
    int16_t vendorId;                   ///< the vendor id that is stored on the chip
    int16_t productId;                  ///< the product id that is stored on the chip
    bool hostPowered;                   ///< if the chip power is drawn from the usb cable
    bool remoteWakeUpCapable;           ///< if the chip is remot wake up capable
    int currentAmountFromHost;          ///< the current amount in mA that the chip requests from the host

    std::wstring manufacturerName;      ///< the manufacturer name that is stored on the chip
    std::wstring productName;           ///< the product name that is stored on the chip
  }
  chipUSBSettings;

	hid_device *hidDevice_ = NULL;    ///< the handle of the interfaced HID of hidapi
	unsigned char buffer[1024];               ///< a binary buffer to be used for sending and receiving
};
