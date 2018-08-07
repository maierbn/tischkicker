#include "control.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

Control::Control() :
  ioextender_(*this, SPIComponent::LEDOutput, 1)    // 0 == led output, 1 = mainboard io
{
  // initialize SPI bridge
  initializeSPIBridge();

  // specifiy direction of I/O pins of mainboard I/O extender (4 LEDs, 2 switches)
  IOExtender::IODirection ioDirectionRegisterA[8] = {IOExtender::IODirection::in};
  IOExtender::IODirection ioDirectionRegisterB[8] = {IOExtender::IODirection::in};

  ioDirectionRegisterA[0] = IOExtender::IODirection::out;
  ioDirectionRegisterA[1] = IOExtender::IODirection::out;
  ioDirectionRegisterA[2] = IOExtender::IODirection::out;
  ioDirectionRegisterA[3] = IOExtender::IODirection::out;

  ioextender_.setPinIODirection(ioDirectionRegisterA, ioDirectionRegisterB);
}

void Control::initializeSPIBridge()
{
  spiBridge_.getAllSettings();

  spiBridge_.setManufacturerName(L"Benjamin Maier");
  spiBridge_.setProductName(L"I/O Extender Test");
  spiBridge_.setCurrentAmountFromHost(2000);

  // set pin designation
  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[PinGPIO0] = SPIBridge::PinDesignation::ChipSelect;
  pinDesignation[PinGPIO1] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[PinGPIO2] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[PinGPIO3] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[PinGPIO4] = SPIBridge::PinDesignation::DedicatedFunction;
  pinDesignation[PinGPIO5] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[PinGPIO6] = SPIBridge::PinDesignation::ChipSelect;
  pinDesignation[PinGPIO7] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[PinGPIO8] = SPIBridge::PinDesignation::ChipSelect;

  // set GPIO values
  bool ioValue[9] = {SPIBridge::LogicalValue::low};
  bool ioDirection[9] = {SPIBridge::GPIODirection::Output};
  for(int i=0; i<9; i++)
    ioDirection[i] = SPIBridge::GPIODirection::Output;

  bool enableRemoteWakeUp = false;
  bool enableSPIBusRelease = true;

  // store settings on chip
  spiBridge_.setSettings(true, pinDesignation, ioValue, ioDirection, enableRemoteWakeUp,
    SPIBridge::InterruptPinMode::noInterruptCounting, enableSPIBusRelease);

  // set transfer settings
  bitRate_ = IOExtender::maximumSPIBitrate();     // max. 10 MHz according to datasheet of I/O extender, max. 1 MHz according to datasheet of motor driver

  for(int i=0; i<9; i++)
  {
    idleChipSelect_[i] = SPIBridge::ChipSelectValue::CSInactive;
    activeChipSelect_[i] = SPIBridge::ChipSelectValue::CSActive;
  }

  // delays (in seconds)
  double delayCSToData = 100e-9;     //100 ns
  double delayLastByteToCS = 10e-9;
  double delayDataToData = 10e-9;

  // debugging values
  delayCSToData = 1e-3;
  delayLastByteToCS = 1e-3;
  delayDataToData = 1e-3;

  int spiTransactionLength = 2;   // 2 bytes per packet
  SPIBridge::SPIMode spiMode = SPIBridge::SPIMode::mode0;

  // store settings on chip
  spiBridge_.setTransferSettings(true, bitRate_, idleChipSelect_, activeChipSelect_,
    delayCSToData, delayLastByteToCS, delayDataToData, spiTransactionLength, spiMode);

  collectAndPrintSettings();

  std::cout<<"Configuration of SPI Bridge done."<<std::endl;
}

void Control::collectAndPrintSettings()
{
  spiBridge_.getAllSettings();
  spiBridge_.outputSettings();
}

void Control::spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length)
{
  unsigned char *spiSendBuffer = buffer;
  unsigned char spiRecvBuffer[length];

  // set settings for next spi transfer to chip
  spiBridge_.prepareSPITransfer(idleChipSelect_, activeChipSelect_, bitRate_, length);

  // transfer data
  for(int i=0; i<length; i++)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<int(spiRecvBuffer[i]);
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);
  std::cout<<" ";
  for(int i=0; i<length; i++)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<int(spiRecvBuffer[i]);
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);
  std::cout<<" ";
  for(int i=0; i<length; i++)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<int(spiRecvBuffer[i]);
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);
  std::cout<<" ";
  for(int i=0; i<length; i++)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<int(spiRecvBuffer[i]);
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);
  std::cout<<" ";
  for(int i=0; i<length; i++)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<int(spiRecvBuffer[i]);
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);
  std::cout<<" "<<std::endl;

  // copy receive buffer back to buffer
  memcpy(buffer, spiRecvBuffer, length);
}

void Control::testLEDs()
{
  std::cout<<std::endl;
  std::cout<<"======================"<<std::endl;
  std::cout<<"Control::testLEDs"<<std::endl;
  ioextender_.showChasingLight();

}
