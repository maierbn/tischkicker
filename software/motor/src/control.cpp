#include "control.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

#include "easylogging++.h"

Control::Control() :
  motorDriver_{
    MotorDriver(*this,0),
  }
{
  // initialize SPI bridge
  initializeSPIBridge();
}

void Control::initializeSPIBridge()
{
  spiBridge_.getAllSettings();

  spiBridge_.setManufacturerName(L"Benjamin Maier");
  spiBridge_.setProductName(L"Tischkicker Control");
  spiBridge_.setCurrentAmountFromHost(500);

  // set pin designation
  SPIBridge::PinDesignation pinDesignation[9] = {SPIBridge::PinDesignation::GPIO};
  pinDesignation[PinCSMotor1] = SPIBridge::PinDesignation::ChipSelect;   // nCS for motor driver 1
  pinDesignation[PinNxt1]     = SPIBridge::PinDesignation::GPIO;         // NXT for motor driver 1
  
  // set GPIO values
  bool ioValue[9] = {SPIBridge::LogicalValue::low};
  bool ioDirection[9] = {SPIBridge::GPIODirection::Output};
  
  SPIChipSelectAddress_ = 0;

  bool enableRemoteWakeUp = false;
  bool enableSPIBusRelease = true;

  // store settings on chip
  spiBridge_.setSettings(true, pinDesignation, ioValue, ioDirection, enableRemoteWakeUp,
    SPIBridge::InterruptPinMode::noInterruptCounting, enableSPIBusRelease);

  // set transfer settings
  bitRate_ = MotorDriver::maximumSPIBitrate();     // max. 10 MHz according to datasheet of I/O extender, max. 1 MHz according to datasheet of motor driver

  for(int i=0; i<9; i++)
  {
    idleChipSelect_[i] = SPIBridge::ChipSelectValue::CSInactive;
    activeChipSelect_[i] = SPIBridge::ChipSelectValue::CSInactive;
  }
  currentlySelectedComponent_ = None;

  // delays (in seconds)
  double delayCSToData = 100e-9;     //100 ns
  double delayLastByteToCS = 10e-9;
  double delayDataToData = 10e-9;

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

void Control::selectChip(SPIComponent spiComponent, int spiTransactionLength)
{
  if(spiComponent > maxSpiComponent || spiComponent < 0)
  {
    std::cout<<"Error in Control::selectChip: SPI component "<<spiComponent<<" does not exist!"<<std::endl;
    exit(-1);
  }
  //std::cout<<"select chip "<<spiComponentName[spiComponent]<<std::endl;
  
  // only change settings if the chip is not already selected
  if(spiComponent != currentlySelectedComponent_ || spiTransactionLength != currentSPITransferLength_)
  {
    // configure SPI bridge to select component
    for(int i=0; i<9; i++)
    {
      activeChipSelect_[i] = SPIBridge::ChipSelectValue::CSActive;
      idleChipSelect_[i] = SPIBridge::ChipSelectValue::CSInactive;
    }

    // chips that are addressed via PinCSAux
    if(spiComponent == Motor0)
    {
      //int motorNumber = (int)spiComponent - (int)Motor0;

      // set bit rate
      bitRate_ = MotorDriver::maximumSPIBitrate();   // 1 MHz

      // set values of chip select pins
      activeChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSActive;
      //activeChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      idleChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      //idleChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = spiTransactionLength;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect_, activeChipSelect_, bitRate_, currentSPITransferLength_);
    }
  }
  //collectAndPrintSettings();
}

void Control::spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length)
{
  // select chip for SPI transfer
  selectChip(spiComponent, length);

  unsigned char *spiSendBuffer = buffer;
  unsigned char spiRecvBuffer[length];

  // set settings for next spi transfer to chip
  spiBridge_.prepareSPITransfer(idleChipSelect_, activeChipSelect_, bitRate_, length);

  // transfer data
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);

  // copy receive buffer back to buffer
  memcpy(buffer, spiRecvBuffer, length);
}

void Control::triggerNXT(int motorNumber, int count)
{
  LOG(INFO) << "triggerNXT count=" << count;
  for(int i=0; i<count; i++)
  {
    
    // set Pin to high, measure duration
    auto start = std::chrono::steady_clock::now();
    spiBridge_.setGPOutputPin(PinNxt1, SPIBridge::LogicalValue::high);
    std::chrono::microseconds duration1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start);
    

    // hold NXT high for at least 2 us
    std::this_thread::sleep_for(std::chrono::microseconds(3));
    
    // set pin to low, measure duration
    start = std::chrono::steady_clock::now();
    spiBridge_.setGPOutputPin(PinNxt1, SPIBridge::LogicalValue::low);
    std::chrono::microseconds duration2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start);
    //LOG(INFO) << "   " << duration1.count()/1000. << "ms, " << duration2.count()/1000. << "ms";
    
    if(i != count-1)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(3));
    }
  }
}

void Control::debug()
{
  motorDriver_[0].debugMotor();
}
