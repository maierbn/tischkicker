#include "control.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

Control::Control() :
  motorDriver_({
    MotorDriver(*this,0),
    MotorDriver(*this,1),
    MotorDriver(*this,2),
    MotorDriver(*this,3),
    MotorDriver(*this,4),
    MotorDriver(*this,5),
    MotorDriver(*this,6),
    MotorDriver(*this,7),
    MotorDriver(*this,8),
    MotorDriver(*this,9),
    MotorDriver(*this,10),
    MotorDriver(*this,11),
    MotorDriver(*this,12),
    MotorDriver(*this,13),
    MotorDriver(*this,14),
    MotorDriver(*this,15)
  }),
  ledOutput_(*this)
{
  initializeSPIBridge();
}

void Control::initializeSPIBridge()
{
  spiBridge_.getAllSettings();

  spiBridge_.setManufacturerName(L"Benni Maier");
  spiBridge_.setProductName(L"Tischkicker Motor Control Panel");

  // set pin designation
  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[PinA]            = SPIBridge::PinDesignation::GPIO;               // A (demux address)
  pinDesignation[PinB]            = SPIBridge::PinDesignation::GPIO;               // B (demux address)
  pinDesignation[PinC]            = SPIBridge::PinDesignation::GPIO;               // C (demux address)
  pinDesignation[PinSPI_Transfer] = SPIBridge::PinDesignation::DedicatedFunction;  // SPI_Transfer (low-active)
  pinDesignation[PinCS1]          = SPIBridge::PinDesignation::ChipSelect;         // CS1 (chip select, line 0:7, high-active)
  pinDesignation[PinCS2]          = SPIBridge::PinDesignation::ChipSelect;         // CS2 (chip select, line 8:15, high-active)
  pinDesignation[PinD1]           = SPIBridge::PinDesignation::GPIO;               // D1 (NXT 0:7)
  pinDesignation[PinD2]           = SPIBridge::PinDesignation::GPIO;               // D2 (NXT 8:15)
  pinDesignation[PinCSOutput]     = SPIBridge::PinDesignation::ChipSelect;         // chip select Output (low-active)

  // set GPIO values
  bool ioValue[9] = {false};
  bool ioDirection[9] = {SPIBridge::GPIODirection::Output};

  for(int i=0; i<9; i++)
  {
    ioValue[i] = SPIBridge::LogicalValue::low;
    ioDirection[i] = SPIBridge::GPIODirection::Output;
  }

  bool enableRemoteWakeUp = false;
  bool enableSPIBusRelease = true;
  spiBridge_.setSettings(true, pinDesignation, ioValue, ioDirection, enableRemoteWakeUp,
    SPIBridge::InterruptPinMode::noInterruptCounting, enableSPIBusRelease);

  // set transfer settings
  uint32_t bitRate = 1e6;     // max. 1 MHz according to datasheet of motor driver
  //bitRate = 1e5;
  //bitRate = 0;

  bool idleChipSelect[9] = {SPIBridge::LogicalValue::high};      // status of ChipSelect line when SPI bus is idle: high
  bool activeChipSelect[9] = {SPIBridge::LogicalValue::low};    // status of ChipSelect line when SPI transfer: low

  for (int i=0; i<9; i++)
  {
    idleChipSelect[i] = SPIBridge::LogicalValue::low;
    activeChipSelect[i] = SPIBridge::LogicalValue::high;
  }

  // delays (in seconds)
  double delayCSToData = 100e-9;     //100 ns
  double delayLastByteToCS = 10e-9;
  double delayDataToData = 10e-9;

  int spiTransactionLength = 2;   // 2 bytes per packet
  SPIBridge::SPIMode spiMode = SPIBridge::SPIMode::mode0;

  spiBridge_.setTransferSettings(true, bitRate, idleChipSelect, activeChipSelect,
    delayCSToData, delayLastByteToCS, delayDataToData, spiTransactionLength, spiMode);

  spiBridge_.getAllSettings();
  spiBridge_.outputSettings();
  std::cout<<"Configuration of SPI Bridge done."<<std::endl;
}

void Control::spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length)
{
  if(spiComponent != currentlySelectedComponent_ || length != currentSPITransferLength_)
  {
    // configure SPI bridge to select component
    bool idleChipSelect[9] = {SPIBridge::LogicalValue::high};
    bool activeChipSelect[9] = {SPIBridge::LogicalValue::low};

    //CS1: high-active
    //CS2: high-active
    //CSOutput: low-active

    uint32_t bitRate = 1e6;      // 1 MHz
    int spiTransactionLength = length;

    if(spiComponent == LEDOutput)
    {
      // set bit rate
      bitRate = 10e6;   // 10 MHz

      // set values of chip select pins
      idleChipSelect[PinCS1] = SPIBridge::LogicalValue::low;
      idleChipSelect[PinCS2] = SPIBridge::LogicalValue::low;
      idleChipSelect[PinCSOutput] = SPIBridge::LogicalValue::high;

      activeChipSelect[PinCS1] = SPIBridge::LogicalValue::low;
      activeChipSelect[PinCS2] = SPIBridge::LogicalValue::low;
      activeChipSelect[PinCSOutput] = SPIBridge::LogicalValue::low;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = length;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect, activeChipSelect, bitRate, currentSPITransferLength_);
    }
    else if(spiComponent >= Motor0 && spiComponent <= Motor15)
    {
      int motorNumber = (int)spiComponent - (int)Motor0;

      // set bit rate
      bitRate = 1e6;   // 1 MHz

      // set values of chip select pins
      idleChipSelect[PinCS1] = SPIBridge::LogicalValue::low;
      idleChipSelect[PinCS2] = SPIBridge::LogicalValue::low;

      if(motorNumber <= 7)
      {
        activeChipSelect[PinCS1] = SPIBridge::LogicalValue::high;
        activeChipSelect[PinCS2] = SPIBridge::LogicalValue::low;
      }
      else
      {
        activeChipSelect[PinCS1] = SPIBridge::LogicalValue::low;
        activeChipSelect[PinCS2] = SPIBridge::LogicalValue::high;
      }

      idleChipSelect[PinCSOutput] = SPIBridge::LogicalValue::high;
      activeChipSelect[PinCSOutput] = SPIBridge::LogicalValue::high;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = length;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect, activeChipSelect, bitRate, currentSPITransferLength_);

      // set values of output pins
      if(selectedChannel_ != motorNumber % 8)
      {
        spiBridge_.setGPOutputPinCached(PinA, motorNumber & 0x01);
        spiBridge_.setGPOutputPinCached(PinB, motorNumber & 0x02);
        spiBridge_.setGPOutputPinCached(PinC, motorNumber & 0x04);
        selectedChannel_ = motorNumber % 8;
        spiBridge_.setGPOutputPinFlush();
      }
    }
    else
    {
      std::cout<<"Error in Control::spiTransfer: invalid component"<<std::endl;
      return;
    }
  }

  unsigned char *spiSendBuffer = buffer;
  unsigned char spiRecvBuffer[length];

  // transfer data
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);

  // copy receive buffer back to buffer
  memcpy(buffer, spiRecvBuffer, length);
}

void Control::triggerNXT(int motorNumber, int count)
{
  if(selectedChannel_ != motorNumber % 8)
  {
    // set values of output pins
    spiBridge_.setGPOutputPinCached(PinA, motorNumber & 0x01);
    spiBridge_.setGPOutputPinCached(PinB, motorNumber & 0x02);
    spiBridge_.setGPOutputPinCached(PinC, motorNumber & 0x04);
    spiBridge_.setGPOutputPinFlush();
    selectedChannel_ = motorNumber % 8;
  }

  if(motorNumber < 8)
  {
    for(int i=0; i<count; i++)
    {
      spiBridge_.setGPOutputPin(PinD1, true);
      // hold NXT high for at least 2 us
      std::this_thread::sleep_for(std::chrono::microseconds(2));
      spiBridge_.setGPOutputPin(PinD1, false);
      if(i != count-1)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(2));
      }
    }
  }
  else
  {
    for(int i=0; i<count; i++)
    {
      spiBridge_.setGPOutputPin(PinD2, true);
      // hold NXT high for at least 2 us
      std::this_thread::sleep_for(std::chrono::microseconds(2));
      spiBridge_.setGPOutputPin(PinD2, false);
      if(i != count-1)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(2));
      }
    }
  }
}

void Control::testLEDs()
{
  // ----------------- chasing light of GPIO ----------------------
  // change pin designation to all GPIO pins to show LEDs
  SPIBridge::PinDesignation pinDesignation[9];
  for(int i=0; i<9; i++)
  {
    pinDesignation[i] = SPIBridge::PinDesignation::GPIO;
  }
  spiBridge_.setPinDesignation(pinDesignation);

  // pin 0: A
  // pin 1: B
  // pin 2: C
  // pin 3: SPI_Transfer (inverted)
  // pin 4: CS1
  // pin 5: CS2
  // pin 6: D1
  // pin 7: D2
  // pin 8: PinCSOutput

  bool valueOff[9];
  bool currentValue[9];

  for(int i=0; i<9; i++)
  {
    valueOff[i] = false;
  }
  valueOff[PinSPI_Transfer] = true;

  for(int gpioNumber=0; gpioNumber<9; gpioNumber++)
  {
    // reset all values to off
    memcpy(currentValue, valueOff, 9);

    // set one LED to on
    currentValue[gpioNumber] = !currentValue[gpioNumber];

    spiBridge_.setGPOutputPins(currentValue);

    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  initializeSPIBridge();
  // ----------------- chasing light of Output -------------------

  unsigned char buffer[2] = {false, false};    // nonsense (read WDR)
  for(int motorNumber=0; motorNumber<16; motorNumber++)
  {
    // set LED of ledOutput_
    for(int j=0; j<16; j++)
    {
      ledOutput_.setOutputCached(j, motorNumber==j);
    }
    ledOutput_.applyOutputValues();

    // communicate to motor drivers to enable corresponding CS LEDs
    spiTransfer((Control::SPIComponent)(Motor0+motorNumber), buffer, sizeof(buffer));

    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void Control::debug()
{
  motorDriver_[0].debugMotor();
}
