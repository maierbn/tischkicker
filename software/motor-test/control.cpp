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
  ledOutput_(*this, 0),
  mainboardIO_(*this, 1),
  analogDigitalConverter_({
    AnalogDigitalConverter(*this),
    AnalogDigitalConverter(*this)
  })
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
  pinDesignation[PinNxt1]         = SPIBridge::PinDesignation::GPIO;               // D1 (NXT 0:7)
  pinDesignation[PinNxt2]         = SPIBridge::PinDesignation::GPIO;               // D2 (NXT 8:15)
  pinDesignation[PinCSMotor1]          = SPIBridge::PinDesignation::ChipSelect;         // CS1 (chip select, line 0:7, high-active)
  pinDesignation[PinCSMotor2]          = SPIBridge::PinDesignation::ChipSelect;         // CS2 (chip select, line 8:15, high-active)
  pinDesignation[PinCSAux]        = SPIBridge::PinDesignation::ChipSelect;         // chip select Output (high-active)

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
  uint32_t bitRate = 1e6;     // max. 1 MHz according to datasheet of motor driver
  //bitRate = 1e5;
  //bitRate = 0;

  bool idleChipSelect[9] = {SPIBridge::ChipSelectValue::CSInactive};
  bool activeChipSelect[9] = {SPIBridge::ChipSelectValue::CSInactive};
  currentlySelectedComponent_ = None;

  // delays (in seconds)
  double delayCSToData = 100e-9;     //100 ns
  double delayLastByteToCS = 10e-9;
  double delayDataToData = 10e-9;

  int spiTransactionLength = 2;   // 2 bytes per packet
  SPIBridge::SPIMode spiMode = SPIBridge::SPIMode::mode0;

  // store settings on chip
  spiBridge_.setTransferSettings(true, bitRate, idleChipSelect, activeChipSelect,
    delayCSToData, delayLastByteToCS, delayDataToData, spiTransactionLength, spiMode);

  spiBridge_.getAllSettings();
  spiBridge_.outputSettings();

  std::cout<<"Configuration of SPI Bridge done."<<std::endl;
}

void Control::selectChip(SPIComponent spiComponent, int spiTransactionLength)
{
  if(spiComponent != currentlySelectedComponent_ || spiTransactionLength != currentSPITransferLength_)
  {
    // configure SPI bridge to select component
    bool activeChipSelect[9] = {SPIBridge::ChipSelectValue::CSActive};
    bool idleChipSelect[9] = {SPIBridge::ChipSelectValue::CSInactive};

    uint32_t bitRate = 1e6;      // 1 MHz

    // chips that are addressed via PinCSAux
    if(spiComponent == LEDOutput || spiComponent == mainboardIO || spiComponent == BemfADCA || spiComponent == BemfADCB)
    {
      // set bit rate
      switch(spiComponent)
      {
      case LEDOutput:
      case mainboardIO:
        bitRate = IOExtender::maximumSPIBitrate();
        break;
      case BemfADCA:
      case BemfADCB:
        bitRate = AnalogDigitalConverter::maximumSPIBitrate();
        break;
      }

      // set values of chip select pins
      activeChipSelect[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect[PinCSAux] = SPIBridge::ChipSelectValue::CSActive;

      idleChipSelect[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = spiTransactionLength;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect, activeChipSelect, bitRate, currentSPITransferLength_);

      // set values of address pins
      int newAddress = 0;
      switch(spiComponent)
      {
      case LEDOutput:
      case mainboardIO:
        newAddress = 0;
        break;
      case BemfADCA:
        newAddress = 2;    // note that the ADCs are connected this way (A=2, B=1)
        break;
      case BemfADCB:
         newAddress = 1;
        break;
      }

      if(SPIChipSelectAddress_ != newAddress)
      {
        SPIChipSelectAddress_ = newAddress;

        spiBridge_.setGPOutputPinCached(PinC, SPIChipSelectAddress_ & 0x04);
        spiBridge_.setGPOutputPinCached(PinB, SPIChipSelectAddress_ & 0x02);
        spiBridge_.setGPOutputPinCached(PinA, SPIChipSelectAddress_ & 0x01);    // A is LSB
        spiBridge_.setGPOutputPinFlush();
      }
    }
    else if(spiComponent >= Motor0 && spiComponent <= Motor15)    // motor driver chips, are addressed via PinCSMotor1 and PinCSMotor2
    {
      int motorNumber = (int)spiComponent - (int)Motor0;

      // set bit rate
      bitRate = MotorDriver::maximumSPIBitrate();   // 1 MHz

      // set values of chip select pins
      activeChipSelect[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;

      if(motorNumber <= 7)
        activeChipSelect[PinCSMotor1] = SPIBridge::ChipSelectValue::CSActive;
      else
        activeChipSelect[PinCSMotor2] = SPIBridge::ChipSelectValue::CSActive;

      activeChipSelect[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      idleChipSelect[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = spiTransactionLength;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect, activeChipSelect, bitRate, currentSPITransferLength_);

      // set values of address pins
      if(SPIChipSelectAddress_ != motorNumber % 8)
      {
        SPIChipSelectAddress_ = motorNumber % 8;

        spiBridge_.setGPOutputPinCached(PinC, SPIChipSelectAddress_ & 0x04);
        spiBridge_.setGPOutputPinCached(PinB, SPIChipSelectAddress_ & 0x02);
        spiBridge_.setGPOutputPinCached(PinA, SPIChipSelectAddress_ & 0x01);    // A is LSB
        spiBridge_.setGPOutputPinFlush();
      }
    }
    else
    {
      std::cout<<"Error in Control::spiTransfer: invalid component"<<std::endl;
      return;
    }
  }
}

void Control::spiTransfer(SPIComponent spiComponent, unsigned char *buffer, int length)
{
  // select chip for SPI transfer
  selectChip(spiComponent, length);

  unsigned char *spiSendBuffer = buffer;
  unsigned char spiRecvBuffer[length];

  // transfer data
  spiBridge_.spiTransfer(spiSendBuffer, spiRecvBuffer, length);

  // copy receive buffer back to buffer
  memcpy(buffer, spiRecvBuffer, length);
}

void Control::triggerNXT(int motorNumber, int count)
{
  // set values of address pins
  if(SPIChipSelectAddress_ != motorNumber % 8)
  {
    SPIChipSelectAddress_ = motorNumber % 8;

    spiBridge_.setGPOutputPinCached(PinC, SPIChipSelectAddress_ & 0x04);
    spiBridge_.setGPOutputPinCached(PinB, SPIChipSelectAddress_ & 0x02);
    spiBridge_.setGPOutputPinCached(PinA, SPIChipSelectAddress_ & 0x01);    // A is LSB
    spiBridge_.setGPOutputPinFlush();
  }

  if(motorNumber < 8)
  {
    for(int i=0; i<count; i++)
    {
      spiBridge_.setGPOutputPin(PinNxt1, SPIBridge::LogicalValue::high);

      // hold NXT high for at least 2 us
      std::this_thread::sleep_for(std::chrono::microseconds(2));
      spiBridge_.setGPOutputPin(PinNxt1, SPIBridge::LogicalValue::low);
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
      spiBridge_.setGPOutputPin(PinNxt2, SPIBridge::LogicalValue::high);
      // hold NXT high for at least 2 us
      std::this_thread::sleep_for(std::chrono::microseconds(2));
      spiBridge_.setGPOutputPin(PinNxt2, SPIBridge::LogicalValue::low);
      if(i != count-1)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(2));
      }
    }
  }
}

void Control::testLEDs()
{
  std::chrono::milliseconds timestep = std::chrono::milliseconds(200);

  std::cout<<"in control::testLEDs"<<std::endl;
  return;
  mainboardIO_.showChasingLight();

  return;

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
  // pin 4: NXT1
  // pin 5: NXT2
  // pin 6: CSMotor1
  // pin 7: CSMotor2
  // pin 8: CSAux

  bool valueOff[9] = {SPIBridge::LogicalValue::low};
  valueOff[PinSPI_Transfer] = SPIBridge::LogicalValue::high;

  bool currentValue[9] = {SPIBridge::LogicalValue::low};

  for(int gpioNumber=0; gpioNumber<9; gpioNumber++)
  {
    // reset all values to off
    memcpy(currentValue, valueOff, 9);

    // set one LED to on
    currentValue[gpioNumber] = !currentValue[gpioNumber];

    spiBridge_.setGPOutputPins(currentValue);

    // sleep
    std::this_thread::sleep_for(timestep);
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
    std::this_thread::sleep_for(timestep);
  }
}

void Control::debug()
{
  motorDriver_[0].debugMotor();
}
