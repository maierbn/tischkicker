#include "control.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

Control::Control() :
  motorDriver_{
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
  },
  ledOutput_(*this, SPIComponent::LEDOutput, 0),
  mainboardIO_(*this, SPIComponent::mainboardIO, 1),
  analogDigitalConverter_({
    AnalogDigitalConverter(*this),
    AnalogDigitalConverter(*this)
  })
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

  mainboardIO_.setPinIODirection(ioDirectionRegisterA, ioDirectionRegisterB);
}

void Control::initializeSPIBridge()
{
  spiBridge_.getAllSettings();

  spiBridge_.setManufacturerName(L"Benjamin Maier");
  spiBridge_.setProductName(L"Tischkicker Control");
  spiBridge_.setCurrentAmountFromHost(500);

  // set pin designation
  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[PinA]            = SPIBridge::PinDesignation::GPIO;               // A (demux address)
  pinDesignation[PinB]            = SPIBridge::PinDesignation::GPIO;               // B (demux address)
  pinDesignation[PinC]            = SPIBridge::PinDesignation::GPIO;               // C (demux address)
  pinDesignation[PinSPI_Transfer] = SPIBridge::PinDesignation::DedicatedFunction;  // SPI_Transfer (low-active)
  pinDesignation[PinNxt1]         = SPIBridge::PinDesignation::GPIO;               // D1 (NXT 0:7)
  pinDesignation[PinNxt2]         = SPIBridge::PinDesignation::GPIO;               // D2 (NXT 8:15)
  pinDesignation[PinCSMotor1]     = SPIBridge::PinDesignation::ChipSelect;         // CS1 (chip select, line 0:7, high-active)
  pinDesignation[PinCSMotor2]     = SPIBridge::PinDesignation::ChipSelect;         // CS2 (chip select, line 8:15, high-active)
  pinDesignation[PinCSAux]        = SPIBridge::PinDesignation::ChipSelect;         // chip select Output (high-active)

  // set GPIO values
  bool ioValue[9] = {SPIBridge::LogicalValue::low};
  bool ioDirection[9] = {SPIBridge::GPIODirection::Output};
  for(int i=0; i<9; i++)
    ioDirection[i] = SPIBridge::GPIODirection::Output;

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

  // debugging values
  delayCSToData = 1;
  delayLastByteToCS = 1;
  delayDataToData = 1;

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
  if(spiComponent > BemfADCB || spiComponent < 0)
  {
    std::cout<<"Error in Control::selectChip: SPI component "<<spiComponent<<" does not exist!"<<std::endl;
    exit(-1);
  }
  //std::cout<<"select chip "<<spiComponentName[spiComponent]<<std::endl;
  if(spiComponent != currentlySelectedComponent_ || spiTransactionLength != currentSPITransferLength_)
  {
    // configure SPI bridge to select component
    for(int i=0; i<9; i++)
    {
      activeChipSelect_[i] = SPIBridge::ChipSelectValue::CSActive;
      idleChipSelect_[i] = SPIBridge::ChipSelectValue::CSInactive;
    }

    // chips that are addressed via PinCSAux
    if(spiComponent == LEDOutput || spiComponent == mainboardIO || spiComponent == BemfADCA || spiComponent == BemfADCB)
    {
      // set bit rate
      switch(spiComponent)
      {
      case LEDOutput:
      case mainboardIO:
        bitRate_ = IOExtender::maximumSPIBitrate();
        break;
      case BemfADCA:
      case BemfADCB:
        bitRate_ = AnalogDigitalConverter::maximumSPIBitrate();
        break;
      }

      // set values of chip select pins
      activeChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSActive;

      idleChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      // case direct connection via GPIO6 (CSMotor1)
      if(true)
      {
        activeChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSActive;
        activeChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
        activeChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

        idleChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
        idleChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
        idleChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;
      }

      // case jumper GPIO8-CSIO-Ex set
      if (spiComponent == mainboardIO && false)
      {
        activeChipSelect_[PinCSAux] = 1;
        idleChipSelect_[PinCSAux] = 0;
      }

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = spiTransactionLength;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect_, activeChipSelect_, bitRate_, currentSPITransferLength_);

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
      bitRate_ = MotorDriver::maximumSPIBitrate();   // 1 MHz

      // set values of chip select pins
      activeChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      activeChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;

      if(motorNumber <= 7)
        activeChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSActive;
      else
        activeChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSActive;

      activeChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      idleChipSelect_[PinCSMotor1] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect_[PinCSMotor2] = SPIBridge::ChipSelectValue::CSInactive;
      idleChipSelect_[PinCSAux] = SPIBridge::ChipSelectValue::CSInactive;

      currentlySelectedComponent_ = spiComponent;
      currentSPITransferLength_ = spiTransactionLength;

      // store settings to spi bridge
      spiBridge_.prepareSPITransfer(idleChipSelect_, activeChipSelect_, bitRate_, currentSPITransferLength_);

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
  collectAndPrintSettings();
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
  std::chrono::milliseconds timestep = std::chrono::milliseconds(1000);

  std::cout<<std::endl;
  std::cout<<"======================"<<std::endl;
  std::cout<<"Control::testLEDs"<<std::endl;

  if(0)
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
    // pin 4: NXT1
    // pin 5: NXT2
    // pin 6: CSMotor1
    // pin 7: CSMotor2
    // pin 8: CSAux

    bool valueOff[9] = {SPIBridge::LogicalValue::low};
    valueOff[PinSPI_Transfer] = SPIBridge::LogicalValue::high;
    valueOff[PinCSAux] = SPIBridge::LogicalValue::high;

    bool currentValue[9] = {SPIBridge::LogicalValue::low};

    if(0)
    {
      std::cout<<"Test 1: Switch on GPIO 0 and 1"<<std::endl;
      spiBridge_.setGPOutputPin(1, !valueOff[1]);
      spiBridge_.setGPOutputPin(0, !valueOff[0]);

      std::this_thread::sleep_for(std::chrono::milliseconds(5000));

      std::cout<<"Test 2: Switch all GPIO on "<<std::endl;
      for(int gpioNumber = 0; gpioNumber < 9; gpioNumber++)
      {
        spiBridge_.setGPOutputPinCached(gpioNumber, !valueOff[gpioNumber]);
      }
      spiBridge_.setGPOutputPinFlush();
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));

      std::cout<<"Test 3: chasing light of GPIO (2 times)"<<std::endl;
      if(1)
      for(int i=0; i<2; i++)
      {
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
      }

      std::cout<<"Test 4: count up"<<std::endl;
      // reset all values to off
      memcpy(currentValue, valueOff, 9);
      spiBridge_.setGPOutputPins(currentValue);

      for(int j=0; j<2; j++)
      for(int i=0; i<8; i++)
      {
        spiBridge_.setGPOutputPinCached(2, (i & 0x04? !valueOff[2] : valueOff[2]));   // C
        spiBridge_.setGPOutputPinCached(1, (i & 0x02? !valueOff[1] : valueOff[1]));   // B
        spiBridge_.setGPOutputPinCached(0, (i & 0x01? !valueOff[0] : valueOff[0]));   // A
        spiBridge_.setGPOutputPinFlush();

        // sleep
        std::this_thread::sleep_for(timestep);
      }
    }

    std::cout<<"initialize SPI Bridge"<<std::endl;
    initializeSPIBridge();

  }

  if(0)
  {
    std::cout<<"select chip motor"<<std::endl;
    selectChip(Motor1, 2);

    spiBridge_.getAllSettings();
    spiBridge_.outputSettings();



    std::cout<<"Test motor driver"<<std::endl;
    motorDriver_[1].debugMotor();
  }

  unsigned char buffer[2] = {false, false};    // nonsense (read WDR)
  //spiTransfer((Control::SPIComponent)(Motor0), buffer, sizeof(buffer));

  if(0)
  {
    return;
    std::cout<<"Test 5: chasing light of output"<<std::endl;
    // ----------------- chasing light of Output -------------------

    for(int motorNumber=0; motorNumber<16; motorNumber++)
    {
      // set LED of ledOutput_
      for(int j=0; j<16; j++)
      {
        ledOutput_.setOutputCached(j, motorNumber==j);
      }
      ledOutput_.applyOutputValues();

      // communicate to motor drivers to enable corresponding CS LEDs
      spiTransfer((SPIComponent)(SPIComponent::Motor0 + motorNumber), buffer, sizeof(buffer));

      // sleep
      std::this_thread::sleep_for(timestep);
    }

  }

  std::cout<<"Test 6: chasing light on mainboard"<<std::endl;
  ledOutput_.showChasingLight();
  //mainboardIO_.showChasingLight();

}

void Control::debug()
{
  motorDriver_[0].debugMotor();
}
