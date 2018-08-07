#include "motor_driver.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>

MotorDriver::MotorDriver(SPIBridge &spiBridge, int chipSelectNumber) :
  spiBridge_(spiBridge),
  chipSelectNumber_(chipSelectNumber)
{
  configureSPIBridge();
}


void MotorDriver::configureSPIBridge()
{
  std::cout<<"Configure SPI Bridge for MotorDriver "<<chipSelectNumber_<<std::endl;
  spiBridge_.getAllSettings();

  spiBridge_.setManufacturerName(L"Benni Maier");
  spiBridge_.setProductName(L"MCP2210 USB to SPI Master");

  // set pin designation and chip select line
  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[0] = SPIBridge::PinDesignation::ChipSelect;
  pinDesignation[1] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[2] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[3] = SPIBridge::PinDesignation::DedicatedFunction;   // SPI transfer LED (on=no transfer, off=transfer)
  pinDesignation[4] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[5] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[6] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[7] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[8] = SPIBridge::PinDesignation::GPIO;

  pinDesignation[chipSelectNumber_] = SPIBridge::PinDesignation::ChipSelect;

  // set GPIO values
  bool ioValue[9];
  bool ioDirection[9];
  for(int i=0; i<9; i++)
  {
    ioValue[i] = 0;
    ioDirection[i] = 0;
  }

  bool enableRemoteWakeUp = false;
  bool enableSPIBusRelease = false;
  spiBridge_.setSettings(true, pinDesignation, ioValue, ioDirection, enableRemoteWakeUp,
    SPIBridge::InterruptPinMode::noInterruptCounting, enableSPIBusRelease);

  // set transfer settings
  uint32_t bitRate = 1e6;     // max. 4 MHz
  bool idleChipSelect[9] = {true};
  bool activeChipSelect[9] = {true};

  activeChipSelect[chipSelectNumber_] = false;

  double delayCSToData = 10e-3;
  double delayLastByteToCS = 10e-3;
  double delayDataToData = 10e-3;
  int spiTransactionLength = 2;   // 12 bits per packet = 2 byte (including 4 dummy bits)
  SPIBridge::SPIMode spiMode = SPIBridge::SPIMode::mode0;

  spiBridge_.setTransferSettings(true, bitRate, idleChipSelect, activeChipSelect,
    delayCSToData, delayLastByteToCS, delayDataToData, spiTransactionLength, spiMode);

  std::cout<<"MotorDriver("<<chipSelectNumber_<<") configure SPI Bridge done."<<std::endl;
  spiBridge_.getAllSettings();
  spiBridge_.outputSettings();
}

void MotorDriver::setBufferCoilCurrent(char *buffer, int coil[2], bool mixedDecay[2])
{
  // reset buffer to 0
  buffer[0] = 0;
  buffer[1] = 0;

  // parse input parameters
  bool polarity[2];
  int currentAmount[2];
  polarity[0] = coil[0] >= 0;
  polarity[1] = coil[1] >= 0;
  currentAmount[0] = abs(coil[0]);
  currentAmount[1] = abs(coil[1]);

  if(currentAmount[0] == 0 && currentAmount[1] == 0)
  {
    std::cout<<"Request clearance of overcurrent and switch into low current standby mode."<<std::endl;
  }

  // fill buffer
  buffer[0] |= bool(mixedDecay[0]) * 0x80;              // bit 11: mixed decay phase A
  buffer[0] |= bool(currentAmount[0] & 0x08) * 0x40;    // bit 10: current A.3 (MSB)
  buffer[0] |= bool(currentAmount[0] & 0x04) * 0x20;    // bit 9: current A.2
  buffer[0] |= bool(currentAmount[0] & 0x02) * 0x10;    // bit 8: current A.1
  buffer[0] |= bool(currentAmount[0] & 0x01) * 0x08;    // bit 7: current A.0 (LSB)
  buffer[0] |= bool(!polarity[0]) * 0x04;               // bit 6: polarity bridge A (0 = current flow from OA1 to OA2)
  buffer[0] |= bool(mixedDecay[1]) * 0x02;              // bit 5: mixed decay phase B
  buffer[0] |= bool(currentAmount[1] & 0x08) * 0x01;    // bit 4: current B.3 (MSB)
  buffer[1] |= bool(currentAmount[1] & 0x04) * 0x80;    // bit 3: current B.2
  buffer[1] |= bool(currentAmount[1] & 0x02) * 0x40;    // bit 2: current B.1
  buffer[1] |= bool(currentAmount[1] & 0x01) * 0x20;    // bit 1: current B.0 (LSB)
  buffer[1] |= bool(!polarity[1]) * 0x10;               // bit 0: polarity bridge B (0 = current flow from OB1 to OB2)
  // bits 12 to 15: reserved (set to 0)
}

void MotorDriver::parseFlags(char *buffer, int length)
{
  if(length >= 2)
  {
    union
    {
      short value;
      char b[2];
    };
    b[1] = buffer[0];   // MSB is transmitted first
    b[0] = buffer[1];   // LSB is transmitted second

    std::bitset<16> bits(value);

    std::cout<<"bitset: "<<bits<<std::endl;

    for(int i=0; i<16; i++)
    {
      std::cout<<i<<"  "<<int(bits[i])<<std::endl;
    }

    bool answerValid = !bits[15] && !bits[14] && !bits[13] && bits[12];   // transfer starts with 0001
    bool overtemperature = bits[11];
    bool temperaturePrewarning = bits[10];
    bool driverUndervoltage = bits[9];
    bool overcurrentHighSide = bits[8];
    bool openLoadBridgeB = bits[7];
    bool openLoadBridgeA = bits[6];
    bool overcurrentBridgeBLowSide = bits[5];
    bool overcurrentBridgeALowSide = bits[4];

    std::cout<<"signalling flags: ";
    if(answerValid)
      std::cout<<"answerValid ";
    if(!answerValid)
      std::cout<<"!answerInvalid! ";
    if(overtemperature)
      std::cout<<"overtemperature ";
    if(temperaturePrewarning)
      std::cout<<"temperaturePrewarning ";
    if(driverUndervoltage)
      std::cout<<"driverUndervoltage ";
    if(overcurrentHighSide)
      std::cout<<"overcurrentHighSide ";
    if(overcurrentBridgeALowSide)
      std::cout<<"overcurrentBridgeALowSide ";
    if(overcurrentBridgeBLowSide)
      std::cout<<"overcurrentBridgeBLowSide ";
    if(openLoadBridgeA)
      std::cout<<"openLoadBridgeA ";
    if(openLoadBridgeB)
      std::cout<<"openLoadBridgeB ";
    std::cout<<std::endl;
  }
}

void MotorDriver::outputBuffer(char buffer[2])
{
  for(int byte=0; byte<2; byte++)
  {
    char mask = 0x80;
    std::cout<<"   MSB..LSB"<<std::endl<<byte<<"  ";
    std::cout<<std::bitset<8>(buffer[byte])
      <<" = "<<std::dec<<(unsigned int)(buffer[byte])
      <<" = 0x"<<std::hex<<(unsigned int)(buffer[byte])<<std::dec<<std::endl;
  }
}

void MotorDriver::debugMotor()
{
  std::cout<<"debugMotor"<<std::endl;

  char buffer[2];
  size_t length = 2;

  bool mixedDecay[2] = {false};
  int coil[2];

  coil[0] = 5;
  coil[1] = 0;

  // fill data
  setBufferCoilCurrent(buffer, coil, mixedDecay);

  std::cout<<"coil: "<<coil[0]<<","<<coil[1]<<", mixedDecay: "<<mixedDecay[0]<<","<<mixedDecay[1]<<std::endl;
  outputBuffer(buffer);

  spiBridge_.spiTransfer(buffer, length);

  std::cout<<"received "<<length<<" bytes."<<std::endl;
  outputBuffer(buffer);
  parseFlags(buffer, length);
}

void MotorDriver::debugChasingLight()
{
  std::cout<<std::endl;
  std::cout<<"======================"<<std::endl;
  std::cout<<"MotorDriver::debugChasingLight"<<std::endl;

  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[0] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[1] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[2] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[3] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[4] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[5] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[6] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[7] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[8] = SPIBridge::PinDesignation::GPIO;

  bool ioValue[9], ioValueOff[9];
  bool ioDirection[9];
  for(int i=0; i<9; i++)
  {
    ioValue[i] = 0;
    ioDirection[i] = 0;
    ioValueOff[i] = 0;
  }
  ioValueOff[3] = 1;
  ioValueOff[8] = 1;

  spiBridge_.setSettings(true, pinDesignation, ioValue, ioDirection, true, SPIBridge::InterruptPinMode::noInterruptCounting, false);
  spiBridge_.outputSettings();

  std::cout<<"switch all off"<<std::endl;

  for(int i=0; i<9; i++)
    spiBridge_.setGPOutputPin(i, ioValueOff[i]);



  std::cout<<"switch on 1"<<std::endl;
  spiBridge_.setGPOutputPin(1, !ioValueOff[1]);
  spiBridge_.setGPOutputPin(0, !ioValueOff[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000000));

  //exit(0);

  std::cout<<"start moving light"<<std::endl;

  // moving light
  for(int i=0; ; i++)
  {
    if(i == 9)
      i=0;
    std::cout<<"i = "<<i<<std::endl;
    for(int j=0; j<9; j++)
    {
      spiBridge_.setGPOutputPin(j, (i==j? !ioValueOff[j] : ioValueOff[j]));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }

}
