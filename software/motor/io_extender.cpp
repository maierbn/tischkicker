#include "io_extender.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

#include "control.h"
#include "spi_component.h"

IOExtender::IOExtender(Control &control, SPIComponent spiComponent, int hardwareAddress) :
  control_(control),
  spiComponent_(spiComponent_),
  hardwareAddress_(hardwareAddress)
{
  if(hardwareAddress_ < 0 || hardwareAddress_ >= 8)
  {
    std::cout<<"Error in IOExtender(): hardwareAddress "<<hardwareAddress_<<" is invalid!"<<std::endl;
    return;
  }

  // initialize 8 bit device Opcode: 0 1 0 0 A2 A1 A0 R/W   (R/W=0=write, R/W=1=read)
  deviceOpcode_ = 0x40 | (hardwareAddress_ << 1);

  // define I/O direction
  for(int i=0; i<8; i++)
  {
    ioDirectionA_[i] = IODirection::in;
    ioDirectionB_[i] = IODirection::in;
  }

  // initialize current output to false
  for(int i=0; i<16; i++)
    currentOutput_[i] = false;

  configured_ = false;
}

void IOExtender::configure(bool debug)
{
  configured_ = true;
  unsigned char value;

  // false: register addresses are interleaved (A and B after each other)
  // true: registers associated with each port are separated
  bool bank = false;

  // true: INT pins are internally connected
  bool mirror = false;

  // true: byte mode, address pointer toggles between both values (A and B) of a pair (assuming BANK=0)
  // false: sequential mode, address pointer increments after each byte during data transfer
  bool seqop = true;

  // true: enable slew rate control for I2C connection (not used)
  bool disslw = false;

  // true: enable hardware address bit
  bool haen = true;

  // INT pin, true: open-drain output (behaves like switch to GND, 0=connected to GND, 1=NC), false: active driver output (with polarity INTPOL)
  bool odr = true;

  // true: active-high, false: active-low
  bool intpol = false;

  // data byte
  value = (bank? 0x80: 0) | (mirror? 0x40: 0) | (seqop? 0x20: 0) | (disslw? 0x10: 0)
   | (haen? 0x08: 0) | (odr? 0x04: 0) | (intpol? 0x02: 0);

  writeRegister(RegisterAddress::IOCON, value, debug);

  // set I/O direction of register A
  value = 0;
  unsigned char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    value |= (ioDirectionA_[i]? mask : 0x00);
    mask <<= 1;
  }

  writeRegister(RegisterAddress::IODIRA, value, debug);

  // set I/O direction of register B
  value = 0;
  mask = 0x01;
  for(int i=0; i<8; i++)
  {
    value |= (ioDirectionB_[i]? mask : 0x00);
    mask <<= 1;
  }
  writeRegister(RegisterAddress::IODIRB, value, debug);

  // enable interrupts for all inputs
  value = 0xff;
  writeRegister(RegisterAddress::GPINTENA, value, debug);
  writeRegister(RegisterAddress::GPINTENB, value, debug);
  // INTCON = 0 (default value) = compare against previous pin value

  // set compare configuration to compare to default value (DEFVAL)
  writeRegister(RegisterAddress::INTCONA, value, debug);
  writeRegister(RegisterAddress::INTCONB, value, debug);

  // set default value to 1
  value = 0xff;
  writeRegister(RegisterAddress::DEFVALA, value, debug);
  writeRegister(RegisterAddress::DEFVALB, value, debug);

  std::cout<<"Configuration of I/O-Extender ("<<hardwareAddress_<<") done."<<std::endl;
}

///! read the value of a register
unsigned char IOExtender::readRegister(RegisterAddress registerAddress, bool debug)
{
  if(!configured_)
    configure();

  unsigned char buffer[3];

  // chip select byte and command
  buffer[0] = deviceOpcode_ | RegisterCommand::read;

  // register select byte
  buffer[1] = registerAddress;

  // data byte
  buffer[2] = 0x00; // dummy value

  control_.spiTransfer(spiComponent_, buffer, sizeof(buffer));

  if(debug)
  {
    std::cout<<"readRegister("<<registerAddressName[registerAddress]<<"): "<<std::hex<<std::setfill('0')<<std::setw(2)
      <<int(buffer[2])<<std::dec<<std::endl;
  }

  // return the last returned value
  return buffer[2];
}

///! write a value to a register
void IOExtender::writeRegister(RegisterAddress registerAddress, unsigned char value, bool debug)
{
  if(!configured_)
    configure();

  unsigned char buffer[3];

  // chip select byte and command
  buffer[0] = deviceOpcode_ | RegisterCommand::write;

  // register select byte
  buffer[1] = registerAddress;

  // data byte
  buffer[2] = value;

  control_.spiTransfer(spiComponent_, buffer, sizeof(buffer));

  if(debug)
  {
    std::cout<<"writeRegister("<<registerAddressName[registerAddress]<<"): "<<std::hex<<std::setfill('0')<<std::setw(2)<<int(value)
      <<", old value: "<<std::setfill('0')<<std::setw(2)<<int(buffer[2])<<std::dec<<std::endl;
    std::cout<<"for debugging, read same register:"<<std::endl;
    readRegister(registerAddress, true);
  }
}

void IOExtender::outputAllSettings(bool withLegend)
{
  if(!configured_)
    configure();

  unsigned int valueA = readRegister(RegisterAddress::IODIRA);
  unsigned int valueB = readRegister(RegisterAddress::IODIRB);
  std::cout<<"        A   B    A        B"<<std::endl<<std::hex;
  std::cout<<"IODIR   "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::IPOLA);
  valueB = readRegister(RegisterAddress::IPOLB);
  std::cout<<"IPOL    "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::GPINTENA);
  valueB = readRegister(RegisterAddress::GPINTENB);
  std::cout<<"GPINTEN "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::DEFVALA);
  valueB = readRegister(RegisterAddress::DEFVALB);
  std::cout<<"DEFVAL  "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::INTCONA);
  valueB = readRegister(RegisterAddress::INTCONB);
  std::cout<<"INTCON  "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::IOCON);
  std::cout<<"IOCON   "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"     "
    <<"  BANK="<<(valueA&0x80?"1":"0")
    <<", MIRROR="<<(valueA&0x40?"1":"0")
    <<", SEQOP="<<(valueA&0x20?"1":"0")
    <<", DISSLW="<<(valueA&0x10?"1":"0")
    <<", HAEN="<<(valueA&0x08?"1":"0")
    <<", ODR="<<(valueA&0x04?"1":"0")
    <<", INTPOL="<<(valueA&0x02?"1":"0")
    <<std::endl;

  valueA = readRegister(RegisterAddress::GPPUA);
  valueB = readRegister(RegisterAddress::GPPUB);
  std::cout<<"GPPU    "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::INTFA);
  valueB = readRegister(RegisterAddress::INTFB);
  std::cout<<"INTF    "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::INTCAPA);
  valueB = readRegister(RegisterAddress::INTCAPB);
  std::cout<<"INTCAP  "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::GPIOA);
  valueB = readRegister(RegisterAddress::GPIOB);
  std::cout<<"GPIO    "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  valueA = readRegister(RegisterAddress::OLATA);
  valueB = readRegister(RegisterAddress::OLATB);
  std::cout<<"OLAT    "<<std::setfill('0')<<std::setw(2)<<int(valueA)<<"  "<<std::setfill('0')<<std::setw(2)<<int(valueB)
    <<" = "<<std::bitset<8>(valueA)<<" "<<std::bitset<8>(valueB)<<std::endl;

  if(withLegend)
  {
    std::cout<<"Legend: "<<std::endl
      <<"  IODIR:   I/O direction, 0=output, 1=input"<<std::endl
      <<"  IPOL:    Input polarity, 0=register=input pin, 1=register=not input pin"<<std::endl
      <<"  GPINTEN: GP interrupt-on-change, 1=pin is enabled for interrupt-on-change events"<<std::endl
      <<"  DEFVAL:  Default value to compare against for interrupt-on-change"<<std::endl
      <<"  INTCON:  Interrupt control, 0=compare against previous pin value, 1=compare to DEFVAL"<<std::endl
      <<"  IOCON:   Configuration, BANK, MIRROR, SEQOP, DISSLW, HAEN, ODR, INTPOL, - "<<std::endl
      <<"           BANK: 1=different banks, MIRROR: 1=INT pins are connected, SEQOP: 1=address pointer does not increment, DISSLW: 1=slew rate disabled, "<<std::endl
      <<"           HAEN: 1=hardware address pins enabled, ODR: 1=INT is open-drain, INTPOL: 1=INT is active-high (only if ODR=0)"<<std::endl
      <<"  GPPU:    GP pull-up restistors, 1=pull-up enabled, 0=pull-up disabled"<<std::endl
      <<"  INTF:    Interrupt flag, 0=no interrupt, 1=pin caused interrupt"<<std::endl
      <<"  INTCAP:  Interrupt captured value"<<std::endl
      <<"  GPIO:    GP port value (read/write)"<<std::endl
      <<"  OLAT:    GP output latch value (read/write). Write to GPIO = write to OLAT. Read from OLAT != read from GPIO"<<std::endl;
  }
}

void IOExtender::showChasingLight()
{
  std::cout<<"IOExtender::showChasingLight"<<std::endl;

  for(int i=0; i<16; i++)
  {
    // set all LEDs
    for(int j=0; j<16; j++)
    {
      setOutputCached(j, i==j);
    }
    applyOutputValues();

    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void IOExtender::setOutputCached(int ledNumber, bool on)
{
  if(ledNumber < 0 || ledNumber >= 16)
  {
    std::cout<<"Error in IOExtender::setOutput: number "<<ledNumber<<" is invalid!"<<std::endl;
    return;
  }
  currentOutput_[ledNumber] = on;
}

///! set the value of a single output led
void IOExtender::setOutput(int ledNumber, bool on)
{
  setOutputCached(ledNumber, on);
  applyOutputValues();
}

///! set the value of all output leds
void IOExtender::setOutput(bool on[16])
{
  for(int i=0; i<16; i++)
    currentOutput_[i] = on[i];
  applyOutputValues();
}

void IOExtender::applyOutputValues()
{
  if(!configured_)
    configure();

  unsigned char buffer[4];
  buffer[0] = deviceOpcode_ | RegisterCommand::write;
  buffer[1] = RegisterAddress::OLATA;
  buffer[2] = 0x00;
  buffer[3] = 0x00;

  // set first data byte
  int i=0;
  unsigned char mask = 0x01;
  for(i=0; i<8; i++)
  {
    buffer[2] |= (currentOutput_[i]? 1 : 0) * mask;
    mask <<= 1;
  }

  // set second data byte
  mask = 0x01;
  for(i=8; i<16; i++)
  {
    buffer[3] |= (currentOutput_[i]? 1 : 0) * mask;
    mask <<= 1;
  }

  control_.spiTransfer(spiComponent_, buffer, sizeof(buffer));
}

uint32_t IOExtender::maximumSPIBitrate()
{
return 1e4;
  return 10e6;    // 10 MHz
}
