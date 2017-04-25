#include "io_extender.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>

#include "control.h"
#include "spi_component.h"

IOExtender::IOExtender(Control &control, int hardwareAddress) :
  control_(control),
  hardwareAddress_(hardwareAddress)
{
  if(hardwareAddress_ < 0 || hardwareAddress_ >= 8)
  {
    std::cout<<"Error in IOExtender(): hardwareAddress "<<hardwareAddress_<<" is invalid!"<<std::endl;
    return;
  }

  // initialize current output to false
  for(int i=0; i<16; i++)
    currentOutput_[i] = false;

  std::cout<<"Warning: IOExtender configuration disabled!"<<std::endl;
  return;

  // store on-chip configuration
  configure();
}

void IOExtender::configure()
{
  unsigned char buffer[3];
  deviceOpcode_ = 0x40 | (hardwareAddress_ << 1);

  // chip select byte and command
  buffer[0] = deviceOpcode_ | RegisterCommand::write;

  // register select byte
  buffer[1] = RegisterAddress::IOCON;

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

  // INT pin, true: open-drain output, false: active driver output (with polarity INTPOL)
  bool odr = true;

  // true: active-high, false: active-low
  bool intpol = false;

  // data byte
  buffer[2] = (bank? 0x80: 0) | (mirror? 0x40: 0) | (seqop? 0x20: 0) | (disslw? 0x10: 0)
   | (haen? 0x08: 0) | (odr? 0x04: 0) | (intpol? 0x02: 0);

  control_.spiTransfer(SPIComponent::LEDOutput, buffer, sizeof(buffer));
}

void IOExtender::showChasingLight()
{
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

  control_.spiTransfer(SPIComponent::LEDOutput, buffer, sizeof(buffer));
}

uint32_t IOExtender::maximumSPIBitrate()
{
  return 10e6;    // 10 MHz
}
