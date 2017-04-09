#pragma once

#include <iostream>
#include <bitset>

class Control;

/** A class that encapsules the stepper motor driver for motor motorNumber
*
*/
class MotorDriver
{
public:

  MotorDriver(Control &control, int motorNumber=0);

  ///! test function to check if motor driver chip works as expected
  void debugMotor();

  ///! turn motor to given position
  void moveToPosition(int position);

private:

  enum RegisterAddress
  {
    WR = 0x00,    // watchdog control register
    CR0 = 0x01,   // control register 0
    CR1 = 0x02,   // control register 1
    CR2 = 0x03,   // control register 2
    CR3 = 0x09,   // control register 3
    SR0 = 0x04,   // status register 0
    SR1 = 0x05,   // status register 1
    SR2 = 0x06,   // status register 2
    SR3 = 0x07,   // status register 3
    SR4 = 0x0A,   // status register 4
  };

  const std::string registerAddressName[11]
    = {"WR", "CR0", "CR1", "CR2", "SR0", "SR1", "SR2", "SR3", "invalid", "CR3", "SR4"};
  //   0x00, 0x01,  0x02,  0x03,  0x04,  0x05,  0x06,  0x07,  0x08,      0x09,  0x0A

  enum RegisterCommand
  {
    Read = 0x00,    // Read command mask:  0000 0000
    Write = 0x80,   // Write command mask: 1000 0000
  };

  struct ErrorFlags
  {
    bool chargePump;
    bool openCoilX;
    bool openCoilY;
    bool overCurrentXNegativeBottom;
    bool overCurrentXNegativeTop;
    bool overCurrentXPositiveBottom;
    bool overCurrentXPositiveTop;
    bool overCurrentYNegativeBottom;
    bool overCurrentYNegativeTop;
    bool overCurrentYPositiveBottom;
    bool overCurrentYPositiveTop;
    bool thermalShutdown;
    bool thermalWarning;
    bool watchdogEvent;

    void print();
  }
  errorFlags_;

  struct ChipSettings
  {
    bool dirctrl;
    bool nxtp;
    std::bitset<2> emc;
    bool slat;
    bool slag;
    bool pwmf;
    bool pwmj;
    std::bitset<3> sm;
    std::bitset<3> esm;
    bool slp;
    bool moten;
    std::bitset<5> cur;
  }
  chipSettings_;

  ///! store parameters on chip
  void configure();

  ///! parse the content of the SPI control or status (according to registerAddress) register in byte
  ///! \return: false if parity check fails, else true
  bool parseResponse(unsigned char byte, RegisterAddress registerAddress);

  ///! write 2 bytes to std output
  void output(std::string specifier, unsigned char byte[2]);

  unsigned char readRegister(RegisterAddress registerAddress);
  void writeRegister(RegisterAddress registerAddress, unsigned char value);

  ///! set the maximum current by a scale 0 <= factor < 1, value clamped
  void setMaxCurrent(double factor);

  ///! set the step width (number of steps for a quarter circle) to one of 1,2,4,8,16,32,64,128
  void setStepWidth(int stepWidth);

  Control &control_;   ///< the control object that has access to all components
  int motorNumber_;  ///< the number of the motor (0-15)
  int componentNumber_;   ///< the SPI component number of this chip
  std::bitset<9> microStepPosition_;
  int currentPosition_;
};
