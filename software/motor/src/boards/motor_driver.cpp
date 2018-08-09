#include "motor_driver.h"

#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <bitset>
#include <iomanip>
#include <cmath>

#include "easylogging++.h"
#include "control.h"

MotorDriver::MotorDriver(Control &control, int motorNumber) :
  control_(control),
  motorNumber_(motorNumber),
  spiComponent_((SPIComponent)(int(SPIComponent::Motor0) + motorNumber_))
{
  // store on-chip configuration
  configure();
}

void MotorDriver::configure()
{
  VLOG(1) << "MotorDriver::configure";
  
  // set control parameters
  // rotation direction: true: CCW, false: CW
  chipSettings_.dirctrl = true;

  // true: trigger NXT on falling edge, false: on rising edge
  chipSettings_.nxtp = false;

  // slopes of motor driver: 00=very fast, 01=fast, 10=slow, 11=very slow
  // reduce to reduce radiated/conducted emission
  chipSettings_.emc = std::bitset<2>("00");

  // speed and load angle (SLA) transparency
  // true: show voltage transient behaviour
  // false: smoother back emf
  chipSettings_.slat = false;

  // divider of back emf, speed load angle gain, false: 0.5, true: 0.25
  chipSettings_.slag = false;

  // doubling of PWM frequency, true: double frequency, false: default frequency
  chipSettings_.pwmf = false;

  // add artificial jitter to PWM, true: enabled, false: disabled
  chipSettings_.pwmj = false;

  // 001: 1/128, 010: 1/64,
  // 011: compensated full step, 2 phase (45deg, 135deg, 225deg, 315deg),
  // 100: compensated full step, 1 phase (0deg, 90deg, 180deg, 270deg),
  // other: defined by sm
  chipSettings_.esm = std::bitset<3>("001");

  // step mode if esm=111:
  // 000: 1/32, 001: 1/16, 010: 1/8, 011: 1/4,
  // 100: compensated 1/2, 101: uncompensated 1/2, 110: 1/1, 111: 1/1
  chipSettings_.sm = std::bitset<3>("000");

  // sleep mode: true: sleeping, false: active
  chipSettings_.slp = false;

  // activate motor driver outputs: true: enabled, false: disabled
  chipSettings_.moten = true;

  // peak or amplitude of the regulated current waveform in the motor coils
  // 0 <= cur < 32, 0:  132 mA, 25: 3 A
  // current ranges: 0-2, 3-9, 10-15, 16-31, if current is reduced over current ranges, overcurrent might be detected errornously
  int current_scale = 10;
  chipSettings_.cur = std::bitset<5>(current_scale);

  unsigned char cr0, cr1, cr2, cr3;
  cr0 = chipSettings_.cur.to_ulong() | (chipSettings_.sm.to_ulong() << 5);
  cr1 = (chipSettings_.dirctrl? 0x80: 0) | (chipSettings_.nxtp? 0x40: 0)
        | (chipSettings_.pwmf? 0x08: 0) | (chipSettings_.pwmj? 0x04: 0) | chipSettings_.emc.to_ulong();
  cr2 = (chipSettings_.moten? 0x80: 0) | (chipSettings_.slp? 0x40: 0) | (chipSettings_.slag? 0x20: 0) | (chipSettings_.slat? 0x10: 0);
  cr3 = chipSettings_.esm.to_ulong();

  writeRegister(RegisterAddress::CR0, cr0);
  writeRegister(RegisterAddress::CR1, cr1);
  writeRegister(RegisterAddress::CR2, cr2);
  writeRegister(RegisterAddress::CR3, cr3);
}

void MotorDriver::setStepWidth(int stepWidth)
{
  // esm
  // 001: 1/128, 010: 1/64,
  // 011: compensated full step, 2 phase (45deg, 135deg, 225deg, 315deg),
  // 100: compensated full step, 1 phase (0deg, 90deg, 180deg, 270deg),
  // other: defined by sm

  // sm:
  // 000: 1/32, 001: 1/16, 010: 1/8, 011: 1/4,
  // 100: compensated 1/2, 101: uncompensated 1/2, 110,111: uncompensated 1/1

  if(stepWidth == 128)
  {
    chipSettings_.esm = std::bitset<3>("001");
    unsigned char cr3 = chipSettings_.esm.to_ulong();
    writeRegister(RegisterAddress::CR3, cr3);
  }
  else if(stepWidth == 64)
  {
    chipSettings_.esm = std::bitset<3>("010");
    unsigned char cr3 = chipSettings_.esm.to_ulong();
    writeRegister(RegisterAddress::CR3, cr3);
  }
  else if(stepWidth == 32 || stepWidth == 16 || stepWidth == 8 || stepWidth == 4 || stepWidth == 2)
  {
    chipSettings_.esm = std::bitset<3>("111");
    unsigned char cr3 = chipSettings_.esm.to_ulong();
    writeRegister(RegisterAddress::CR3, cr3);
    if(stepWidth == 32)
    {
      chipSettings_.sm = std::bitset<3>("000");
    }
    else if(stepWidth == 16)
    {
      chipSettings_.sm = std::bitset<3>("001");
    }
    else if(stepWidth == 8)
    {
      chipSettings_.sm = std::bitset<3>("010");
    }
    else if(stepWidth == 4)
    {
      chipSettings_.sm = std::bitset<3>("011");
    }
    else if(stepWidth == 2)
    {
      chipSettings_.sm = std::bitset<3>("100");
    }
    unsigned char cr0 = chipSettings_.cur.to_ulong() | (chipSettings_.sm.to_ulong() << 5);
    writeRegister(RegisterAddress::CR0, cr0);
  }
  else if(stepWidth == 1)
  {
    chipSettings_.esm = std::bitset<3>("100");
    unsigned char cr3 = chipSettings_.esm.to_ulong();
    writeRegister(RegisterAddress::CR3, cr3);
  }
  else
  {
    LOG(ERROR) << "Error in MotorDriver::setStepWidth: invalid stepWidth "<<stepWidth;
    return;
  }
}

void MotorDriver::setMaxCurrent(double factor)
{
  // peak or amplitude of the regulated current waveform in the motor coils
  // 0 <= cur < 32, 0:  132 mA, 25: 3 A
  // current ranges: 0-2, 3-9, 10-15, 16-31, if current is reduced over current ranges, overcurrent might be detected errornously
  int current_scale = int(factor*32);
  if(current_scale >= 32)
    current_scale = 31;
  if(current_scale < 0)
    current_scale = 0;
  chipSettings_.cur = std::bitset<5>(current_scale);
  unsigned char cr0 = chipSettings_.cur.to_ulong() | (chipSettings_.sm.to_ulong() << 5);

  writeRegister(RegisterAddress::CR0, cr0);

}

void MotorDriver::setRotationDirection(bool clockwise)
{
  // rotation direction: true: CCW, false: CW
  chipSettings_.dirctrl = clockwise;
  unsigned char cr1 = (chipSettings_.dirctrl? 0x80: 0) | (chipSettings_.nxtp? 0x40: 0)
                       | (chipSettings_.pwmf? 0x08: 0) | (chipSettings_.pwmj? 0x04: 0) | chipSettings_.emc.to_ulong();
  
  writeRegister(RegisterAddress::CR1, cr1);
}

void MotorDriver::readPosition()
{
  readRegister(RegisterAddress::SR3, false);
  readRegister(RegisterAddress::SR4, false);
}

unsigned char MotorDriver::readRegister(RegisterAddress registerAddress, bool printOutput)
{
  if (printOutput)
  {
    LOG(INFO) << "";
    LOG(INFO) << "MotorDriver::readRegister("<<registerAddressName[registerAddress]<<")";
  }

  // send command "Read"
  RegisterCommand registerCommand = RegisterCommand::Read;
  unsigned char buffer[2] = {0,0};
  buffer[0] = registerCommand | registerAddress;
  buffer[1] = 0x00;
  control_.spiTransfer(spiComponent_, buffer, sizeof(buffer));

  if (printOutput)
  {
    output("   Value of "+std::string(registerAddressName[registerAddress]), buffer);
  }

  parseResponse(buffer[1], registerAddress, printOutput);
  return buffer[1];
}

void MotorDriver::writeRegister(RegisterAddress registerAddress, unsigned char value)
{
  LOG(INFO) << "";
  LOG(INFO) << "MotorDriver::writeRegister("<<registerAddressName[registerAddress]<<", "<<std::bitset<8>(value)
    <<" = "<<std::hex<<std::setfill('0')<<std::setw(2)<<int(value)<<std::dec<<")";

  // send command "Write"
  RegisterCommand registerCommand = RegisterCommand::Write;
  unsigned char buffer[2] = {0,0};
  buffer[0] = registerCommand | registerAddress;
  buffer[1] = value;
  control_.spiTransfer(spiComponent_, buffer, sizeof(buffer));

  output("  Previous value of "+std::string(registerAddressName[registerAddress]), buffer);
}

void MotorDriver::output(std::string specifier, unsigned char byte[2])
{ 
  std::stringstream s;
  s << specifier << ":" << std::hex;
  
  for(int i = 0; i < 2; i++)
    s << " " << std::setfill('0') << std::setw(2) << int(byte[i]);
  
  s << std::dec << " =";
  for(int i = 0; i < 2; i++)
    s << " " << std::bitset<8>(byte[i]);
  
  LOG(INFO) << s.str();
}

void MotorDriver::ErrorFlags::reset()
{
  chargePump = false;
  openCoilX = false;
  openCoilY = false;
  overCurrentXNegativeBottom = false;
  overCurrentXNegativeTop = false;
  overCurrentXPositiveBottom = false;
  overCurrentXPositiveTop = false;
  overCurrentYNegativeBottom = false;
  overCurrentYNegativeTop = false;
  overCurrentYPositiveBottom = false;
  overCurrentYPositiveTop = false;
  thermalShutdown = false;
  thermalWarning = false;
  watchdogEvent = false;
}

void MotorDriver::ErrorFlags::print()
{
  if(chargePump)
    LOG(INFO) << "  Charge Pump Failure";
  if(openCoilX)
    LOG(INFO) << "  Open Coil X";
  if(openCoilY)
    LOG(INFO) << "  Open Coil Y";
  if(overCurrentXNegativeBottom) 
    LOG(INFO) << "  Overcurrent X Negative Bottom\a";
  if(overCurrentXNegativeTop)
    LOG(INFO) << "  Overcurrent X Negative Top\a";
  if(overCurrentXPositiveBottom)
    LOG(INFO) << "  Overcurrent X Positive Bottom\a";
  if(overCurrentXPositiveTop)
    LOG(INFO) << "  Overcurrent X Positive Top\a";
  if(overCurrentYNegativeBottom)
    LOG(INFO) << "  Overcurrent Y Negative Bottom\a";
  if(overCurrentYNegativeTop)
    LOG(INFO) << "  Overcurrent Y Negative Top\a";
  if(overCurrentYPositiveBottom)
    LOG(INFO) << "  Overcurrent Y Positive Bottom\a";
  if(overCurrentYPositiveTop)
    LOG(INFO) << "  Overcurrent Y Positive Top\a";
  if(thermalShutdown)
    LOG(INFO) << "  Thermal Shutdown (>180°C)\a";
  if(thermalWarning)
    LOG(INFO) << "  Thermal Warning (>160°C)\a";
  if(watchdogEvent)
    LOG(INFO) << "  Watchdog Event";

  if(!chargePump && !openCoilX && !openCoilY && !overCurrentXNegativeBottom
    && !overCurrentXNegativeTop && !overCurrentXPositiveBottom && !overCurrentXPositiveTop
    && !overCurrentYNegativeBottom && !overCurrentYNegativeTop && !overCurrentYPositiveBottom
    && !overCurrentYPositiveTop && !thermalShutdown && !thermalWarning && !watchdogEvent)
  {
    LOG(INFO) << "  No error flags set.";
  }

  if(overCurrentXNegativeBottom || overCurrentXNegativeTop || overCurrentXPositiveBottom || overCurrentXPositiveTop
     || overCurrentYNegativeBottom || overCurrentYNegativeTop || overCurrentYPositiveBottom || overCurrentYPositiveTop
     || thermalShutdown)
  {
     LOG(INFO) << "Execution paused for 1 second to prevent damage to the IC!";
     std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool MotorDriver::parseResponse(unsigned char byte, RegisterAddress registerAddress, bool printOutput)
{
  if (printOutput)
  {
    LOG(INFO) << "  MotorDriver::parseResponse: register "<<registerAddressName[registerAddress]<<", byte="
      <<std::hex<<std::setfill('0')<<std::setw(2)<<int(byte)<<std::dec
      <<" "<<std::bitset<8>(byte);
  }

  // for status registers check parity bit
  if (registerAddress == RegisterAddress::SR0 
    || registerAddress == RegisterAddress::SR1 
    || registerAddress == RegisterAddress::SR2 
    || registerAddress == RegisterAddress::SR3 
    || registerAddress == RegisterAddress::SR4 
  )
  {
    // check parity, number of ones is always even (assured by parity bit byte<<7)
    // http://graphics.stanford.edu/~seander/bithacks.html#ParityNaive
    bool parity = false;  // parity will be the parity of v
    char v = byte;
    while (v)
    {
      parity = !parity;
      v = v & (v - 1);
    }
    if (parity)
    {
      LOG(INFO) << "  Parity mismatch in byte "<<std::hex<<std::setfill('0')<<std::setw(2)<<int(byte)<<std::dec
        <<", received parity = "<<(bool(byte & 0x80)? "1" : "0");
      return false;
    }
  }

  // parse data in byte
  // status registers (read access)
  if (registerAddress == RegisterAddress::SR0)
  {
    std::bitset<1> tw(byte >> 6);
    std::bitset<1> cpfail(byte >> 5);
    std::bitset<1> wd(byte >> 4);
    std::bitset<1> openx(byte >> 3);
    std::bitset<1> openy(byte >> 2);
    
    if (printOutput)
      LOG(INFO) <<"  TW: "<<tw<<", CPFAIL: "<<cpfail<<", WD: "<<wd<<", OPENX: "<<openx<<", OPENY: "<<openy;
    
    errorFlags_.reset();
    errorFlags_.thermalWarning = tw[0];
    errorFlags_.chargePump = cpfail[0];
    errorFlags_.watchdogEvent = wd[0];
    errorFlags_.openCoilX = openx[0];
    errorFlags_.openCoilY = openy[0];
    
    if (printOutput)
      errorFlags_.print();
  }
  else if(registerAddress == RegisterAddress::SR1)
  {
    std::bitset<1> ovcxpt(byte >> 6);
    std::bitset<1> ovcxpb(byte >> 5);
    std::bitset<1> ovcxnt(byte >> 4);
    std::bitset<1> ovcxnb(byte >> 3);
    
    if (printOutput)
      LOG(INFO) <<"  OVCXPT: "<<ovcxpt<<", OVCXPB: "<<ovcxpb<<", OVCXNT: "<<ovcxnt<<", OVCXNB: "<<ovcxnb;
    
    errorFlags_.reset();
    errorFlags_.overCurrentXPositiveTop = ovcxpt[0];
    errorFlags_.overCurrentXPositiveBottom = ovcxpb[0];
    errorFlags_.overCurrentXNegativeTop = ovcxnt[0];
    errorFlags_.overCurrentXNegativeBottom = ovcxnb[0];
    
    if (printOutput)
      errorFlags_.print();
  }
  else if(registerAddress == RegisterAddress::SR2)
  {
    std::bitset<1> ovcypt(byte >> 6);
    std::bitset<1> ovcypb(byte >> 5);
    std::bitset<1> ovcynt(byte >> 4);
    std::bitset<1> ovcynb(byte >> 3);
    std::bitset<1> tsd(byte >> 2);
    
    if (printOutput)
      LOG(INFO) <<"  OVCYPT: "<<ovcypt<<", OVCYPB: "<<ovcypb<<", OVCYNT: "<<ovcynt<<", OVCYNB: "<<ovcynb<<", TSD: "<<tsd;
    
    errorFlags_.reset();
    errorFlags_.overCurrentYPositiveTop = ovcypt[0];
    errorFlags_.overCurrentYPositiveBottom = ovcypb[0];
    errorFlags_.overCurrentYNegativeTop = ovcynt[0];
    errorFlags_.overCurrentYNegativeBottom = ovcynb[0];
    errorFlags_.thermalShutdown = tsd[0];
    
    if (printOutput)
      errorFlags_.print();
  }
  else if(registerAddress == RegisterAddress::SR3)
  {
    std::bitset<7> msp(byte); // MSP[8:2]

    int position = 0;
    for(int i=2; i<=8; i++)
    {
      microStepPosition_[i] = msp[position];
      position++;
    }
    
    if (printOutput)
      LOG(INFO) <<"  MSP[8:2]: "<<msp<<", micro step position: "<<microStepPosition_;
  }
  else if(registerAddress == RegisterAddress::SR4)
  {
    std::bitset<7> msp(byte); // MSP[6:0]

    int position = 0;
    for(int i=0; i<=6; i++)
    {
      microStepPosition_[i] = msp[position];
      position++;
    }
    
    if (printOutput)
      LOG(INFO) <<"  MSP[6:0]: "<<msp<<", micro step position: "<<microStepPosition_;
  }
  // control registers (read/write access)
  else if(registerAddress == RegisterAddress::CR0)
  {
    std::bitset<3> sm(byte >> 5);
    std::bitset<5> cur(byte);
    LOG(INFO) <<"  CUR: "<<cur.to_ulong()<<", SM: "<<sm;
  }
  else if(registerAddress == RegisterAddress::CR1)
  {
    std::bitset<1> dirctrl(byte >> 7);
    std::bitset<1> nxtp(byte >> 6);
    std::bitset<1> pwmf(byte >> 3);
    std::bitset<1> pwmj(byte >> 2);
    std::bitset<2> emc(byte);
    
    if (printOutput)
      LOG(INFO) <<"  DIRCTRL: "<<dirctrl<<", NXTP: "<<nxtp<<", PWMF: "<<pwmf<<", PWMJ: "<<pwmj<<", EMC: "<<emc;
  }
  else if(registerAddress == RegisterAddress::CR2)
  {
    std::bitset<1> moten(byte >> 7);
    std::bitset<1> slp(byte >> 6);
    std::bitset<1> slag(byte >> 5);
    std::bitset<1> slat(byte >> 4);
    
    if (printOutput)
      LOG(INFO) <<"  MOTEN: "<<moten<<", SLP: "<<slp<<", SLAG: "<<slag<<", SLAT: "<<slat;
  }
  else if(registerAddress == RegisterAddress::CR3)
  {
    std::bitset<3> esm(byte);
    
    if (printOutput)
      LOG(INFO) <<"  ESM: "<<esm;
  }
  else if(registerAddress == RegisterAddress::WR)
  {
      // not implemented
  }
  else
  {
    LOG(INFO) <<"  Error in MotorDriver::parseResponse: invalid register address "
      <<std::hex<<registerAddress;
  }
  return true;
}

void MotorDriver::debugMotor()
{
  LOG(INFO) << "=======================";
  LOG(INFO) << "MotorDriver::debugMotor, spiComponent: "<<spiComponentName[spiComponent_];

  // send command "Write CR0"
  writeRegister(RegisterAddress::CR0, (unsigned char)(std::bitset<8>("10100101").to_ulong()));

  setMaxCurrent(0.1);   // set relative current value, 1.0 corresponds to 3 A, 
  // 0.25 corresponds to 0.64A, the maximum for the small motor (QSH2818-32-07-006)
  // 0.38 corresponds to 1A, the maximum for the big motor (QSH4218-51-10-049)
 
  setStepWidth(1);     // set step width of one tick, 1/128 to 1/1 (parameter is inverse value), 1 = full stepping, 2 = half stepping etc.
  
  // send command "Read CR0"
  readRegister(RegisterAddress::CR0);
  readRegister(RegisterAddress::CR1);
  readRegister(RegisterAddress::CR2);
  readRegister(RegisterAddress::CR3);

  // send command "Read SR0"
  readRegister(RegisterAddress::SR0);
  readRegister(RegisterAddress::SR1);
  readRegister(RegisterAddress::SR2);
  readRegister(RegisterAddress::SR3);
  readRegister(RegisterAddress::SR4);
  
  int oldPosition = microStepPosition_.to_ulong();
  
  bool directionCw = true;
  
  while(true)
  {
    setRotationDirection(directionCw);
    directionCw = !directionCw;
   
    int nSteps = 200;
     
    auto start = std::chrono::steady_clock::now();
    control_.triggerNXT(motorNumber_, nSteps);
    std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start);
    LOG(INFO) << duration.count()/1000. << "ms";
    
    readPosition();
    LOG(INFO) << "current position: " << microStepPosition_.to_ulong() << " delta: " << int(microStepPosition_.to_ulong()) - oldPosition;
    oldPosition = int(microStepPosition_.to_ulong());
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

void MotorDriver::moveToPosition(int position)
{
  int currentPosition = microStepPosition_.to_ulong();   // 0 to 511
  currentPosition = currentPosition_;

  // compute amount to turn
  int difference = position - currentPosition;

  // extract direction
  bool direction = difference > 0;
  difference = abs(difference);

  int nStepsFull = int(difference/128);
  difference -= nStepsFull * 128;

  int nSteps2 = int(difference/64);
  difference -= nSteps2 * 64;

  int nSteps4 = int(difference/32);
  difference -= nSteps4 * 32;

  int nSteps8 = int(difference/16);
  difference -= nSteps8 * 16;

  int nSteps16 = int(difference/8);
  difference -= nSteps16 * 8;

  int nSteps32 = int(difference/4);
  difference -= nSteps32 * 4;

  int nSteps64 = int(difference/2);
  difference -= nSteps64 * 2;

  int nSteps128 = difference;

  LOG(INFO) << "move from "<<currentPosition<<" to "<<position<<" ("<<(position - currentPosition)<<"), direction="
    <<int(direction)<<std::endl
    <<"  number of steps: full: "<<nStepsFull<<", 1/2: "<<nSteps2<<", 1/4: "<<nSteps4<<", 1/8: "<<nSteps8<<", 1/16: "<<nSteps16
    <<", 1/32: "<<nSteps32<<", 1/64: "<<nSteps64<<", 1/128: "<<nSteps128;

  if(nStepsFull != 0)
  {
    setStepWidth(1);
    control_.triggerNXT(motorNumber_, nStepsFull);
  }
  if(nSteps2 != 0)
  {
    setStepWidth(2);
    control_.triggerNXT(motorNumber_, nSteps2);
  }
  if(nSteps4 != 0)
  {
    setStepWidth(4);
    control_.triggerNXT(motorNumber_, nSteps4);
  }
  if(nSteps8 != 0)
  {
    setStepWidth(8);
    control_.triggerNXT(motorNumber_, nSteps8);
  }
  if(nSteps16 != 0)
  {
    setStepWidth(16);
    control_.triggerNXT(motorNumber_, nSteps16);
  }
  if(nSteps32 != 0)
  {
    setStepWidth(32);
    control_.triggerNXT(motorNumber_, nSteps32);
  }
  if(nSteps64 != 0)
  {
    setStepWidth(64);
    control_.triggerNXT(motorNumber_, nSteps64);
  }
  if(nSteps128 != 0)
  {
    setStepWidth(128);
    control_.triggerNXT(motorNumber_, nSteps128);
  }

  currentPosition_ = position;
  while(position < 0)
  {
    position += 512;
  }
  
  microStepPosition_ = std::bitset<9>(position%512);
  
  LOG(INFO) << "after moveToPosition: currentPosition_="<<currentPosition_
    <<", microStepPosition_="<<microStepPosition_<<" (="<<microStepPosition_.to_ulong()<<")";
    
  readRegister(RegisterAddress::SR3);
  readRegister(RegisterAddress::SR4);
}

uint32_t MotorDriver::maximumSPIBitrate()
{
  //return 1.0e5;    // 100 kHz
  return 1.0e3;    // 1 kHz
  //return 1.0e6;    // 1 MHz
}
