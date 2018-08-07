#include "analog_digital_converter.h"

//#include "control.h"

AnalogDigitalConverter::AnalogDigitalConverter(Control &control) :
  control_(control)
{
}

double AnalogDigitalConverter::maximumSPIBitrate()
{
  return 8.0e9;    // 8 MHz
}
