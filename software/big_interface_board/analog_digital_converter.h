#pragma once

#include <iostream>

class Control;

/** A class that gives access to the MCP3008 analog digital converter
*/
class AnalogDigitalConverter
{
public:
  AnalogDigitalConverter(Control &control);

  ///! returns the maximum bitrate that is possible for SPI communication
  static double maximumSPIBitrate();

private:

  Control &control_;   ///< the control object that has access to all components
};

