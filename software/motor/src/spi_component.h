#pragma once

///! all components with which communication via SPI is possible
enum SPIComponent
{
  None = 0,
  Motor0,   ///< motor driver
};

const char spiComponentName[][12] = {
  "None",
  "Motor0",   ///< motor driver
};

const int maxSpiComponent = SPIComponent::Motor0;  //< the last SPI component
