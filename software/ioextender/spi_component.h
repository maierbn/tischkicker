#pragma once

///! all components with which communication via SPI is possible
enum SPIComponent
{
  None = 0,
  Motor0,   ///< motor driver
  Motor1,
  Motor2,
  Motor3,
  Motor4,
  Motor5,
  Motor6,
  Motor7,
  Motor8,
  Motor9,
  Motor10,
  Motor11,
  Motor12,
  Motor13,
  Motor14,
  Motor15,
  LEDOutput,   ///< LEDs via I/O extender chip on output board (16 white LEDs)
  mainboardIO,   ///< I/O extender on main board (2 push buttons, 2 white LEDs, 2 red LEDs)
  BemfADCA,      ///< backward EMF ADC A
  BemfADCB,       ///< backward EMF ADC B
};

const char spiComponentName[][12] = {
  "None",
  "Motor0",   ///< motor driver
  "Motor1",
  "Motor2",
  "Motor3",
  "Motor4",
  "Motor5",
  "Motor6",
  "Motor7",
  "Motor8",
  "Motor9",
  "Motor10",
  "Motor11",
  "Motor12",
  "Motor13",
  "Motor14",
  "Motor15",
  "LEDOutput",   ///< LEDs via I/O extender chip on output board (16 white LEDs)
  "mainboardIO",   ///< I/O extender on main board (2 push buttons, 2 white LEDs, 2 red LEDs)
  "BemfADCA",      ///< backward EMF ADC A
  "BemfADCB",       ///< backward EMF ADC B
};
