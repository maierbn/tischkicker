
#include "hidapi.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include "spi_bridge.h"

int main(int argc, char* argv[])
{
  SPIBridge spiBridge;
  spiBridge.getAllSettings();
  spiBridge.outputSettings();

  spiBridge.setManufacturerName(L"Benni Maier");
  spiBridge.setProductName(L"MCP2210 USB to SPI Master");

  SPIBridge::PinDesignation pinDesignation[9];
  pinDesignation[0] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[1] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[2] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[3] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[4] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[5] = SPIBridge::PinDesignation::GPIO;
  pinDesignation[6] = SPIBridge::PinDesignation::DedicatedFunction;
  pinDesignation[7] = SPIBridge::PinDesignation::DedicatedFunction;
  pinDesignation[8] = SPIBridge::PinDesignation::DedicatedFunction;

  bool ioValue[9];
  bool ioDirection[9];
  for(int i=0; i<9; i++)
  {
    ioValue[i] = 0;
    ioDirection[i] = 0;
  }

  spiBridge.setSettings(true, pinDesignation, ioValue, ioDirection, true, SPIBridge::InterruptPinMode::noInterruptCounting, false);
  spiBridge.outputSettings();

  std::cout<<"switch on GP0 to GP5"<<std::endl;
  spiBridge.setGPOutputPin(0, 1);
  spiBridge.setGPOutputPin(1, 1);
  spiBridge.setGPOutputPin(2, 1);
  spiBridge.setGPOutputPin(3, 1);
  spiBridge.setGPOutputPin(4, 1);
  spiBridge.setGPOutputPin(5, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout<<"start moving light"<<std::endl;

  // moving light
  for(int i=0; ; i++)
  {
    if(i == 6)
      i=0;
    for(int j=0; j<6; j++)
    {
      spiBridge.setGPOutputPin(j, i==j);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  }

  std::cout<<"Program done."<<std::endl;
	return 0;
}
