
#include "hidapi.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include "spi_bridge.h"
#include "motor_driver.h"

int main(int argc, char* argv[])
{
  SPIBridge spiBridge;
  MotorDriver motor(spiBridge, 0);

//  motor.debugMotor();
  motor.debugChasingLight();

  std::cout<<"Program done."<<std::endl;
	return 0;
}
