
#include "hidapi.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include "control.h"

int main(int argc, char* argv[])
{
  Control control;
  control.testLEDs();
  //control.debug();

  std::cout<<"Program done."<<std::endl;
	return 0;
}
