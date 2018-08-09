
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>

#include "init_logging.h"
#include "control.h"
#include "easylogging++.h"

int main(int argc, char* argv[])
{
  initializeLogging(argc, argv);

  Control control;
  control.debug();

  LOG(INFO) << "Program done.";
	return 0;
}
