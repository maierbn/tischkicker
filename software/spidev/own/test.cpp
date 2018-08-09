#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstring>

int main(int argc, char *argv[])
{
  std::string filename;
  std::fstream file;
  
  file.open(filename.c_str(), std::ios::in | std::ios::out | std::ios::binary);
  
  if (!file.is_open())
  {
    std::cout << "Could not open file \"" << filename << "\"." << std::endl;
  }
  else 
  {
    std::cout << "Opened file \"" << filename << "\"." << std::endl;
  }
  
  char buffer[65];
  memset(&buffer, 0, 64);
  buffer[0] = 0x61;
  buffer[1] = 0x50;
  file.write(buffer, 64);
  
  file.read(buffer, 64);
  buffer[64] = '\0';
  
  std::cout << "Output: [" << std::string((const char *)buffer) << "]" << std::endl;
  if (file.eof())
    std::cout << "EOF" << std::endl;
  if (file.fail())
    std::cout << "FAIL" << std::endl;
  if (file.bad())
    std::cout << "BAD" << std::endl;
    
  file.close();
  
  return EXIT_SUCCESS;
}
