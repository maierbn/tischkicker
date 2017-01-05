#include "spi_bridge.h"

#include "hidapi.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <assert.h>
#include <locale>
#include <codecvt>

SPIBridge::SPIBridge()
{
  initDevice();
}

SPIBridge::~SPIBridge()
{
	if(hidDevice_)
    hid_close(hidDevice_);

	// free static HIDAPI objects
	hid_exit();
}

void SPIBridge::initDevice()
{
  // initialize hidapi
	if (hid_init())
  {
    std::cout << "Failed to initialize hidapi!" << std::endl;
		exit(1);
  }

  // enumerate usb devices
	struct hid_device_info *devices = NULL;
	struct hid_device_info *currentDevice = NULL;

  //setup converter
  std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;

  std::stringstream s;
  int n_devices = 0;

  // enumerate until HID is found
  do
  {
    devices = hid_enumerate(0, 0);   // enumerate all
    currentDevice = devices;

    s.str("");

    // loop over all available HIDs
    for (int i=0; currentDevice != NULL; currentDevice = currentDevice->next, i++)
    {
      s << " Device " << i << ": " << std::endl
        << "  product:      \"" << converter.to_bytes(currentDevice->product_string) << "\"" << std::endl
        << "  manufacturer: \"" << converter.to_bytes(currentDevice->manufacturer_string) << "\"" << std::endl
        << "  vendor id:     " << currentDevice->vendor_id << ", product id: " << currentDevice->product_id << std::endl
        << "  serial number: " << converter.to_bytes(currentDevice->serial_number) << ", release number: "<<currentDevice->release_number << std::endl
        << "  interface number: " << currentDevice->interface_number << ", path: \""<<currentDevice->path<<"\""<<std::endl;
      n_devices++;
    }

    //hid_free_enumeration(devices);

    if(n_devices == 0)
    {
      std::cout<<"No USB HID device found. Retrying . . . "<<std::string(80, '\b');
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
      std::cout << n_devices << " Device(s) found." <<std::endl;
      std::cout<<s.str()<<std::endl;
    }
  }
  while(n_devices == 0);

  // Select first enumerated device to be used
  currentDevice = devices;

  for(;;)
  {
    // Open the device using the VID, PID,
    // and optionally the Serial number.
    ////hidDevice_ = hid_open(0x4d8, 0x3f, L"12345");
    //hidDevice_ = hid_open(currentDevice->vendor_id, currentDevice->product_id, NULL);
    //hidDevice_ = hid_open(0, 0, NULL);
    hidDevice_ = hid_open_path(currentDevice->path);

    if (!hidDevice_)
    {
      std::cout<< "Could not open device \""<<converter.to_bytes(currentDevice->product_string)<<"\", \""
        <<converter.to_bytes(currentDevice->manufacturer_string)<<"\" at \""<<currentDevice->path<<"\"!"<<std::endl;
      currentDevice = currentDevice->next;
      if(currentDevice == NULL)
      {
        std::cout<<"No other HIDs are available."<<std::endl;
        exit(1);
      }
    }
    else
    {
      std::wcout<<"Open device \""<<std::wstring(currentDevice->product_string)<<"\", \""
        <<std::wstring(currentDevice->manufacturer_string)<<"\"."<<std::endl;
      break;
    }
  }
}

void SPIBridge::setPowerUpSettings(PinDesignation pinDesignation[9], bool defaultOutput[9], bool defaultDirection[9],
    bool enableRemoteWakeUp, InterruptPinMode interruptPinMode, bool enableSPIBusRelease)
{
  //initialize buffer to 0
  memset(buffer, 0x00, 64);

  //fill in command
  buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
  buffer[1] = 0x20;   // sub-command code for "Set Chip Settings Power-up Default"
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  // bytes 4 to 12: GP0-GP8 Pin Designation
  for(int i=0; i<9; i++)
  {
    buffer[4+i] = pinDesignation[i];
  }

  // bytes 13 and 14: Default GPIO Output
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[13] |= (defaultOutput[i] * mask);
    mask << 1;
  }
  buffer[14] |= defaultOutput[8] * 0x01;

  // bytes 15 and 16: Default GPIO Direction
  mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[15] |= (defaultDirection[i] * mask);
    mask << 1;
  }
  buffer[16] |= defaultDirection[8] * 0x01;

  // byte 17: other chip settings
  buffer[17] |= enableRemoteWakeUp * 0x10;
  buffer[17] |= (interruptPinMode << 1);
  buffer[17] |= enableSPIBusRelease * 0x01;

  buffer[18] = 0x00;    // no password protection of chip settings
  // bytes 19 to 26: new password characters (not used here)
  // bytes 27 to 63: reserved (fill with 0x00)

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x20);  // byte 2: sub-command echoed back

}
void SPIBridge::setPowerUpTransferValues(uint32_t bitRate, bool idleChipSelect[9], bool activeChipSelect[9],
  double delayCSToData, double delayLastByteToCS, double delayDataToData, int32_t nextSPIMessageLength, SPIMode spiMode)
{

  for(;;)
  {
    //initialize buffer to 0
    memset(buffer, 0x00, 64);

    //fill in command
    buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
    buffer[1] = 0x10;   // sub-command code for "Set SPI Power-up Transfer Settings"
    buffer[2] = 0x00;   // reserved
    buffer[3] = 0x00;   // reserved

    union {
      uint32_t uint;
      char bytes[4];
    };

    //example: 12000000 = 0x00B71B00, buffer[5] = 0x1B = 27, buffer[6] = 0xB7
    uint = bitRate;
    buffer[4] = bytes[0];   // lsbyte
    buffer[5] = bytes[1];
    buffer[6] = bytes[2];
    buffer[7] = bytes[3];   // msbyte

    // bytes 8 and 9: Idle Chip Select Value
    char mask = 0x01;
    for(int i=0; i<8; i++)
    {
      buffer[8] |= (idleChipSelect[i] * mask);
      mask << 1;
    }
    buffer[9] |= idleChipSelect[8] * 0x01;

    // bytes 10 and 11: Active Chip Select Value
    mask = 0x01;
    for(int i=0; i<8; i++)
    {
      buffer[10] |= (activeChipSelect[i] * mask);
      mask << 1;
    }
    buffer[11] |= activeChipSelect[8] * 0x01;

    // byte 12 and 13: chip select to data delay (quanta of 100us) as 16-bit value
    union {
      uint16_t int16;
      char b[2];
    };
    int16 = uint16_t(delayCSToData / 100.0e-6);

    buffer[12] = b[0];
    buffer[13] = b[1];

    // byte 14 and 15: last data byte to CS
    int16 = uint16_t(delayLastByteToCS / 100.0e-6);
    buffer[14] = b[0];
    buffer[15] = b[1];

    // byte 16 and 17: delay between subsequent data bytes
    int16 = uint16_t(delayDataToData / 100.0e-6);
    buffer[16] = b[0];
    buffer[17] = b[1];

    // byte 18 and 19: number of bytes to transfer per SPI Transaction
    int16 = nextSPIMessageLength;
    buffer[18] = b[0];
    buffer[19] = b[1];

    buffer[20] = spiMode;
    // bytes 21 to 63: reserved (fill with 0x00)

    // send command
    hid_write(hidDevice_, buffer, 64);

    // receive response (blocking)
    hid_read(hidDevice_, buffer, 64);

    // parse response (Response 2 or 3 in data sheet)
    assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
    assert(buffer[2] == 0x10);  // byte 2: sub-command echoed back

    if(buffer[1] == 0x00) // settings written
    {
      break;
    }
    else if(buffer[1] == 0xF8)    // settings not written, USB Transfer in Progress
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // resend command
    }
  }

}

void SPIBridge::setPowerUpKeyParameters(int16_t vendorId, int16_t productId, bool hostPowered, bool remoteWakeUpCapable, int currentAmountFromHost)
{
  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
  buffer[1] = 0x30;   // sub-command code for "Set USB Power-up Key Parameters"
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  union {
    uint16_t int16;
    char b[2];
  };

  // bytes 4 and 5: vendor id
  int16 = vendorId;
  buffer[4] = b[0];
  buffer[5] = b[1];

  // bytes 6 and 7: product id
  int16 = productId;
  buffer[6] = b[0];
  buffer[7] = b[1];

  // byte 8: chip power option
  buffer[8] |= hostPowered * 0x128;     // bit 7: host powered
  buffer[8] |= (!hostPowered) * 0x64;   // bit 6: self powered
  buffer[8] |= remoteWakeUpCapable * 0x32; // bit 5: remote wake-up capable

  // byte 9: requested current amount from USB host (quanta of 2 mA)
  buffer[9] = std::min(currentAmountFromHost, 510) / 2;

  // bytes 10 to 63: reserved (fill with 0x00)

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x30);  // byte 2: sub-command echoed back

}

void SPIBridge::setManufacturerName(std::string manufacturerName)
{
  if(manufacturerName.length() > 29)
  {
    manufacturerName.erase(29);
    std::cout<<"warning: manufacturer string will be cropped: ["<<manufacturerName<<"]"<<std::endl;
  }

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
  buffer[1] = 0x50;   // sub-command code for "Set Manufacturer String"
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  // byte 4: string length*2+2
  buffer[4] = manufacturerName.length()*2 + 2;

  // byte 5: USB String Descriptor ID = 0x03
  buffer[5] = 0x03;

  for(int i=0; i<manufacturerName.length(); i++)
  {
    buffer[6+2*i+0] = manufacturerName[i];
    buffer[6+2*i+1] = 0x00;     // 2nd byte of unicode character, but unicode characters are not used
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x50);  // byte 2: sub-command echoed back

}

void SPIBridge::setProductName(std::string productName)
{
  if(productName.length() > 29)
  {
    productName.erase(29);
    std::cout<<"warning: product string will be cropped: ["<<productName<<"]"<<std::endl;
  }

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
  buffer[1] = 0x40;   // sub-command code for "Set Manufacturer String"
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  // byte 4: string length*2+2
  buffer[4] = productName.length()*2 + 2;

  // byte 5: USB String Descriptor ID = 0x03
  buffer[5] = 0x03;

  for(int i=0; i<productName.length(); i++)
  {
    buffer[6+2*i+0] = productName[i];
    buffer[6+2*i+1] = 0x00;     // 2nd byte of unicode character, but unicode characters are not used
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x40);  // byte 2: sub-command echoed back

}
