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


const bool DEBUG = false;

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

  std::cout<<"Scanning USB devices . . ."<<std::string(50, '\b')<<std::flush;

  // enumerate until HID is found
  do
  {
    devices = hid_enumerate(0, 0);   // enumerate all
    currentDevice = devices;

    s.str("");

    // loop over all available HIDs
    for (int i=0; currentDevice != NULL; currentDevice = currentDevice->next, i++)
    {
      std::string product_string;
      std::string manufacturer_string;
      std::string serial_number;

      try
      {
        product_string = converter.to_bytes(currentDevice->product_string);
        manufacturer_string = converter.to_bytes(currentDevice->manufacturer_string);
        serial_number = converter.to_bytes(currentDevice->serial_number);
      }
      catch(...)
      {
        std::cout<<"Conversion from wstring failed."<<std::endl;
      }

      s << " Device " << i << ": " << std::endl
        << "  product:      \"" << product_string << "\"" << std::endl
        << "  manufacturer: \"" << manufacturer_string << "\"" << std::endl
        << "  vendor id:     " << currentDevice->vendor_id << ", product id: " << currentDevice->product_id << std::endl
        << "  serial number: " << serial_number << ", release number: "<<currentDevice->release_number << std::endl
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
    //hidDevice_ = hid_open(currentDevice->vendorId, currentDevice->productId, NULL);
    //hidDevice_ = hid_open(0, 0, NULL);
    hidDevice_ = hid_open_path(currentDevice->path);

    if (!hidDevice_)
    {
      std::cout<<"Could not open device \""<<converter.to_bytes(currentDevice->product_string)<<"\", \""
        <<converter.to_bytes(currentDevice->manufacturer_string)<<"\" at \""<<currentDevice->path<<"\"!"<<std::endl;
      std::cout<<"Maybe you need root privileges?"<<std::endl;
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

void SPIBridge::setSettings(bool currentValues, PinDesignation pinDesignation[9], bool ioValue[9], bool ioDirection[9],
    bool enableRemoteWakeUp, InterruptPinMode interruptPinMode, bool enableSPIBusRelease)
{
  if(DEBUG)
    std::cout << "SPIBridge::setSettings(currentValues=" << std::boolalpha << currentValues << ")" << std::endl;

  //initialize buffer to 0
  memset(buffer, 0x00, 64);

  ChipSettings *chipSettings;

  //fill in command
  if(currentValues)
  {
    buffer[0] = 0x21;   // command code for "Get (VM) GPIO Current Chip Settings"
    buffer[1] = 0x00;   // reserved
    buffer[2] = 0x00;   // reserved
    buffer[3] = 0x00;   // reserved
    chipSettings = &currentChipSettings_;
  }
  else
  {
    buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
    buffer[1] = 0x20;   // sub-command code for "Set Chip Settings Power-up Default"
    buffer[2] = 0x00;   // reserved
    buffer[3] = 0x00;   // reserved
    chipSettings = &powerUpChipSettings_;
  }

  // bytes 4 to 12: GP0-GP8 Pin Designation
  for(int i=0; i<9; i++)
  {
    buffer[4+i] = pinDesignation[i];
  }

  // bytes 13 and 14: Default GPIO Output
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[13] |= (ioValue[i] * mask);
    mask <<= 1;
  }
  buffer[14] |= ioValue[8] * 0x01;

  // bytes 15 and 16: Default GPIO Direction
  mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[15] |= (ioDirection[i] * mask);
    mask <<= 1;
  }
  buffer[16] |= ioDirection[8] * 0x01;

  // byte 17: other chip settings
  buffer[17] |= enableRemoteWakeUp * 0x10;
  buffer[17] |= (interruptPinMode << 1);
  buffer[17] |= (!enableSPIBusRelease) * 0x01;

  buffer[18] = 0x00;    // no password protection of chip settings
  // bytes 19 to 26: new password characters (not used here)
  // bytes 27 to 63: reserved (fill with 0x00)

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
	if(currentValues)
	{
    assert(buffer[0] == 0x21);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: settings written
	}
	else
	{
    assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: settings written
    assert(buffer[2] == 0x20);  // byte 2: sub-command echoed back
  }

	// copy settings to currentChipSettings_
	for(int i=0; i<9; i++)
	{
    chipSettings->pinDesignation[i] = pinDesignation[i];
    chipSettings->ioValue[i] = ioValue[i];
    chipSettings->ioDirection[i] = ioDirection[i];
    chipSettings->enableRemoteWakeUp = enableRemoteWakeUp;
    chipSettings->interruptPinMode = interruptPinMode;
    chipSettings->enableSPIBusRelease = enableSPIBusRelease;
	}

}

void SPIBridge::setTransferSettings(bool currentValues, uint32_t bitRate, bool idleChipSelect[9], bool activeChipSelect[9],
  double delayCSToData, double delayLastByteToCS, double delayDataToData, int32_t spiTransactionLength, SPIMode spiMode)
{
  if(DEBUG)
    std::cout << "SPIBridge::setTransferSettings(currentValues=" << std::boolalpha << currentValues << ")" << std::endl;

  for(;;)
  {
    //initialize buffer to 0
    memset(buffer, 0x00, 64);

    ChipSettings *chipSettings;

    //fill in command
    if(currentValues)
    {
      buffer[0] = 0x40;   // command code for "Set (VM) SPI Transfer Settings"
      buffer[1] = 0x00;   // reserved
      buffer[2] = 0x00;   // reserved
      buffer[3] = 0x00;   // reserved
      chipSettings = &currentChipSettings_;
    }
    else
    {
      buffer[0] = 0x60;   // command code for "Set Chip NVRAM Parameters"
      buffer[1] = 0x10;   // sub-command code for "Set SPI Power-up Transfer Settings"
      buffer[2] = 0x00;   // reserved
      buffer[3] = 0x00;   // reserved
      chipSettings = &powerUpChipSettings_;
    }

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
      mask <<= 1;
    }
    buffer[9] |= idleChipSelect[8] * 0x01;

    // bytes 10 and 11: Active Chip Select Value
    mask = 0x01;
    for(int i=0; i<8; i++)
    {
      buffer[10] |= (activeChipSelect[i] * mask);
      mask <<= 1;
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
    int16 = spiTransactionLength;
    buffer[18] = b[0];
    buffer[19] = b[1];

    buffer[20] = spiMode;
    // bytes 21 to 63: reserved (fill with 0x00)

    // send command
    hid_write(hidDevice_, buffer, 64);

    // receive response (blocking)
    hid_read(hidDevice_, buffer, 64);

    // parse response (Response 2 or 3 in data sheet)
    if(currentValues)
    {
      assert(buffer[0] == 0x40);
    }
    else
    {
      assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
      assert(buffer[2] == 0x10);  // byte 2: sub-command echoed back
    }

    if(buffer[1] == 0x00) // settings written
    {
      // copy settings to chipSettings
      chipSettings->bitRate = bitRate;
      for(int i=0; i<9; i++)
      {
        chipSettings->idleChipSelect[i] = idleChipSelect[i];
        chipSettings->activeChipSelect[i] = activeChipSelect[i];
      }
      chipSettings->delayCSToData = delayCSToData;
      chipSettings->delayDataToData = delayDataToData;
      chipSettings->delayLastByteToCS = delayLastByteToCS;
      chipSettings->spiTransactionLength = spiTransactionLength;
      chipSettings->spiMode = spiMode;
      break;
    }
    else if(buffer[1] == 0xF8)    // settings not written, USB Transfer in Progress
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // resend command
    }
  }

}

void SPIBridge::setUSBKeyParameters(int16_t vendorId, int16_t productId, bool hostPowered, bool remoteWakeUpCapable, int currentAmountFromHost)
{
  if(DEBUG)
    std::cout << "SPIBridge::setUSBKeyParameters" << std::endl;

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

	// copy settings to currentChipSettings_
	chipUSBSettings.vendorId = vendorId;
	chipUSBSettings.productId = productId;
	chipUSBSettings.currentAmountFromHost = currentAmountFromHost;
	chipUSBSettings.hostPowered = hostPowered;
	chipUSBSettings.remoteWakeUpCapable = remoteWakeUpCapable;

}

void SPIBridge::setManufacturerName(std::wstring manufacturerName)
{
  if(DEBUG)
    std::cout << "SPIBridge::setManufacturerName" << std::endl;

  if(manufacturerName.length() > 29)
  {
    manufacturerName.erase(29);
    std::wcout<<"warning: manufacturer string will be cropped: ["<<manufacturerName<<"]"<<std::endl;
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
    union
    {
      wchar_t c;
      char bytes[4];
    };
    c = manufacturerName[i];

    buffer[6+2*i+0] = bytes[0];
    buffer[6+2*i+1] = bytes[1];     // 2nd byte of unicode character
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x50);  // byte 2: sub-command echoed back

	chipUSBSettings.manufacturerName = manufacturerName;

}

void SPIBridge::setProductName(std::wstring productName)
{
  if(DEBUG)
    std::cout << "SPIBridge::setProductName" << std::endl;

  if(productName.length() > 29)
  {
    productName.erase(29);
    std::wcout<<"warning: product string will be cropped: ["<<productName<<"]"<<std::endl;
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
    union
    {
      wchar_t c;
      char bytes[4];
    };
    c = productName[i];

    buffer[6+2*i+0] = bytes[0];
    buffer[6+2*i+1] = bytes[1];     // 2nd byte of unicode character
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 2 in data sheet)
	assert(buffer[0] == 0x60);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: settings written
	assert(buffer[2] == 0x40);  // byte 2: sub-command echoed back

	chipUSBSettings.productName = productName;
}

void SPIBridge::getTransferSettings(bool currentValues)
{
  if(DEBUG)
    std::cout << "SPIBridge::getTransferSettings(currentValues=" << std::boolalpha << currentValues << ")" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  ChipSettings *chipSettings;

  // fill in command
  if(currentValues)
  {
    buffer[0] = 0x41;   // command code for "Get (VM) SPI Transfer Settings"
    buffer[1] = 0x00;   // Reserved
    chipSettings = &currentChipSettings_;
  }
  else
  {
    buffer[0] = 0x61;   // command code for "Get NVRAM Settings"
    buffer[1] = 0x10;   // sub-command code for "Get SPI Power-up Transfer Settings"
    chipSettings = &powerUpChipSettings_;
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	std::cout<<"response: ";
	for(int i=0; i<21; i++)
    std::cout<<std::hex<<int(buffer[i])<<" ";
  std::cout<<std::endl;

	// parse response (Response 1 in data sheet)
  if(currentValues)
  {
    assert(buffer[0] == 0x41);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: command completed successfully
    assert(buffer[2] == 0x11);  // byte 2: size in bytes of the SPI transfer structure (17 bytes)
  }
  else
  {
    assert(buffer[0] == 0x61);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: command completed successfully
    assert(buffer[2] == 0x10);  // byte 2: sub-command echoed back
  }
	// byte 3: don't care

	union {
    uint32_t int32;
    char b[4];
	};

	// bytes 4 to 7: bit rate
	memcpy(b, buffer+4, 4);
	chipSettings->bitRate = int32;

	// bytes 8 and 9: idle chip select value
	char mask = 0x01;
	for(int i=0; i<8; i++)
	{
    chipSettings->idleChipSelect[i] = buffer[8] & mask;
    mask <<= 1;
	}
	chipSettings->idleChipSelect[8] = buffer[9] & 0x01;

	// bytes 10 and 11: active chip select value
	mask = 0x01;
	for(int i=0; i<8; i++)
	{
    chipSettings->activeChipSelect[i] = buffer[10] & mask;
    mask <<= 1;
	}
	chipSettings->activeChipSelect[8] = buffer[11] & 0x01;

	// bytes 12 and 13: chip select to data delay (quanta of 100 us)
	union {
    uint16_t int16;
    char bytes[2];
	};

	bytes[0] = buffer[12];
	bytes[1] = buffer[13];
	chipSettings->delayCSToData = int16 * 100e-6;

	// bytes 14 and 15: last data byte to chip select (quanta of 100 us)
	bytes[0] = buffer[14];
	bytes[1] = buffer[15];
	chipSettings->delayLastByteToCS = int16 * 100e-6;

	// bytes 16 and 17: delay between subsequent data bytes (quanta of 100 us)
	bytes[0] = buffer[16];
	bytes[1] = buffer[17];
	chipSettings->delayDataToData = int16 * 100e-6;

	// bytes 18 and 19: bytes to transfer per SPI TRansaction
	bytes[0] = buffer[18];
	bytes[1] = buffer[19];
	chipSettings->spiTransactionLength = int16;

	// byte 20: SPI mode
	chipSettings->spiMode = SPIBridge::SPIMode(buffer[20]);

	// bytes 21 to 63: don't care
}

void SPIBridge::getSettings(bool currentValues)
{
  if(DEBUG)
    std::cout << "SPIBridge::getSettings(currentValues=" << std::boolalpha << currentValues << ")" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  ChipSettings *chipSettings;

  //fill in command
  if(currentValues)
  {
    buffer[0] = 0x20;   // command code for "Get (VM) GPIO Current Chip Settings"
    buffer[1] = 0x00;   // reserved
    buffer[2] = 0x00;   // reserved
    chipSettings = &currentChipSettings_;
  }
  else
  {
    buffer[0] = 0x61;   // command code for "Get NVRAM Settings"
    buffer[1] = 0x20;   // sub-command code for "Get SPI Power-up Transfer Settings"
    chipSettings = &powerUpChipSettings_;
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
	if(currentValues)
	{
    assert(buffer[0] == 0x20);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: command completed successfully
	}
	else
	{
    assert(buffer[0] == 0x61);  // byte 0: echos back the given command code
    assert(buffer[1] == 0x00);  // byte 1: command completed successfully
    assert(buffer[2] == 0x20);  // byte 2: sub-command echoed back
  }
	// byte 3: don't care

	// bytes 4 to 12: Pin designations
	for(int i=0; i<9; i++)
	{
    chipSettings->pinDesignation[i] = PinDesignation(buffer[i+4]);
	}

	// bytes 13 and 14: default GPIO output
	char mask = 0x01;
	for(int i=0; i<8; i++)
	{
    chipSettings->ioValue[i] = buffer[13] & mask;
    mask <<= 1;
	}
	chipSettings->ioValue[8] = buffer[14] & 0x01;

	// bytes 15 and 16: default GPIO direction
	mask = 0x01;
	for(int i=0; i<8; i++)
	{
    chipSettings->ioDirection[i] = buffer[15] & mask;
    mask <<= 1;
	}
	chipSettings->ioDirection[8] = buffer[16] & 0x01;

	// byte 17: other chip settings
	chipSettings->enableRemoteWakeUp = buffer[17] & 0x10;
	chipSettings->interruptPinMode = InterruptPinMode(buffer[17] & 0x02);
	chipSettings->enableSPIBusRelease = buffer[17] & 0x01;

	// byte 18: chip parameter access control
	assert(buffer[18] == 0x00);   // 0x00 = chip settings not protected, 0x40 = protected by password, 0x80 = permanently locked

	// bytes 19 to 63: don't care
}

void SPIBridge::getUSBKeyParameters()
{
  if(DEBUG)
    std::cout << "SPIBridge::getUSBKeyParameters" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x61;   // command code for "Get NVRAM Settings"
  buffer[1] = 0x30;   // sub-command code for "Get USB Key Parameters"

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
	assert(buffer[0] == 0x61);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: command completed successfully
	assert(buffer[2] == 0x30);  // byte 2: sub-command echoed back
	// bytes 3 to 11: don't care

	union {
    uint16_t int16;
    char bytes[2];
	};

	// bytes 12 and 13: vendor id
	bytes[0] = buffer[12];
	bytes[1] = buffer[13];
	chipUSBSettings.vendorId = int16;

	// bytes 14 and 15: product id
	bytes[0] = buffer[14];
	bytes[1] = buffer[15];
	chipUSBSettings.productId = int16;

	// bytes 16 to 28: don't care
	// byte 29: chip power option
	chipUSBSettings.hostPowered = buffer[29] & 0x80;
	chipUSBSettings.remoteWakeUpCapable = buffer[29] & 0x20;

	// byte 30: requested current amount from USB host (quanta of 2mA)
	chipUSBSettings.currentAmountFromHost = buffer[30]*2;

	// bytes 31 to 63: don't care
}

void SPIBridge::getManufacturerName()
{
  if(DEBUG)
    std::cout << "SPIBridge::getManufacturerName" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x61;   // command code for "Get NVRAM Settings"
  buffer[1] = 0x50;   // sub-command code for "Get USB Manufacturer Name"

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
	assert(buffer[0] == 0x61);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: command completed successfully
	assert(buffer[2] == 0x50);  // byte 2: sub-command echoed back
	// byte 3: don't care

	// byte 4: length
	int length = (buffer[4]-2)/2;

	// byte 5: 0x03
	assert(buffer[5] == 0x03);

	// bytes 6 to 63: remaining unicode characters
	chipUSBSettings.manufacturerName = L"";
	for(int i=0; i<length; i++)
	{
    union
    {
      wchar_t c;
      char bytes[2];
    };
    bytes[0] = buffer[6+i*2+0];
    bytes[1] = buffer[6+i*2+1];
    chipUSBSettings.manufacturerName += c;
	}
}

void SPIBridge::getProductName()
{
  if(DEBUG)
    std::cout << "SPIBridge::getProductName" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  // fill in command
  buffer[0] = 0x61;   // command code for "Get NVRAM Settings"
  buffer[1] = 0x40;   // sub-command code for "Get USB Manufacturer Name"

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
	assert(buffer[0] == 0x61);  // byte 0: echos back the given command code
	assert(buffer[1] == 0x00);  // byte 1: command completed successfully
	assert(buffer[2] == 0x40);  // byte 2: sub-command echoed back
	// byte 3: don't care

	// byte 4: length
	int length = (buffer[4]-2)/2;
	// byte 5: 0x03
	assert(buffer[5] == 0x03);

	// bytes 6 to 63: remaining unicode characters
	chipUSBSettings.productName = L"";
	for(int i=0; i<length; i++)
	{
    union
    {
      wchar_t c;
      char bytes[2];
    };
    bytes[0] = buffer[6+i*2+0];
    bytes[1] = buffer[6+i*2+1];
    chipUSBSettings.productName += c;
	}
}

void SPIBridge::getCurrentPinDirection()
{
  if(DEBUG)
    std::cout << "SPIBridge::getCurrentPinDirection" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  buffer[0] = 0x33;   // command code for "Get (VM) GPIO Current Pin Direction"
  buffer[1] = 0x00;   // reserved
  buffer[2] = 0x00;   // reserved

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
  assert(buffer[0] == 0x33);  // byte 0: echos back the given command code
  assert(buffer[1] == 0x00);  // byte 1: command completed successfully
  // bytes 2 and 3: don't care
  // byte 4: GPIO Direction
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    currentChipSettings_.ioDirection[i] = buffer[4] & mask;
    mask <<= 1;
  }
  currentChipSettings_.ioDirection[8] = buffer[5] & 0x01;

  // bytes 6 to 63: don't care
}

void SPIBridge::setCurrentPinDirection(bool ioDirection[9])
{
  if(DEBUG)
    std::cout << "SPIBridge::setCurrentPinDirection" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  buffer[0] = 0x32;   // command code for "Set (VM) GPIO Current Pin Direction"
  buffer[1] = 0x00;   // reserved
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  // byte 4: gpio direction
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[4] |= (ioDirection[i] * mask);
    mask <<= 1;
  }
  buffer[5] |= (ioDirection[8] * 0x01);
  // bytes 6 to 63: reserved

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
  assert(buffer[0] == 0x32);  // byte 0: echos back the given command code
  assert(buffer[1] == 0x00);  // byte 1: command completed successfully
  // bytes 2 to 63: don't care

  // copy new settings to chipSettings
  for(int i=0; i<9; i++)
  {
    currentChipSettings_.ioDirection[i] = ioDirection[i];
  }
}

void SPIBridge::getCurrentPinValue()
{
  if(DEBUG)
    std::cout << "SPIBridge::getCurrentPinValue" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  buffer[0] = 0x31;   // command code for "Get (VM) GPIO Current Pin Value"
  buffer[1] = 0x00;   // reserved
  buffer[2] = 0x00;   // reserved

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
  assert(buffer[0] == 0x31);  // byte 0: echos back the given command code
  assert(buffer[1] == 0x00);  // byte 1: command completed successfully
  // bytes 2 and 3: don't care
  // bytes 4 and 5: GPIO Value
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    currentChipSettings_.ioValue[i] = buffer[4] & mask;
    mask <<= 1;
  }
  currentChipSettings_.ioValue[8] = buffer[5] & 0x01;

  // bytes 6 to 63: don't care
}

void SPIBridge::setCurrentPinValue(bool ioValue[9])
{
  if(DEBUG)
    std::cout << "SPIBridge::setCurrentPinValue" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  buffer[0] = 0x30;   // command code for "Set (VM) GPIO Current Pin Value"
  buffer[1] = 0x00;   // reserved
  buffer[2] = 0x00;   // reserved
  buffer[3] = 0x00;   // reserved

  // byte 4: pin value
  char mask = 0x01;
  for(int i=0; i<8; i++)
  {
    buffer[4] |= (ioValue[i] * mask);
    mask <<= 1;
  }
  buffer[5] |= (ioValue[8] * 0x01);
  // bytes 6 to 63: reserved

  // copy new settings to chipSettings
  for(int i=0; i<9; i++)
  {
    currentChipSettings_.ioValue[i] = ioValue[i];
  }

  // send command
	hid_write(hidDevice_, buffer, 64);

	// receive response (blocking)
	hid_read(hidDevice_, buffer, 64);

	// parse response (Response 1 in data sheet)
  assert(buffer[0] == 0x30);  // byte 0: echos back the given command code
  assert(buffer[1] == 0x00);  // byte 1: command completed successfully
  // bytes 2 and 3: don't care

  // bytes 4 and 5: GPIO Value
  mask = 0x01;
  for(int i=0; i<8; i++)
  {
    currentChipSettings_.ioValue[i] = buffer[4] & mask;
    mask <<= 1;
  }
  currentChipSettings_.ioValue[8] = buffer[5] & 0x01;
  // bytes 6 to 63: don't care
}

void SPIBridge::spiTransfer(unsigned char *spiSendBuffer, unsigned char *spiRecvBuffer, size_t length)
{
  if(DEBUG || 1)
  {
    std::cout << "SPIBridge::spiTransfer of " << length << " bytes (hex): ";
    for(int i=0; i<length; i++)
    {
      std::cout<<std::hex<<int(spiSendBuffer[i])<<" ";
    }
    std::cout<<std::dec<<std::endl;
  }

  // initialize usbSendBuffer to 0
  unsigned char usbSendBuffer[64];
  unsigned char usbRecvBuffer[64];
  memset(usbSendBuffer, 0x00, 64);
  memset(spiRecvBuffer, 0x00, length);

  if(length > 60)
  {
    std::cout<<"Warning: Can only send 60 bytes in one SPI transfer!"<<std::endl;
    length = 60;
  }

  usbSendBuffer[0] = 0x42;   // command code for "Transfer SPI Data"
  usbSendBuffer[1] = length;   // number of bytes to be transferred in this packet
  usbSendBuffer[2] = 0x00;   // reserved
  usbSendBuffer[3] = 0x00;   // reserved

  memcpy(usbSendBuffer+4, spiSendBuffer, length);

  for(int nMessages = 0; nMessages < 200; nMessages++)
  {
    // send command
    hid_write(hidDevice_, usbSendBuffer, 64);
    if(DEBUG)
    {
      std::cout<<"  SPIBridge::spiTransfer hid_write complete"<<std::endl;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));

    // receive response (blocking)
    hid_read(hidDevice_, usbRecvBuffer, 64);

    if(DEBUG)
    {
      std::cout<<"  usbRecvBuffer (hex): ";
      for(int i=0; i<64; i++)
      {
        std::cout<<std::hex<<int(usbRecvBuffer[i])<<" ";
      }
      std::cout<<std::dec<<std::endl;

      std::cout<<"  SPIBridge::spiTransfer hid_read complete, status 0x"<<std::hex<<int(usbRecvBuffer[1])<<std::dec<<std::endl;
    }

    // parse response (Response 1 or 2 or 3 in data sheet)
    assert(usbRecvBuffer[0] == 0x42);
    if(usbRecvBuffer[1] == 0x00)   // SPI Data accepted, command completed successfully
    {
      // parse received bytes
      size_t nBytesSPIReceived = usbRecvBuffer[2];
      memcpy(spiRecvBuffer, usbRecvBuffer+4, nBytesSPIReceived);

      if(DEBUG)
      {
        std::cout<<"> SPI Data accepted, "<<nBytesSPIReceived<<" bytes of SPI data received: ";
        for(int i=0; i<nBytesSPIReceived; i++)
        {
          std::cout<<std::hex<<int(spiRecvBuffer[i])<<" ";
        }
        std::cout<<std::dec<<"Msg: ";
      }

      if(usbRecvBuffer[3] == 0x20)   // SPI transfer started - no data to receive
      {
        if(DEBUG)
        {
          std::cout<<"SPI transfer started - no data to receive"<<std::endl;
        }
      }
      else if(usbRecvBuffer[3] == 0x30)    //SPI transfer not finished; received data available
      {
        if(DEBUG)
        {
          std::cout<<"SPI transfer not finished; received data available"<<std::endl;
        }
      }
      else if(usbRecvBuffer[3] == 0x10)    //SPI transfer finished - no more data to send
      {
        if(DEBUG)
        {
          std::cout<<"SPI transfer finished - no more data to send. Exit SPIBridge::spiTransfer."<<std::endl;
        }
        break;
      }
      else
      {
        std::cout<<"Error in SPIBridge::spiTransfer: unknown status " << std::hex<<int(usbRecvBuffer[3])<<std::dec<<" received"<<std::endl;
      }
      //break;
    }
    else if(usbRecvBuffer[1] == 0xF7)    // SPI Data not accepted, SPI bus not available (the external owner has control over it)
    {
      if(DEBUG)
      {
        std::cout<<"> SPI Data not accepted, SPI bus not available (the external owner has control over it)"<<std::endl;
        getChipStatus();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else if(usbRecvBuffer[1] == 0xF8)    // SPI Data not accepted, SPI transfer in progress, cannot accept any data for the moment
    {
      if(DEBUG)
      {
        std::cout<<"> SPI Data not accepted, SPI transfer in progress, cannot accept any data for the moment"<<std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    else
    {
      if(DEBUG)
      {
        std::cout<<"> Unknown status of SPI transfer, byte 1 is 0x"<<std::hex<<usbRecvBuffer[1]<<std::dec<<std::endl;
      }
    }
	}

}

void SPIBridge::getChipStatus()
{
  if(DEBUG)
    std::cout << "SPIBridge::getChipStatus" << std::endl;

  // initialize buffer to 0
  memset(buffer, 0x00, 64);

  buffer[0] = 0x10;   // byte 0: command code for "Get MCP2210 Status"
  // byte 2 to 63: reserved

  // send command
  hid_write(hidDevice_, buffer, 64);

  // receive response (blocking)
  hid_read(hidDevice_, buffer, 64);

  // parse response (Response 1 in data sheet)
  assert(buffer[0] == 0x10);  // byte 0: echos back the given command code
  assert(buffer[1] == 0x00);  // byte 1: command completed successfully

  std::cout<<"MCP2210 Status: "<<std::endl;

  // byte 2: SPI Bus Release External Request Status
  if(buffer[2] == 0x01)
  {
    std::cout<<"  No External Request for SPI Bus Release"<<std::endl;
  }
  else if(buffer[2] == 0x00)
  {
    std::cout<<"  Pending External Request for SPI Bus Release"<<std::endl;
  }

  // byte 3: SPI Bus Current Owner
  if(buffer[3] == 0x00)
  {
    std::cout<<"  SPI Bus current Owner: No Owner"<<std::endl;
  }
  else if(buffer[3] == 0x01)
  {
    std::cout<<"  SPI Bus current Owner: USB Bridge"<<std::endl;
  }
  else if(buffer[3] == 0x02)
  {
    std::cout<<"  SPI Bus current Owner: External Master"<<std::endl;
  }

  // byte 4: Attempted Password Accesses
  std::cout<<"  Number of Password Attempts: "<<int(buffer[4])<<std::endl;

  // byte 5: Password Guessed
  if(buffer[5] == 0x00)
  {
    std::cout<<"  Password Not Guessed"<<std::endl;
  }
  else if(buffer[5] == 0x01)
  {
    std::cout<<"  Password Guessed"<<std::endl;
  }

  // bytes 6 to 63: Don't Care

}

void SPIBridge::getAllSettings()
{
  getTransferSettings(false);
  getTransferSettings(true);
  getSettings(false);
  getSettings(true);
  getUSBKeyParameters();
  getManufacturerName();
  getProductName();
  getCurrentPinDirection();
  getCurrentPinValue();
}

void SPIBridge::outputSettings()
{
  std::cout<<"SPIBridge Settings"<<std::dec<<std::endl
    << "------ Power-up Chip Settings -------" << std::endl
    << "Pin Designation:  ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":";
    if(powerUpChipSettings_.pinDesignation[i] == PinDesignation::GPIO)
    {
      std::cout<<"GPIO ";
    }
    else if(powerUpChipSettings_.pinDesignation[i] == PinDesignation::ChipSelect)
    {
      std::cout<<"CS ";
    }
    else
    {
      std::cout<<"function ";
    }
  }
  std::cout<<std::endl
    <<"I/O Value:        ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(powerUpChipSettings_.ioValue[i])<<" ";
  }
  std::cout<<std::endl
    <<"I/O Direction:    ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<(powerUpChipSettings_.ioDirection[i]? "in" : "out")<<" ";
  }
  std::cout<<std::endl
    <<"enableRemoteWakeUp:   "<<std::boolalpha<<powerUpChipSettings_.enableRemoteWakeUp<<std::endl
    <<"interruptPinMode:    ";
  if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::countHighPulses)
  {
    std::cout<<" count high pulses"<<std::endl;
  }
  else if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::countHighPulses)
  {
    std::cout<<" count high pulses"<<std::endl;
  }
  else if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::countLowPulses)
  {
    std::cout<<" count low pulses"<<std::endl;
  }
  else if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::countRisingEdges)
  {
    std::cout<<" count rising edges"<<std::endl;
  }
  else if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::countFallingEdges)
  {
    std::cout<<" count falling edges"<<std::endl;
  }
  else if(powerUpChipSettings_.interruptPinMode == InterruptPinMode::noInterruptCounting)
  {
    std::cout<<" no interrupt counting"<<std::endl;
  }
  else
  {
    std::cout<<" invalid ("<<int(powerUpChipSettings_.interruptPinMode)<<")"<<std::endl;
  }
  std::cout<<"enableSPIBusRelease:  "<<std::boolalpha<<powerUpChipSettings_.enableSPIBusRelease<<std::endl;
  std::cout<<"bitRate:              "<<powerUpChipSettings_.bitRate<<std::endl
    <<"idleChipSelect:   ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(powerUpChipSettings_.idleChipSelect[i])<<" ";
  }
  std::cout<<std::endl
    <<"activeChipSelect: ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(powerUpChipSettings_.activeChipSelect[i])<<" ";
  }
  std::cout<<std::endl
    <<"delayCSToData:        "<<powerUpChipSettings_.delayCSToData<<" s"<<std::endl
    <<"delayLastByteToCS:    "<<powerUpChipSettings_.delayLastByteToCS<<" s"<<std::endl
    <<"delayDataToData:      "<<powerUpChipSettings_.delayDataToData<<" s"<<std::endl
    <<"spiTransactionLength: "<<powerUpChipSettings_.spiTransactionLength<<std::endl
    <<"spiMode:              "<<powerUpChipSettings_.spiMode<<std::endl
    << "------ Current Chip Settings -------" << std::endl
    << "Pin Designation:  ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":";
    if(currentChipSettings_.pinDesignation[i] == PinDesignation::GPIO)
    {
      std::cout<<"GPIO ";
    }
    else if(currentChipSettings_.pinDesignation[i] == PinDesignation::ChipSelect)
    {
      std::cout<<"CS ";
    }
    else
    {
      std::cout<<"function ";
    }
  }
  std::cout<<std::endl
    <<"I/O Value:        ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(currentChipSettings_.ioValue[i])<<" ";
  }
  std::cout<<std::endl
    <<"I/O Direction:    ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<(currentChipSettings_.ioDirection[i]? "in" : "out")<<" ";
  }
  std::cout<<std::endl
    <<"enableRemoteWakeUp:   "<<std::boolalpha<<currentChipSettings_.enableRemoteWakeUp<<std::endl
    <<"interruptPinMode:    ";
  if(currentChipSettings_.interruptPinMode == InterruptPinMode::countHighPulses)
  {
    std::cout<<" count high pulses"<<std::endl;
  }
  else if(currentChipSettings_.interruptPinMode == InterruptPinMode::countHighPulses)
  {
    std::cout<<" count high pulses"<<std::endl;
  }
  else if(currentChipSettings_.interruptPinMode == InterruptPinMode::countLowPulses)
  {
    std::cout<<" count low pulses"<<std::endl;
  }
  else if(currentChipSettings_.interruptPinMode == InterruptPinMode::countRisingEdges)
  {
    std::cout<<" count rising edges"<<std::endl;
  }
  else if(currentChipSettings_.interruptPinMode == InterruptPinMode::countFallingEdges)
  {
    std::cout<<" count falling edges"<<std::endl;
  }
  else if(currentChipSettings_.interruptPinMode == InterruptPinMode::noInterruptCounting)
  {
    std::cout<<" no interrupt counting"<<std::endl;
  }
  else
  {
    std::cout<<" invalid ("<<int(currentChipSettings_.interruptPinMode)<<")"<<std::endl;
  }
  std::cout<<"enableSPIBusRelease:  "<<std::boolalpha<<currentChipSettings_.enableSPIBusRelease<<std::endl;
  std::cout<<"bitRate:              "<<currentChipSettings_.bitRate<<std::endl
    <<"idleChipSelect:   ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(currentChipSettings_.idleChipSelect[i])<<" ";
  }
  std::cout<<std::endl
    <<"activeChipSelect: ";
  for(int i=0; i<9; i++)
  {
    std::cout<<i<<":"<<int(currentChipSettings_.activeChipSelect[i])<<" ";
  }
  std::cout<<std::endl
    <<"delayCSToData:        "<<currentChipSettings_.delayCSToData<<" s"<<std::endl
    <<"delayLastByteToCS:    "<<currentChipSettings_.delayLastByteToCS<<" s"<<std::endl
    <<"delayDataToData:      "<<currentChipSettings_.delayDataToData<<" s"<<std::endl
    <<"spiTransactionLength: "<<currentChipSettings_.spiTransactionLength<<" Byte(s)"<<std::endl
    <<"spiMode:              "<<currentChipSettings_.spiMode<<std::endl
    << "------ USB Settings -------" << std::endl
    <<"vendorId:             "<<chipUSBSettings.vendorId<<std::endl
    <<"productId:            "<<chipUSBSettings.productId<<std::endl
    <<"hostPowered:          "<<std::boolalpha<<chipUSBSettings.hostPowered<<std::endl
    <<"remoteWakeUpCapable:  "<<std::boolalpha<<chipUSBSettings.remoteWakeUpCapable<<std::endl
    <<"currentAmountFromHost:"<<chipUSBSettings.currentAmountFromHost<<" mA"<<std::endl;

  std::wcout<<"manufacturerName: "<<chipUSBSettings.manufacturerName<<std::endl;
  std::wcout<<"productName:      "<<chipUSBSettings.productName<<std::endl;
  std::cout<<"----------------------------------------"<<std::endl;

}

void SPIBridge::setPinDesignation(PinDesignation pinDesignation[9])
{
  setSettings(true, pinDesignation, currentChipSettings_.ioValue, currentChipSettings_.ioDirection,
    currentChipSettings_.enableRemoteWakeUp, currentChipSettings_.interruptPinMode, currentChipSettings_.enableSPIBusRelease);
}

void SPIBridge::setGPOutputPin(int number, bool value)
{

  if(number < 0 || number > 8)
  {
    std::cout<<"SPIBridge::setGPOutputPin: Error, wrong output pin number "<<number<<std::endl;
    return;
  }

  currentChipSettings_.ioValue[number] = value;
  setSettings(true, currentChipSettings_.pinDesignation, currentChipSettings_.ioValue,
    currentChipSettings_.ioDirection, currentChipSettings_.enableRemoteWakeUp,
    currentChipSettings_.interruptPinMode, currentChipSettings_.enableSPIBusRelease);
}

void SPIBridge::setGPOutputPinCached(int number, bool value)
{
  if(number < 0 || number > 8)
  {
    std::cout<<"SPIBridge::setGPOutputPin: Error, wrong output pin number "<<number<<std::endl;
    return;
  }

  currentChipSettings_.ioValue[number] = value;
}

void SPIBridge::setGPOutputPinFlush()
{
  setSettings(true, currentChipSettings_.pinDesignation, currentChipSettings_.ioValue,
    currentChipSettings_.ioDirection, currentChipSettings_.enableRemoteWakeUp,
    currentChipSettings_.interruptPinMode, currentChipSettings_.enableSPIBusRelease);
}

void SPIBridge::setGPOutputPins(bool value[9])
{
  setSettings(true, currentChipSettings_.pinDesignation, value, currentChipSettings_.ioDirection,
    currentChipSettings_.enableRemoteWakeUp, currentChipSettings_.interruptPinMode, currentChipSettings_.enableSPIBusRelease);
}

void SPIBridge::prepareSPITransfer(bool idleChipSelect[9], bool activeChipSelect[9], uint32_t bitRate, int32_t spiTransactionLength)
{
  // modify settings on chip
  setTransferSettings(true, bitRate, idleChipSelect, activeChipSelect,
    currentChipSettings_.delayCSToData, currentChipSettings_.delayLastByteToCS, currentChipSettings_.delayDataToData,
    spiTransactionLength, currentChipSettings_.spiMode);

}
