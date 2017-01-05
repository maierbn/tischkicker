
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
  std::cout<<"end"<<std::endl;
  exit(0);

	int ret;
	unsigned char buffer[256];
	const int MAX_STR = 255;
	wchar_t wstr[255];
	hid_device *device_handle = NULL;
	struct hid_device_info *cur_dev;

  // initialize hidapi
	if (hid_init())
  {
    std::cout << "Failed to initialize hid!" << std::endl;
		return -1;
  }

  // enumerate usb devices
	struct hid_device_info *devices = NULL;
	struct hid_device_info *current_device = devices;

  std::stringstream s;
  int n_devices = 0;

  do
  {
    devices = hid_enumerate(0, 0);   // enumerate all
    current_device = devices;

    s.str("");
    for (int i=0; current_device != NULL; current_device = current_device->next, i++)
    {
      s << " device " << i << ": \"" << current_device->product_string << "\"" << std::endl
        << "\"" << current_device->manufacturer_string << "\"" << std::endl
        << " vendor/product id " << current_device->vendor_id << "/"<<current_device->product_id
        << ", S/N: " << current_device->serial_number << ", release "<<current_device->release_number
        << ", Interface No " << current_device->interface_number << " \""<<current_device->path<<"\""<<std::endl;
      n_devices++;
    }

    hid_free_enumeration(devices);

    if(n_devices == 0)
    {
      std::cout<<"No USB HID device found. Retrying . . . "<<std::string(80, '\b');
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
      std::cout << n_devices << " Devices found." <<std::endl;
      std::cout<<s.str()<<std::endl;
    }
  }
  while(n_devices == 0);

	// Set up the command buffer.
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0x01;
	buffer[1] = 0x81;

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	////device_handle = hid_open(0x4d8, 0x3f, L"12345");
	device_handle = hid_open(current_device->vendor_id, current_device->product_id, NULL);
	if (!device_handle)
	{
    std::cout<< "Could not open device \""<<current_device->product_string<<"\", \""
      <<current_device->manufacturer_string<<"\"!"<<std::endl;

 		return 1;
	}
	else
	{
    std::cout<<"Opened device \""<<current_device->product_string<<"\", \""
      <<current_device->manufacturer_string<<"\"."<<std::endl;
	}
/*
	// Read the Manufacturer String
	wstr[0] = 0x0000;
  ret = hid_get_manufacturer_string(device_handle, wstr, MAX_STR);
	if (ret < 0)
		printf("Unable to read manufacturer string\n");
	printf("Manufacturer String: %ls\n", wstr);

	// Read the Product String
	wstr[0] = 0x0000;
	ret = hid_get_product_string(device_handle, wstr, MAX_STR);
	if (ret < 0)
		printf("Unable to read product string\n");
	printf("Product String: %ls\n", wstr);

	// Read the Serial Number String
	wstr[0] = 0x0000;
	ret = hid_get_serial_number_string(device_handle, wstr, MAX_STR);
	if (ret < 0)
		printf("Unable to read serial number string\n");
	printf("Serial Number String: (%d) %ls", wstr[0], wstr);
	printf("\n");

	// Read Indexed String 1
	wstr[0] = 0x0000;
	ret = hid_get_indexed_string(device_handle, 1, wstr, MAX_STR);
	if (ret < 0)
		printf("Unable to read indexed string 1\n");
	printf("Indexed String 1: %ls\n", wstr);
	*/
/*
	// Set the hid_read() function to be non-blocking.
	hid_set_nonblocking(device_handle, 1);

	// Try to read from the device. There shoud be no
	// data here, but execution should not block.
	ret = hid_read(device_handle, buf, 17);

	// Send a Feature Report to the device
	buf[0] = 0x2;
	buf[1] = 0xa0;
	buf[2] = 0x0a;
	buf[3] = 0x00;
	buf[4] = 0x00;
	ret = hid_send_feature_report(device_handle, buf, 17);
	if (ret < 0) {
		printf("Unable to send a feature report.\n");
	}

	memset(buf,0,sizeof(buf));

	// Read a Feature Report from the device
	buf[0] = 0x2;
	ret = hid_get_feature_report(device_handle, buf, sizeof(buf));
	if (ret < 0) {
		printf("Unable to get a feature report.\n");
		printf("%ls", hid_error(device_handle));
	}
	else {
		// Print out the returned buffer.
		printf("Feature Report\n   ");
		for (i = 0; i < ret; i++)
			printf("%02hhx ", buf[i]);
		printf("\n");
	}

	memset(buf,0,sizeof(buf));

	// Toggle LED (cmd 0x80). The first byte is the report number (0x1).
	buf[0] = 0x1;
	buf[1] = 0x80;
	ret = hid_write(device_handle, buf, 17);
	if (ret < 0) {
		printf("Unable to write()\n");
		printf("Error: %ls\n", hid_error(device_handle));
	}


	// Request state (cmd 0x81). The first byte is the report number (0x1).
	buf[0] = 0x1;
	buf[1] = 0x81;
	hid_write(device_handle, buf, 17);
	if (ret < 0)
		printf("Unable to write() (2)\n");

	// Read requested state. hid_read() has been set to be
	// non-blocking by the call to hid_set_nonblocking() above.
	// This loop demonstrates the non-blocking nature of hid_read().
	ret = 0;
	while (ret == 0) {
		ret = hid_read(device_handle, buf, sizeof(buf));
		if (ret == 0)
			printf("waiting...\n");
		if (ret < 0)
			printf("Unable to read()\n");
		#ifdef WIN32
		Sleep(500);
		#else
		usleep(500*1000);
		#endif
	}

	printf("Data read:\n   ");
	// Print out the returned buffer.
	for (i = 0; i < ret; i++)
		printf("%02hhx ", buf[i]);
	printf("\n");
*/

	if(device_handle)
    hid_close(device_handle);

	/* Free static HIDAPI objects. */
	hid_exit();


	exit(0);
	return 0;
}
