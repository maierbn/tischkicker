#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/usb.h>
 
static dev_t device_number; // Global variable for the device_number device number 
static struct cdev character_device; // Global variable for the character device structure
static struct class *device_class; // Global variable for the device class

#define VENDOR_ID	  0x04d8
#define PRODUCT_ID	0x00de

/* table of devices that work with this driver */
static struct usb_device_id id_table[] =
{
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

struct mcp
{
	struct usb_device *mcp_usb_device;
  
  struct usb_endpoint_descriptor *int_in_endpoint;
	__u8 int_in_endpoint_address;
	unsigned char *int_in_buffer;
	struct urb *int_in_urb;
  size_t int_in_buffer_size;
  
  struct usb_endpoint_descriptor *int_out_endpoint;
	__u8 int_out_endpoint_address;
	unsigned char *int_out_buffer;
	struct urb *int_out_urb;
  size_t int_out_buffer_size;
  
  unsigned char *received_buffer;
  
};

static void read_int_callback(struct urb *urb)
{
	struct mcp *mcp_data = urb->context;
	unsigned char *transfer_buffer = urb->transfer_buffer;
  for (int i=0; i<64; i++)
  {
    mcp_data->received_buffer[i] = transfer_buffer[i];
  }
	int retval;
	int i;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dbg("%s - urb shutting down with status: %d",
		    __func__, urb->status);
		return;
	default:
		dbg("%s - nonzero urb status received: %d",
		    __func__, urb->status);
		goto exit;
	}

	dev_info(&urb->dev->dev, "int read data: ");
	for (i = 0; i < urb->actual_length; ++i)
		printk("%02x ", transfer_buffer[i]);
	printk("\n");

	dev_dbg(&urb->dev->dev, "counter %d, temperature=%d\n",
		 measurement->rolling_counter,
		 measurement->measurement0);
	gdev->temperature = le16_to_cpu(measurement->measurement0);

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&urb->dev->dev,
			"%s - Error %d submitting interrupt urb\n",
			__func__, retval);
}

static int mcp_open(struct inode *i, struct file *f)
{
  printk(KERN_INFO "Driver: open()\n");
  return 0;
}

static int mcp_close(struct inode *i, struct file *f)
{
  printk(KERN_INFO "Driver: close()\n");
  return 0;
}

static ssize_t mcp_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
  printk(KERN_INFO "Driver: read()\n");
  
  /* do an immediate bulk read to get data from the device */
  retval = usb_bulk_msg (skel->dev,
                         usb_rcvbulkpipe (skel->dev,
                         skel->bulk_in_endpointAddr),
                         skel->bulk_in_buffer,
                         skel->bulk_in_size,
                         &count, HZ*10);
  /* if the read was successful, copy the data to user space */
  if (!retval) {
          if (copy_to_user (buffer, skel->bulk_in_buffer, count))
                  retval = -EFAULT;
          else
                  retval = count;
  }
    
  return 0;
}

static ssize_t mcp_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
  printk(KERN_INFO "Driver: write()\n");
  
  /* we can only write as much as 1 urb will hold */
  bytes_written = (count > skel->bulk_out_size) ? skel->bulk_out_size : count;

  /* copy the data from user space into our urb */
  copy_from_user(skel->write_urb->transfer_buffer, buffer, bytes_written);

  /* set up our urb */
  usb_fill_bulk_urb(skel->write_urb,
                    skel->dev,
                    usb_sndbulkpipe(skel->dev, skel->bulk_out_endpointAddr),
                    skel->write_urb->transfer_buffer,
                    bytes_written,
                    skel_write_bulk_callback,
                    skel);

  /* send the data out the bulk port */
  result = usb_submit_urb(skel->write_urb);
  if (result) {
          err("Failed submitting write urb, error %d", result);
  }
    
  return len;
}

static int mcp_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
  printk(KERN_INFO "Driver: probe(), (%04X:%04X) plugged\n", id->idVendor, id->idProduct);
  
  struct usb_device *mcp_usb_device = interface_to_usbdev(interface);
  
	int retval = -ENOMEM;

  // initialize private data structure
  struct mcp *mcp_data = NULL;
	mcp_data = kzalloc(sizeof(*mcp_data), GFP_KERNEL);
	if (mcp_data == NULL)
  {
		dev_err(&interface->dev, "Out of memory\n");
		return -ENOMEM;
	}
  
  mcp_data->received_buffer = kzalloc(64, GFP_KERNEL);
  
  // determine endpoints and store them in mcp_data
  // get current setting
	struct usb_host_interface *mcp_usb_host_interface;
	struct usb_endpoint_descriptor *endpoint = NULL;
	mcp_usb_host_interface = interface->cur_altsetting;
  
  // loop over endpoints of current interface
	for (int i = 0; i < mcp_usb_host_interface->desc.bNumEndpoints; i++)
  {
    // extract endpoint
		endpoint = &mcp_usb_host_interface->endpoint[i].desc;

    // if the endpoint is an interrupt IN endpoint
		if (usb_endpoint_is_int_in(endpoint))
    {
      // allocate buffer with maximum size
			mcp_data->int_in_buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
      mcp_data->int_in_endpoint = endpoint;
			mcp_data->int_in_endpoint_address = endpoint->bEndpointAddress;
			mcp_data->int_in_buffer = kmalloc(mcp_data->int_in_buffer_size, GFP_KERNEL);
			if (!mcp_data->int_in_buffer)
      {
				dev_err(&interface->dev, "Could not allocate interrupt in buffer");
				return -ENOMEM;
			}
		}
    else if (usb_endpoint_is_int_out(endpoint))
    {
      // allocate buffer with maximum size
			mcp_data->int_out_buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
      mcp_data->int_out_endpoint = endpoint;
			mcp_data->int_out_endpoint_address = endpoint->bEndpointAddress;
			mcp_data->int_out_buffer = kmalloc(mcp_data->int_out_buffer_size, GFP_KERNEL);
			if (!mcp_data->int_out_buffer)
      {
				dev_err(&interface->dev, "Could not allocate interrupt out buffer");
				return -ENOMEM;
			}
    }
	}
  
  // check if endpoints were found
	if (!mcp_data->int_in_endpoint_address)
  {
		dev_err(&interface->dev, "Could not find int-in endpoint");
		return -ENODEV;
	}
	if (!mcp_data->int_out_endpoint_address)
  {
		dev_err(&interface->dev, "Could not find int-out endpoint");
		return -ENODEV;
	}

  // create USB request block (URB) for IN (reading)
	mcp_data->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!mcp_data->int_in_urb)
  {
		dev_err(&interface->dev, "No free urbs available\n");
    return -ENOMEM;
	}
  
	usb_fill_int_urb(
    mcp_data->int_in_urb, 
    mcp_usb_device,
		usb_rcvintpipe(mcp_usb_device, mcp_data->int_in_endpoint_address),
    mcp_data->int_in_buffer, 
    mcp_data->int_in_buffer_size,
		read_int_callback,      // completion callback function
    mcp_data,
		mcp_data->int_in_endpoint->bInterval);

  // create USB request block (URB) for OUT (writing)
	mcp_data->int_out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!mcp_data->int_out_urb)
  {
		dev_err(&interface->dev, "No free urbs available\n");
    return -ENOMEM;
	}
  
	usb_fill_int_urb(
    mcp_data->int_out_urb, 
    mcp_usb_device,
		usb_sndintpipe(mcp_usb_device, mcp_data->int_out_endpoint_address),
    mcp_data->int_out_buffer, 
    mcp_data->int_out_buffer_size,
		write_int_callback,      // completion callback function
    mcp_data,
		mcp_data->int_out_endpoint->bInterval);

  // store pointer to private data
	if (usb_set_intfdata(interface, mcp_data))
  {
    err("usb_set_intfdata failed");
  }
/*  
  if (usb_register_dev(interface, &mcp_data))
  {
    err("usb_register_dev failed");
  }*/
/*
	if (device_create_file(&interface->dev, &mcp_data))
  {
    return -ENOMEM;
  }
  */

	dev_info(&interface->dev, "MCP2210 chip now attached\n");
  
  return 0;
}
 
static inline void mcp_disconnect (struct usb_interface *interface)
{
  printk(KERN_INFO "Driver: disconnect()\n");

  kfree (dev->bulk_in_buffer);
  if (dev->bulk_out_buffer != NULL)
      usb_free_coherent (dev->udev, dev->bulk_out_size,
          dev->bulk_out_buffer,
          dev->write_urb->transfer_dma);
  usb_free_urb (dev->write_urb);
  kfree (dev);
}
  
static struct file_operations file_operation_callbacks =
{
  .owner = THIS_MODULE,
  .open = mcp_open,
  .release = mcp_close,
  .read = mcp_read,
  .write = mcp_write
};
 
static struct usb_driver mcp_usb_driver = 
{
  .name        = "mcp2210",
  .probe       = mcp_probe,
  .disconnect  = mcp_disconnect,
  .fops        = &file_operation_callbacks,
  .minor       = USB_MINOR_BASE,
  .id_table    = id_table,
};
  
static int __init mcp_init(void) /* Constructor */
{
	if (usb_register(&mcp_usb_driver) < 0)
  {
    err("usb_register failed");
    return -1;
  }
  
  // get a new device number with major and minor, first minor is 0
  if (alloc_chrdev_region(&device_number, 0, 1, "MCP2210") < 0)
  {
    err("alloc_chrdev_region failed");
    return -1;
  }
  
  // create device class
  if ((device_class = class_create(THIS_MODULE, "chardrv")) == NULL)
  {
    unregister_chrdev_region(device_number, 1);
    err("class_create failed");
    return -1;
  }
  
  // create device
  if (device_create(device_class, NULL, device_number, NULL, "mcp2210") == NULL)
  {
    class_destroy(device_class);
    unregister_chrdev_region(device_number, 1);
    err("device_create failed");
    return -1;
  }
  
  // register file operations
  cdev_init(&character_device, &file_operation_callbacks);
  
  // add character device to system
  if (cdev_add(&character_device, device_number, 1) == -1)
  {
    device_destroy(device_class, device_number);
    class_destroy(device_class);
    unregister_chrdev_region(device_number, 1);
    err("cdev_add failed");
    return -1;
  }
  
  printk(KERN_INFO "MCP2210 chip registered");
  
  return 0;
}
 
static void __exit mcp_exit(void) /* Destructor */
{
  usb_deregister(&mcp_usb_driver);
  
  cdev_del(&character_device);
  device_destroy(device_class, device_number);
  class_destroy(device_class);
  unregister_chrdev_region(device_number, 1);
  printk(KERN_INFO "MCP2210 chip unregistered");
}
 
module_init(mcp_init);
module_exit(mcp_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Benjamin Maier");
MODULE_DESCRIPTION("MCP2210 USB driver");
