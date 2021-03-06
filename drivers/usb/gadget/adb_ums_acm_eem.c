/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * Copyright (C) 2009 Samsung Electronics, Seung-Soo Yang
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "u_serial.h"
#include "f_mass_storage.h"
#include "f_adb.h"
#include "f_acm.h"

#include "f_mtp.h"
#include "fsa9480_i2c.h"
#include "f_rndis.h"

#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
//#undef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
#endif

#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
#include "f_eem.h"
#endif
#include "u_ether.h"
#include "gadget_chips.h"

#define USB_ANDROID_PRINT
#ifdef USB_PRINT
#undef USB_PRINT
#endif

#ifdef USB_ANDROID_PRINT
#define USB_PRINT(fmt...) do { printk(KERN_DEBUG fmt); } while(0)
#else
#define USB_PRINT(fmt...) do { } while(0)
#endif

MODULE_AUTHOR("SeungSoo Yang");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

#define DRIVER_DESC		"Android Composite USB"
#define DRIVER_VERSION	__DATE__
/* if you want to use VTP function, please enable below Feature : VTP_MODE*/
//#define VTP_MODE

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x04e8	/* Samsung */

#if defined (CONFIG_USB_IAD)
#define KIES_PRODUCT_ID 	0x685E
#else
#define KIES_PRODUCT_ID 	0x6877
#endif
#define MTP_PRODUCT_ID 	0x68A9	

#if defined(CONFIG_USB_IAD)
#define ADB_PRODUCT_ID 	0x685E	// new driver
#else
#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
#define ADB_PRODUCT_ID 	0x6850	// searsburg
#else
#define ADB_PRODUCT_ID 	0x681C	// shrewbury
#endif
#endif

#define PRODUCT_ID		0x681D	// UMS only mode

#if defined(CONFIG_USB_IAD)
#define RNDIS_PRODUCT_ID	0x6863
#else
#define RNDIS_PRODUCT_ID	0x6881
#endif

#define RNDIS_VENDOR_NUM	0x0525	/* NetChip */
#define RNDIS_PRODUCT_NUM	0xa4a2	/* Ethernet/RNDIS Gadget */
#define CDC_VENDOR_NUM		0x0525	/* NetChip */
#define CDC_PRODUCT_NUM		0xa4a1	/* Linux-USB Ethernet Gadget */
extern void ap_usb_power_on(int set_vaue);

void get_usb_serial(char *usb_serial_number)
{
	char temp_serial_number[13] = {0};

	u32 serial_number=0;
	
	serial_number = (system_serial_high << 16) + (system_serial_low >> 16);

#if defined(CONFIG_M115S)
	sprintf(temp_serial_number,"M115%08x",serial_number);
#else
	sprintf(temp_serial_number,"M110%08x",serial_number);
#endif
	strcpy(usb_serial_number,temp_serial_number);
}

/* 
 * Originally, adbd will enable adb function at booting 
 * if the value of persist.service.adb.enable is 1
 * ADB_ENABLE_AT_BOOT intends to enable adb function 
 * in case of no enabling by adbd or improper RFS
 */
#define ADB_ENABLE_AT_BOOT	0

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int product_id;
	int adb_product_id;
	int rndis_product_id;
	int mtp_product_id;
	int kies_product_id;
	int version;
	int adb_enabled;
	int nluns;
};

static atomic_t adb_enable_excl;
static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX	0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Samsung Electronics",
#if defined(CONFIG_M115S)
	[STRING_PRODUCT_IDX].s = "SHW-M115S",
#else
	[STRING_PRODUCT_IDX].s = "SHW-M110S",
#endif
	[STRING_SERIAL_IDX].s = "000000000000",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};
static u8 hostaddr[ETH_ALEN];

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass		  = USB_CLASS_PER_INTERFACE,
#if defined(CONFIG_USB_IAD)
	.bDeviceClass		  = USB_CLASS_MISC,
	.bDeviceSubClass	  = 2,
	.bDeviceProtocol	  = 1,
#else
	.bDeviceSubClass	  = 0x00,//US_SC_SCSI,
	.bDeviceProtocol	  = 0x00,//US_PR_BULK,
#endif
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

//static struct usb_device_descriptor eth_device_desc = {
//	.bDeviceClass =		USB_CLASS_COMM,
//	.bDeviceSubClass =	0,
//	.bDeviceProtocol =	0,
//	.idVendor =		__constant_cpu_to_le16 (CDC_VENDOR_NUM),
//	.idProduct =		__constant_cpu_to_le16 (CDC_PRODUCT_NUM),
//};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

void android_usb_set_connected(int connected)
{
	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
}

#if ADB_ENABLE_AT_BOOT
static void enable_adb(struct android_dev *dev, int enable);
#endif

static ssize_t show_serial_num
(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
)
{
	char usb_serial_number[13] = {0,};

	get_usb_serial(usb_serial_number);

	return snprintf(buf, PAGE_SIZE, "%s\n", usb_serial_number);
}

static DEVICE_ATTR(serial_num, S_IRUGO | S_IRUSR, show_serial_num, NULL);

static int __init android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	//ret = acm_bind_config(c, 0);

	ret = acm_function_add(c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to acm_function_add()\n", __func__);
		return ret;
	}
#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
	ret = eem_function_add(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to eem_function_add()\n", __func__);
		return ret;
	}
#endif

	ret = mass_storage_function_add(c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to ums_function_add()\n", __func__);
		return ret;
	}

	ret = adb_function_add(c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to adb_function_add()\n", __func__);
		return ret;
	}

//	ret = mtp_function_add(c);
//	if (ret) {
//		printk(KERN_DEBUG "[%s] Fail to mtp_function_add()\n", __func__);
//		return ret;
//	}

	ret = rndis_bind_config(c, hostaddr);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to rndis_bind_config()\n", __func__);
		return ret;
	}
	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
return ret;
}


static int ums_only_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	dev->cdev->desc.idProduct = __constant_cpu_to_le16(dev->product_id);
	dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
	dev->cdev->desc.bDeviceSubClass = 0x00;//US_SC_SCSI;
	dev->cdev->desc.bDeviceProtocol = 0x00;//US_PR_BULK;

	ret = mass_storage_function_config_changed(dev->cdev, c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to mass_storage_function_config_changed()\n", __func__);
		return ret;
	}	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return ret;
}

static int acm_eem_ums_adb_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	dev->cdev->desc.idProduct =	__constant_cpu_to_le16(dev->adb_product_id);
#if defined(CONFIG_USB_IAD)
	dev->cdev->desc.bDeviceClass	  = USB_CLASS_MISC;
	dev->cdev->desc.bDeviceSubClass   = 0x02;
	dev->cdev->desc.bDeviceProtocol   = 0x01;	
#else
	dev->cdev->desc.bDeviceClass	  = USB_CLASS_COMM;
	dev->cdev->desc.bDeviceSubClass   = 0x00;
	dev->cdev->desc.bDeviceProtocol   = 0x00;
#endif
	
#if defined(CONFIG_USB_IAD)
	ret = mass_storage_function_config_changed(dev->cdev, c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to mass_storage_function_config_changed()\n", __func__);
		return ret;
	}
	ret = acm_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to acm_function_config_changed()\n", __func__);
		return ret;
	}
#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
	ret = eem_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to eem_function_config_changed()\n", __func__);
		return ret;
	}
#endif
	ret = adb_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to adb_function_config_changed()\n", __func__);
		return ret;
	}
#else	
	ret = acm_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to acm_function_config_changed()\n", __func__);
		return ret;
	}
#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
	ret = eem_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to eem_function_config_changed()\n", __func__);
		return ret;
	}
#endif
	ret = mass_storage_function_config_changed(dev->cdev, c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to mass_storage_function_config_changed()\n", __func__);
		return ret;
	}
	ret = adb_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to adb_function_config_changed()\n", __func__);
		return ret;
	}
#endif
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return ret;
}

static int acm_ums_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	dev->cdev->desc.idProduct =	__constant_cpu_to_le16(dev->kies_product_id);
#if defined(CONFIG_USB_IAD)
	dev->cdev->desc.bDeviceClass	  = USB_CLASS_MISC;
	dev->cdev->desc.bDeviceSubClass   = 0x02;
	dev->cdev->desc.bDeviceProtocol   = 0x01;	
#else
	dev->cdev->desc.bDeviceClass	  = USB_CLASS_COMM;
	dev->cdev->desc.bDeviceSubClass   = 0x00;
	dev->cdev->desc.bDeviceProtocol   = 0x00;
#endif
	
#if defined(CONFIG_USB_IAD)
	ret = mass_storage_function_config_changed(dev->cdev, c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to mass_storage_function_config_changed()\n", __func__);
		return ret;
	}
	ret = acm_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to acm_function_config_changed()\n", __func__);
		return ret;
	}
#else	
	ret = acm_function_config_changed(dev->cdev, c);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to acm_function_config_changed()\n", __func__);
		return ret;
	}

	ret = mass_storage_function_config_changed(dev->cdev, c, dev->nluns);
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to mass_storage_function_config_changed()\n", __func__);
	return ret;
}
#endif
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return ret;	
	
}

static int mtp_only_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	dev->cdev->desc.idProduct = __constant_cpu_to_le16(dev->mtp_product_id);
	//dev->cdev->desc.bDeviceClass	  = USB_CLASS_STILL_IMAGE;
	//dev->cdev->desc.bDeviceSubClass   = 0x06;
	//dev->cdev->desc.bDeviceProtocol   = 0x01;

	dev->cdev->desc.bDeviceClass	  = 0x00;
	dev->cdev->desc.bDeviceSubClass   = 0x00;
	dev->cdev->desc.bDeviceProtocol   = 0x01;
	
	ret = mtp_function_config_changed(dev->cdev, c);
	if (ret) {
		printk("[%s] Fail to mtp_function_config_changed()\n", __func__);
		return ret;
	}
	return ret;
}

static int rndis_only_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	dev->cdev->desc.idProduct = __constant_cpu_to_le16(RNDIS_PRODUCT_ID);
	dev->cdev->desc.bDeviceClass	  = USB_CLASS_COMM;
	dev->cdev->desc.bDeviceSubClass   = 0;
	dev->cdev->desc.bDeviceProtocol   = 0;
	
	ret = rndis_function_config_changed(dev->cdev, c);
	if (ret) {
		printk("[%s] Fail to rndis_function_config_changed()\n", __func__);
		return ret;
	}
	return ret;
}

#ifdef CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM
#define	ANDROID_DEBUG_CONFIG_STRING "ACM + EEM + UMS + ADB (Debugging mode)"
#else
#define	ANDROID_DEBUG_CONFIG_STRING "ACM + UMS + ADB (Debugging mode)"
#endif
#define	ANDROID_NO_DEBUG_KIES_CONFIG_STRING "ACM + UMS (No Debugging mode)"
#define	ANDROID_NO_DEBUG_MTP_CONFIG_STRING "MTP Olny (No Debugging mode)"
#define	ANDROID_NO_DEBUG_UMS_CONFIG_STRING "UMS Only (No debugging mode)"
#define	ANDROID_NO_DEBUG_RNDIS_CONFIG_STRING "RNDIS Only (No debugging mode)"

static struct usb_configuration android_config  = {
	.label		= ANDROID_NO_DEBUG_UMS_CONFIG_STRING,
	.bind		= android_bind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static struct usb_configuration ums_only_config  = {
	.label		= ANDROID_NO_DEBUG_UMS_CONFIG_STRING,
	.bind		= ums_only_bind_config,
	.bConfigurationValue = 2,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static struct usb_configuration acm_eem_ums_adb_config  = {
	.label		= ANDROID_DEBUG_CONFIG_STRING,
	.bind		= acm_eem_ums_adb_bind_config,
	.bConfigurationValue = 3,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static struct usb_configuration acm_ums_config  = {
	.label		= ANDROID_NO_DEBUG_KIES_CONFIG_STRING,
	.bind		= acm_ums_bind_config,
	.bConfigurationValue = 4,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static struct usb_configuration mtp_only_config  = {
	.label		= ANDROID_NO_DEBUG_MTP_CONFIG_STRING,
	.bind		= mtp_only_bind_config,
	.bConfigurationValue = 5,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static struct usb_configuration rndis_only_config  = {
	.label		= ANDROID_NO_DEBUG_RNDIS_CONFIG_STRING,
	.bind		= rndis_only_bind_config,
	.bConfigurationValue = 6,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x30, /*  x2 = 160ma */
};

static int __init android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum;
	int			id;
	int			ret;
	int			status;
	char usb_serial_number[13] = {0,};

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	
//	eth_device_desc.idVendor = cpu_to_le16(RNDIS_VENDOR_NUM);
//	eth_device_desc.idProduct = cpu_to_le16(RNDIS_PRODUCT_NUM);
	
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;

	get_usb_serial(usb_serial_number);
	strings_dev[STRING_SERIAL_IDX].id = id;

	if( (usb_serial_number[0] + 
		 usb_serial_number[1] + 
		 usb_serial_number[2]) != 0 )
		strcpy((char *)(strings_dev[STRING_SERIAL_IDX].s), usb_serial_number);

#if defined(CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM)
	printk(KERN_DEBUG "[ADB_EEM_UMS_ACM] string_dev = %s \n",strings_dev[STRING_SERIAL_IDX].s);
#else
	printk(KERN_DEBUG "[ADB_UMS_ACM] string_dev = %s \n",strings_dev[STRING_SERIAL_IDX].s);
#endif

	device_desc.iSerialNumber = id;
	device_desc.bcdDevice = cpu_to_le16(0x0400);

#if 0
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
	{
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	}
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}
#endif
	
	if (gadget_is_otg(cdev->gadget)) 
		android_config.descriptors = otg_desc;
	
// Fix: USBCV Ch9 RemoteWakeup Enable/Disable Test.
#if 0	
	if (gadget->ops->wakeup)
		android_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
#endif

	/* set up network link layer */
	status = gether_setup(cdev->gadget, hostaddr);
	if (status < 0)
		return status;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		goto acm_fail;
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	
#if defined(CONFIG_USB_IAD)
	ret = usb_change_config(dev->cdev, &acm_ums_config);			
#else	
	ret = usb_change_config(dev->cdev, &ums_only_config);
#endif
	if (ret) {
		printk(KERN_DEBUG "[%s] Fail to usb_change_config()\n", __func__);
		return ret;
	}
	
#if ADB_ENABLE_AT_BOOT
	printk(KERN_DEBUG "[%s] Enabling adb function at booting\n", __func__);
	enable_adb(dev, USBSTATUS_ADB);
#endif

	// /sys/devices/platform/s3c-usbgadget/gadget/serial_num
	if (device_create_file(&cdev->gadget->dev, &dev_attr_serial_num) < 0)
	{
		printk(KERN_DEBUG "Failed to create device file(%s)!\n", dev_attr_serial_num.attr.name);
	}

	INFO(cdev, "%s, version: " DRIVER_VERSION "\n", DRIVER_DESC);

	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return 0;
	
acm_fail:
	gether_cleanup();
	return ret;
}

/*
	Google gadget doesn't support module type building
static int __exit android_unbind(struct usb_composite_dev *cdev)
{
	gserial_cleanup();
	return 0;
}
*/

static struct usb_composite_driver android_usb_driver = {
	//driver.name refered by S3C-UDC for setting config_gadget_driver
#if defined(CONFIG_USB_ANDROID_ADB_UMS_ACM_EEM)
	.name		= "android_adb_eem_ums_acm",
#else
	.name		= "android_adb_ums_acm",
#endif
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
//	.unbind 	= __exit_p(android_unbind),
};

int currentusbstatus=0;
extern int oldusbstatus;
int UmsCDEnable=0;
int ums_mount_status = 0;
int askonstatus = 0;
int inaskonstatus=0;
static int prev_status_before_adb;  // previous USB setting before using ADB
static int prev_enable_status;  // previous USB setting
extern int mtp_mode_on;

static void enable_adb(struct android_dev *dev, int enable)
{
	int ret;
	printk(KERN_DEBUG "[USB:ANDRIOD] %s. enable:%d, dev->adb_enabled:%d\n", __func__, enable, dev->adb_enabled);

	if (enable != dev->adb_enabled)	{
		prev_enable_status = dev->adb_enabled;
		if (dev->cdev && dev->cdev->gadget) {
			usb_gadget_disconnect(dev->cdev->gadget);
		}
askonstatus=0;

recheck:
		oldusbstatus = currentusbstatus;
		currentusbstatus=enable;

		ums_mount_status=0;

		/* set product ID to the appropriate value */
		if (enable== USBSTATUS_SAMSUNG_KIES)
		{
			printk("[enable_adb]USBSTATUS_SAMSUNG_KIES\n");
			mtp_mode_on = 0;
			UmsCDEnable=0;
			ret = usb_change_config(dev->cdev, &acm_ums_config);
			if (ret) {
				printk("[%s] Fail to acm_ums_config()\n", __func__);
			}
			dev->adb_enabled = enable;
		}
#if 0		
		else if(enable == USBSTATUS_MTPONLY)
		{
			printk("[enable_adb]USBSTATUS_MTPONLY\n");
			mtp_mode_on = 1;
			ret = usb_change_config(dev->cdev, &mtp_only_config);
			if (ret) {
				printk("[%s] Fail to mtp_only_config()\n", __func__);
			}
			dev->adb_enabled = enable;
		}
		else if(enable == USBSTATUS_ASKON)
		{
			printk("[enable_adb]USBSTATUS_ASKON\n");
			mtp_mode_on = 0;
			askonstatus=1;
			dev->adb_enabled = enable;
		}
		else 
#endif
		else if(enable == USBSTATUS_VTP) 
		{
			printk("[enable_adb]USBSTATUS_VTP\n");
#ifdef VTP_MODE
		mtp_mode_on = 0;
		UmsCDEnable=1;
		ret = usb_change_config(dev->cdev, &ums_only_config);
		if (ret) {
			printk("[%s] Fail to ums_only_config()\n", __func__);
				}
		dev->adb_enabled = enable;
#else
		mtp_mode_on = 0;
        ret = usb_change_config(dev->cdev, &rndis_only_config);
			if (ret) {
				printk("[%s] Fail to rndis_only_config()\n", __func__);
			}
			dev->adb_enabled = enable;
#endif
		}
		else if(enable == USBSTATUS_UMS)
		{
			printk("[enable_adb]USBSTATUS_UMS\n");
			mtp_mode_on = 0;
			UmsCDEnable=0;
#if 0			
			if(prev_enable_status == USBSTATUS_ADB && prev_status_before_adb != USBSTATUS_UMS) {
				printk("[USB] %s - prev_status(0x%02x), prev_status_before_adb setting(0x%02x)\n",
						__func__, prev_enable_status, prev_status_before_adb);
				enable = prev_status_before_adb;  // set previous status
				prev_enable_status = prev_status_before_adb = 0; //reset
				goto recheck;
				}
#endif
			ret = usb_change_config(dev->cdev, &ums_only_config);
			if (ret) {
				printk(KERN_DEBUG "[%s] Fail to ums_only_config()\n", __func__);
			}
			dev->adb_enabled = enable;
		}
		else if(enable == USBSTATUS_ADB)
		{
			printk("[enable_adb]USBSTATUS_ADB\n");
			mtp_mode_on = 0;
			prev_status_before_adb = prev_enable_status;
			ret = usb_change_config(dev->cdev, &acm_eem_ums_adb_config);
			if (ret) {
				printk(KERN_DEBUG "[%s] Fail to acm_eem_ums_adb_config()\n", __func__);
			}
			dev->adb_enabled = enable;
		}

/*
	for reenumeration in case of booting-up connected with host
	because if connected, host don't enumerate 
*/
		if(!mtp_mode_on)
			ap_usb_power_on(1);
		else
			ap_usb_power_on(0);

		if (dev->cdev && dev->cdev->gadget ) 
		{
			//usb_gadget_disconnect(dev->cdev->gadget);
			mdelay(5);
			usb_gadget_connect(dev->cdev->gadget);
		}
	}
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
}

static void disable_mtp(struct android_dev *dev)
{
	if (dev->cdev && dev->cdev->gadget) {
		usb_gadget_disconnect(dev->cdev->gadget);
	}

#ifdef VODA
    mdelay(1000);
	ret = usb_change_config(dev->cdev, &acm_only_config);
#else
	ap_usb_power_on(0);
#endif

	if (dev->cdev && dev->cdev->gadget ) {
		 usb_gadget_connect(dev->cdev->gadget);
		}
}

extern void UsbIndicator(u8 state);

static void enable_askon(struct android_dev *dev, int enable)
{
	int ret;

		ums_mount_status=0;

	
	if (dev->cdev && dev->cdev->gadget) {
	usb_gadget_disconnect(dev->cdev->gadget);
	}

	oldusbstatus = currentusbstatus;
	currentusbstatus=enable;

	/* set product ID to the appropriate value */
#if 0
	if (enable== USBSTATUS_SAMSUNG_KIES)
	{
		printk("[enable_askon]USBSTATUS_SAMSUNG_KIES\n");
		mtp_mode_on = 1;
		inaskonstatus = 1;
		UsbIndicator(1);
		ret = usb_change_config(dev->cdev, &acm_mtp_config);
		if (ret) {
			printk("[%s] Fail to acm_mtp_config()\n", __func__);
		}
	}
	else if(enable == USBSTATUS_MTPONLY)
	{
		printk("[enable_askon]USBSTATUS_MTPONLY\n");
		mtp_mode_on = 1;
		inaskonstatus = 1;			
		UsbIndicator(1);
		ret = usb_change_config(dev->cdev, &mtp_only_config);
		if (ret) {
			printk("[%s] Fail to mtp_only_config()\n", __func__);
		}
	}
	else 
#endif
	if(enable == USBSTATUS_VTP) 
	{
		printk("[enable_askon]USBSTATUS_VTP\n");
		inaskonstatus = 1;		
#ifdef VTP_MODE
		mtp_mode_on = 0;
		UsbIndicator(1);
		UmsCDEnable=1;
		ret = usb_change_config(dev->cdev, &ums_only_config);
		if (ret) {
			printk("[%s] Fail to ums_only_config()\n", __func__);
		}
#else
		mtp_mode_on = 0;
		UsbIndicator(1);
		ret = usb_change_config(dev->cdev, &rndis_only_config);
		if (ret) {
			printk("[%s] Fail to rndis_only_config()\n", __func__);
		}
#endif
	}
	else if(enable == USBSTATUS_UMS)
	{
		printk("[enable_askon]USBSTATUS_UMS\n");
		mtp_mode_on = 0;
		inaskonstatus = 1;			
		UmsCDEnable=0;
		UsbIndicator(1);

		ret = usb_change_config(dev->cdev, &ums_only_config);
		if (ret) {
			printk("[%s] Fail to ums_only_config()\n", __func__);
		}
	}
/*
	for reenumeration in case of booting-up connected with host
	because if connected, host don't enumerate 
*/
	if(!mtp_mode_on)
		ap_usb_power_on(1);
	
	if (dev->cdev && dev->cdev->gadget ) {
          usb_gadget_connect(dev->cdev->gadget);
	}
}

void askon_gadget_disconnect(void)
{
	struct android_dev *dev = _android_dev;

	if(askonstatus)
		usb_gadget_disconnect(dev->cdev->gadget);

}

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}
    printk(KERN_DEBUG "enabling adb\n");
	enable_adb(_android_dev, USBSTATUS_ADB);
	atomic_dec(&adb_enable_excl);
	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);

	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}
	enable_adb(_android_dev, USBSTATUS_SAMSUNG_KIES);
	atomic_dec(&adb_enable_excl);
	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return 0;
}

int usb_mtp_select(int disable)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
	atomic_dec(&adb_enable_excl);
	return -EBUSY;
	}

	disable_mtp(_android_dev);
	atomic_dec(&adb_enable_excl);
	return 0;
}

int usb_switch_select(int enable)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
	atomic_dec(&adb_enable_excl);
	return -EBUSY;
	}

	enable_adb(_android_dev, enable);
	atomic_dec(&adb_enable_excl);
	return 0;
}

int askon_switch_select(int enable)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
	atomic_dec(&adb_enable_excl);
	return -EBUSY;
	}

	enable_askon(_android_dev, enable);
	atomic_dec(&adb_enable_excl);
	return 0;
}


static const struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	if (pdata) {
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->adb_product_id)
			dev->adb_product_id = pdata->adb_product_id;

		if (pdata->mtp_product_id)
			dev->mtp_product_id = pdata->mtp_product_id;
		
		if (pdata->kies_product_id)
			dev->kies_product_id = pdata->kies_product_id;

		if (pdata->rndis_product_id)
			dev->rndis_product_id = pdata->rndis_product_id;

		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		dev->nluns = pdata->nluns;
	}
	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "C110 Android USB", },
	.probe = android_probe,
};

static int __init init(void)
{
	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);

	struct android_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	dev->adb_product_id = ADB_PRODUCT_ID;
	dev->mtp_product_id = MTP_PRODUCT_ID;
	dev->kies_product_id = KIES_PRODUCT_ID;
	dev->rndis_product_id = RNDIS_PRODUCT_ID;

	_android_dev = dev;
	USB_PRINT("[USB:ANDROID] %s USB_Debuging mode: %d ++\n", __func__, dev->adb_enabled);

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		return ret;
	ret = misc_register(&adb_enable_device);
	if (ret) {
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}
	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		misc_deregister(&adb_enable_device);
		platform_driver_unregister(&android_platform_driver);
	}
	
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	USB_PRINT("[USB:ANDROID] %s ++\n", __func__);
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
	gserial_cleanup();
	USB_PRINT("[USB:ANDROID] %s --\n", __func__);
}
module_exit(cleanup);
