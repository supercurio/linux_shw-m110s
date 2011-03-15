/* drivers/input/keyboard/s3c-keypad.c
 *
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-keypad.h>
#ifdef CONFIG_CPU_FREQ
#include <mach/cpu-freq-v210.h>
#endif
#include <plat/pm.h>

#include "s3c-keypad.h"

#ifndef FEATURE_FTM_SLEEP
#define FEATURE_FTM_SLEEP
#endif

#ifdef FEATURE_FTM_SLEEP
extern unsigned char ftm_sleep;
#endif

//#define PM_DEBUG
#ifdef PM_DEBUG
extern void s5pc11x_pm_do_dump(void);
#endif
extern unsigned long s5pc11x_pm_get_wakeup_src(void);
extern void s5pc11x_pm_clear_wakeup_src(void);

#define KEY_KERN_DEBUG	KERN_DEBUG

#define DEVICE_NAME "s3c-keypad"

#define	SUBJECT	"s3c_keypad.c"
#define P(format,...)\
    printk ("[ "SUBJECT " (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#define FI \
    printk ("[ "SUBJECT " (%s,%d) ] " "%s - IN" "\n", __func__, __LINE__, __func__);
#define FO \
    printk ("[ "SUBJECT " (%s,%d) ] " "%s - OUT" "\n", __func__, __LINE__, __func__);

static struct timer_list power_on_timer;
static struct timer_list power_off_timer;

static struct clk *keypad_clock;

struct class *keypad_class;
struct device *keypad_test_dev;
struct input_dev *power_dev;
struct input_dev *hold_event_dev;

static u8 test_onoff = 0; //0:not-test    1: test
static u8 activation_onoff = 1; // 0:deactivate   1:activate
static u8 home_on = 0;
static u8 volup_on = 0;
static u8 voldn_on = 0;
static u8 power_on = 0;
static u32 keymask[KEYPAD_COLUMNS];
static u32 prevmask[KEYPAD_COLUMNS];
unsigned long wakeup_src=0;

static ssize_t keypad_short_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	u32 power_short, home_short, voldn_short, volup_short;
	
	printk(KEY_KERN_DEBUG "called %s \n",__func__);

	power_short = ~(readl(S5PV210_GPH2DAT)) & (1 << 6);
	home_short = ~(readl(S5PV210_GPH3DAT)) & (1 << 0);
	voldn_short = ~(readl(S5PV210_GPH3DAT)) & (1 << 1);
	volup_short = ~(readl(S5PV210_GPH3DAT)) & (1 << 2);
	if(power_short|home_short|voldn_short|volup_short)
		return sprintf(buf,"1\n"); //key short
	else
		return sprintf(buf,"0\n"); // key non-short
}

static ssize_t keypad_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{	
	printk(KEY_KERN_DEBUG "called %s \n",__func__);
	return sprintf(buf,"%d\n",test_onoff);
}

static ssize_t keypad_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{	
	printk(KEY_KERN_DEBUG "called %s\n",__func__);
	sscanf(buf, "%d", &test_onoff);
	printk(KEY_KERN_DEBUG "keypad test = %d\n",test_onoff);
	return size;
}

static ssize_t keypad_hold_event_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{	
	printk(KEY_KERN_DEBUG "called %s\n",__func__);
	input_report_key(hold_event_dev,KEYCODE_POWER,1);
	msleep(20);
	input_report_key(hold_event_dev,KEYCODE_POWER,0);
	return size;
}

static ssize_t keypad_activation_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{	
	printk(KEY_KERN_DEBUG "called %s \n",__func__);
	return sprintf(buf,"%d\n",activation_onoff);
}

static ssize_t keypad_activation_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{	
	printk(KEY_KERN_DEBUG "called %s\n",__func__);
	sscanf(buf, "%d", &activation_onoff);
	printk(KEY_KERN_DEBUG "deactivation test = %d\n",activation_onoff);
	return size;
}

static DEVICE_ATTR(keypad_short, S_IRUGO, keypad_short_show, NULL);
static DEVICE_ATTR(keypad_test, S_IRUGO | S_IWUGO, keypad_test_show, keypad_test_store);
static DEVICE_ATTR(keypad_hold_event, S_IRUGO | S_IWUGO, NULL, keypad_hold_event_store);
static DEVICE_ATTR(keypad_activation, S_IRUGO | S_IWUGO, keypad_activation_show, keypad_activation_store);

static void power_on_timer_handler(unsigned long data)
{
		power_on = 1;
		if(test_onoff)
		{
			input_report_key(power_dev,KEYCODE_CLEAR,1);
			printk(KEY_KERN_DEBUG "key Pressed : clear %d\n",KEYCODE_CLEAR);
		}
		else
		{
#ifdef CONFIG_CPU_FREQ
			set_dvfs_target_level(LEV_800MHZ);
#endif
			input_report_key(power_dev,KEYCODE_POWER,1);
			printk(KEY_KERN_DEBUG "key Pressed : power %d\n",KEYCODE_POWER);
		}
}

static void power_off_timer_handler(unsigned long data)
{
		power_on = 0;
		if(test_onoff)
		{
			input_report_key(power_dev,KEYCODE_CLEAR,0);
			printk(KEY_KERN_DEBUG "key Released : clear %d\n",KEYCODE_CLEAR);
		}
		else
		{
			input_report_key(power_dev,KEYCODE_POWER,0);
			printk(KEY_KERN_DEBUG "key Released : power %d\n",KEYCODE_POWER);
		}
}

static irqreturn_t s3c_keygpio_pwr_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	power_dev = pdata->dev;

	if(!activation_onoff)
		return IRQ_HANDLED;
	
	key_status = (readl(S5PV210_GPH2DAT)) & (1 << 6);
	
	if(key_status){
		del_timer(&power_on_timer);
		if(!power_on)
			return IRQ_HANDLED;
		mod_timer(&power_off_timer,jiffies + 6);//30ms delay, 5ms per 1
	}
	else{
		del_timer(&power_off_timer);
		if(power_on)
			return IRQ_HANDLED;
		mod_timer(&power_on_timer,jiffies + 4);//20ms delay, 5ms per 1
	}

        return IRQ_HANDLED;
}

extern void TSP_forced_release_forOKkey(void); 

static irqreturn_t s3c_keygpio_home_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	if(!activation_onoff)
		return IRQ_HANDLED;

#ifdef CONFIG_CPU_FREQ
	set_dvfs_target_level(LEV_800MHZ);
#endif
	key_status = (readl(S5PV210_GPH3DAT)) & (1 << 0);
	
	if(key_status){
		if(!home_on)
			return IRQ_HANDLED;

		home_on = 0;
		if(test_onoff){				
			input_report_key(dev,KEYCODE_ENTER,0);
			printk(KEY_KERN_DEBUG "key Released : enter %d\n",KEYCODE_ENTER);
		}
		else{
			TSP_forced_release_forOKkey();			
			input_report_key(dev,KEYCODE_HOME,0);		
			printk(KEY_KERN_DEBUG "key Released : home %d\n",KEYCODE_HOME);
		}
	}
	else{
		if(home_on)
			return IRQ_HANDLED;
		
		home_on = 1;
		if(test_onoff){				
			input_report_key(dev,KEYCODE_ENTER,1);
			printk(KEY_KERN_DEBUG "key Pressed : enter %d\n",KEYCODE_ENTER);
		}
		else{
			input_report_key(dev,KEYCODE_HOME,1);
			printk(KEY_KERN_DEBUG "key Pressed : home %d\n",KEYCODE_HOME);
		}
	}

        return IRQ_HANDLED;
}


static irqreturn_t s3c_keygpio_voldn_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	if(!activation_onoff)
		return IRQ_HANDLED;

	key_status = (readl(S5PV210_GPH3DAT)) & (1 << 1);
	
	if(key_status){
		if(!voldn_on)
			return IRQ_HANDLED;
		
		voldn_on = 0;
		input_report_key(dev,KEYCODE_VOLUME_DOWN,0);
		printk(KEY_KERN_DEBUG "key Released : vol down %d\n",KEYCODE_VOLUME_DOWN);
#ifdef PM_DEBUG
		s5pc11x_pm_do_dump();
#endif
	}
	else{
		if(voldn_on)
			return IRQ_HANDLED;
		
		voldn_on = 1;
		input_report_key(dev,KEYCODE_VOLUME_DOWN,1);
		printk(KEY_KERN_DEBUG "key Pressed : vol down %d\n",KEYCODE_VOLUME_DOWN);
	}

        return IRQ_HANDLED;
}

static irqreturn_t s3c_keygpio_volup_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	if(!activation_onoff)
		return IRQ_HANDLED;
	
	key_status = (readl(S5PV210_GPH3DAT)) & (1 << 2);
	
	if(key_status){
		if(!volup_on)
			return IRQ_HANDLED;
		
		volup_on = 0;
		input_report_key(dev,KEYCODE_VOLUME_UP,0);
		printk(KEY_KERN_DEBUG "key Released : vol up %d\n",KEYCODE_VOLUME_UP);
	}
	else{
		if(volup_on)
			return IRQ_HANDLED;
		
		volup_on = 1;
		input_report_key(dev,KEYCODE_VOLUME_UP,1);
		printk(KEY_KERN_DEBUG "key Pressed : vol up %d\n",KEYCODE_VOLUME_UP);
	}

        return IRQ_HANDLED;
}

static int s3c_keygpio_isr_setup(void *pdev)
{
	int ret;

	s3c_gpio_setpull(S5PV210_GPH3(0), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(24), IRQ_TYPE_EDGE_BOTH);  
        ret = request_irq(IRQ_EINT(24), s3c_keygpio_home_isr, IRQF_SAMPLE_RANDOM,
                "key home", (void *) pdev);
        if (ret) {
                printk(KERN_ERR "request_irq failed (IRQ_KEYPAD (home)) !!!\n");
                ret = -EIO;
		return ret;
        }

	s3c_gpio_setpull(S5PV210_GPH3(1), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(25), IRQ_TYPE_EDGE_BOTH);  
        ret = request_irq(IRQ_EINT(25), s3c_keygpio_voldn_isr, IRQF_SAMPLE_RANDOM,
                "key voldn", (void *) pdev);
        if (ret) {
                printk(KERN_ERR "request_irq failed (IRQ_KEYPAD (voldn)) !!!\n");
                ret = -EIO;
		return ret;
        }

	s3c_gpio_setpull(S5PV210_GPH3(2), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(26), IRQ_TYPE_EDGE_BOTH);  
        ret = request_irq(IRQ_EINT(26), s3c_keygpio_volup_isr, IRQF_SAMPLE_RANDOM,
                "key volup", (void *) pdev);
        if (ret) {
                printk(KERN_ERR "request_irq failed (IRQ_KEYPAD (volup)) !!!\n");
                ret = -EIO;
		return ret;
	}
	
	s3c_gpio_setpull(S5PV210_GPH2(6), S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_EINT(22), IRQ_TYPE_EDGE_BOTH);  
        ret = request_irq(IRQ_EINT(22), s3c_keygpio_pwr_isr, IRQF_SAMPLE_RANDOM,
                "key power", (void *) pdev);
        if (ret) {
                printk(KERN_ERR "request_irq failed (IRQ_KEYPAD (power)) !!!\n");
                ret = -EIO;
        }
		
	return ret;

}

static int __init s3c_keypad_probe(struct platform_device *pdev)
{
	struct resource *res, *keypad_mem, *keypad_irq;
	struct input_dev *input_dev;
	struct s3c_keypad *s3c_keypad;
	int ret, size;
	int key, code;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev,"no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;

	keypad_mem = request_mem_region(res->start, size, pdev->name);
	if (keypad_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	key_base = ioremap(res->start, size);
	if (key_base == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		ret = -ENOMEM;
		goto err_map;
	}

	keypad_clock = clk_get(&pdev->dev, "keypad");
	if (IS_ERR(keypad_clock)) {
		dev_err(&pdev->dev, "failed to find keypad clock source\n");
		ret = PTR_ERR(keypad_clock);
		goto err_clk;
	}

	clk_enable(keypad_clock);
	
	s3c_keypad = kzalloc(sizeof(struct s3c_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!s3c_keypad || !input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, s3c_keypad);
	s3c_keypad->dev = input_dev;

	s3c_setup_keypad_cfg_gpio(KEYPAD_ROWS, KEYPAD_COLUMNS);

	/* create and register the input driver */
	set_bit(EV_KEY, input_dev->evbit);
	/*Commenting the generation of repeat events*/
	//set_bit(EV_REP, input_dev->evbit);
	s3c_keypad->nr_rows = KEYPAD_ROWS;
	s3c_keypad->no_cols = KEYPAD_COLUMNS;
	s3c_keypad->total_keys = MAX_KEYPAD_NR;

	for(key = 0; key < s3c_keypad->total_keys; key++){
		code = s3c_keypad->keycodes[key] = keypad_keycode[key];
		if(code<=0)
			continue;
		set_bit(code & KEY_MAX, input_dev->keybit);
	}
	set_bit(KEYCODE_ENTER & KEY_MAX, input_dev->keybit); //for factory key test
	set_bit(KEYCODE_CLEAR & KEY_MAX, input_dev->keybit); //for factory key test
	set_bit(KEYCODE_CAMERA & KEY_MAX, input_dev->keybit); //for AT cmd test
	set_bit(KEYCODE_POWER & KEY_MAX, input_dev->keybit);

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";
	
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_dev->keycode = keypad_keycode;

	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR "Unable to register s3c-keypad input device!!!\n");
		goto out;
	}

	keypad_class = class_create(THIS_MODULE, "keypad");
	if (IS_ERR(keypad_class)){
		pr_err("Failed to create class(keypad)!\n");
		}

	keypad_test_dev = device_create(keypad_class, NULL, 0, NULL, "test_cmd");
	if (IS_ERR(keypad_test_dev)){
		pr_err("Failed to create device(keypad_test_dev)!\n");
		}

	if (device_create_file(keypad_test_dev, &dev_attr_keypad_test) < 0){
		pr_err("Failed to create device file(%s)!\n", dev_attr_keypad_test.attr.name);
		}
	
	if (device_create_file(keypad_test_dev, &dev_attr_keypad_activation) < 0){
		pr_err("Failed to create device file(%s)!\n", dev_attr_keypad_activation.attr.name);
		}

	if (device_create_file(keypad_test_dev, &dev_attr_keypad_short) < 0){
		pr_err("Failed to create device file(%s)!\n", dev_attr_keypad_short.attr.name);
		}
		if (device_create_file(keypad_test_dev, &dev_attr_keypad_hold_event) < 0){
		pr_err("Failed to create device file(%s)!\n", dev_attr_keypad_hold_event.attr.name);
		}
	init_timer(&power_on_timer);
	power_on_timer.function = power_on_timer_handler;
	power_on_timer.data = (unsigned long)s3c_keypad;
	power_on_timer.expires = jiffies + HZ;
	init_timer(&power_off_timer);
	power_off_timer.function = power_off_timer_handler;
	power_off_timer.data = (unsigned long)s3c_keypad;
	power_off_timer.expires = jiffies + HZ;
	
	s3c_keygpio_isr_setup((void *)s3c_keypad);
	hold_event_dev = input_dev;
	
	printk(KEY_KERN_DEBUG DEVICE_NAME " Initialized\n");
	return 0;

out:
	free_irq(keypad_irq->start, input_dev);
	free_irq(keypad_irq->end, input_dev);

err_irq:
	input_free_device(input_dev);
	kfree(s3c_keypad);
	
err_alloc:
	clk_disable(keypad_clock);
	clk_put(keypad_clock);

err_clk:
	iounmap(key_base);

err_map:
	release_resource(keypad_mem);
	kfree(keypad_mem);

err_req:
	return ret;
}

static int s3c_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);
	writel(KEYIFCON_CLEAR, key_base+S3C_KEYIFCON);

	if(keypad_clock) {
		clk_disable(keypad_clock);
		clk_put(keypad_clock);
		keypad_clock = NULL;
	}

	input_unregister_device(input_dev);
	iounmap(key_base);
	kfree(pdev->dev.platform_data);

	del_timer(&power_on_timer);
	del_timer(&power_off_timer);
		
	printk(KEY_KERN_DEBUG DEVICE_NAME " Removed.\n");
	return 0;
}

#ifdef CONFIG_PM

static struct sleep_save s3c_keypad_save[] = {
	SAVE_ITEM(KEYPAD_ROW_GPIOCON),
	SAVE_ITEM(KEYPAD_COL_GPIOCON),
	SAVE_ITEM(KEYPAD_ROW_GPIOPUD),
	SAVE_ITEM(KEYPAD_COL_GPIOPUD),
};

static unsigned int keyifcon, keyiffc;

static int s3c_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	keyifcon = readl(key_base+S3C_KEYIFCON);
	keyiffc = readl(key_base+S3C_KEYIFFC);

	s3c_pm_do_save(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));
	clk_disable(keypad_clock);

	return 0;
}

static int s3c_keypad_resume(struct platform_device *dev)
{
	struct s3c_keypad          *s3c_keypad = (struct s3c_keypad *) platform_get_drvdata(dev);
      struct input_dev           *iDev = s3c_keypad->dev;

	printk(KEY_KERN_DEBUG "++++ %s\n", __FUNCTION__ );

	clk_enable(keypad_clock);

	wakeup_src = s5pc11x_pm_get_wakeup_src();
	printk(KEY_KERN_DEBUG "wakeup source %x \n",wakeup_src);

	if(activation_onoff){
		//if(wakeup_src & (WAKEUP_SRC_PWRBUTTON|WAKEUP_SRC_SDDET|WAKEUP_SRC_EARJACK))
		if(wakeup_src & (WAKEUP_SRC_PWRBUTTON|WAKEUP_SRC_SDDET))
		{
			input_report_key(iDev, KEYCODE_POWER, 1);
			udelay(5);
			input_report_key(iDev, KEYCODE_POWER, 0);
			printk(KEY_KERN_DEBUG "pbt,sddet report power key \n");
		}else if(wakeup_src & WAKEUP_SRC_HOME)
		{
			input_report_key(iDev, KEYCODE_HOME, 1);
			udelay(5);
			input_report_key(iDev, KEYCODE_HOME, 0);
			printk(KEY_KERN_DEBUG "report home key \n");
		}else if(wakeup_src & WAKEUP_SRC_RTC_TICK)
		{
			printk(KEY_KERN_DEBUG "rtc tick wakeup \n");
			if(ftm_sleep)
			{
				input_report_key(iDev, KEYCODE_POWER, 1);
				udelay(5);
				input_report_key(iDev, KEYCODE_POWER, 0);
				printk(KEY_KERN_DEBUG "fmt sleep mode, force wakeup \n");			
			}
		}else if(wakeup_src & WAKEUP_SRC_VOLDN) //LCD is on by report key -> why?
		{
			input_report_key(iDev, KEYCODE_VOLUME_DOWN, 1);
			udelay(5);
			input_report_key(iDev, KEYCODE_VOLUME_DOWN, 0);
			printk(KEY_KERN_DEBUG "report volume down key \n");
		}else if(wakeup_src & WAKEUP_SRC_VOLUP)
		{
			input_report_key(iDev, KEYCODE_VOLUME_UP, 1);
			udelay(5);
			input_report_key(iDev, KEYCODE_VOLUME_UP, 0);
			printk(KEY_KERN_DEBUG "report volume up key \n");
		}else 
		{
		    //send some event to android to start the full resume
		    //input_report_key(iDev, KEYCODE_UNKNOWN, 1);//ENDCALL up event
		    //udelay(5);
		    //input_report_key(iDev, KEYCODE_UNKNOWN, 0);//ENDCALL down event
		}
	}

	wakeup_src=0;
	s5pc11x_pm_clear_wakeup_src();

	//printk("H3C %x H2C %x \n",readl(S5PCV210_GPH3CON),readl(S5PV210_GPH2CON));

	s3c_pm_do_restore(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));
	
	printk(KEY_KERN_DEBUG "---- %s\n", __FUNCTION__ );
	return 0;
}
#else
#define s3c_keypad_suspend NULL
#define s3c_keypad_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver s3c_keypad_driver = {
	.probe		= s3c_keypad_probe,
	.remove		= s3c_keypad_remove,
	.suspend	= s3c_keypad_suspend,
	.resume		= s3c_keypad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-keypad",
	},
};

static int __init s3c_keypad_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_keypad_driver);
	
	if(!ret)
	   printk(KEY_KERN_DEBUG "S3C Keypad Driver\n");

	return ret;
}

static void __exit s3c_keypad_exit(void)
{
	platform_driver_unregister(&s3c_keypad_driver);
}

module_init(s3c_keypad_init);
module_exit(s3c_keypad_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");
