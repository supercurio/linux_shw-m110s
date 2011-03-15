/*
 * Driver for S5KA3DFX (UXGA camera) from Samsung Electronics
 * 
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEBUG	1	//for output dump log message

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/s5ka3dfx_platform.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

//vga tuning
#include "s5ka3dfx.h"
#if defined CONFIG_M115S		// Latona
#include "s5ka3dfx_sehf_register.h"
#else							// Galaxy-S
#include "s5ka3dfx_lsi_register.h"	
#endif

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <mach/max8998_function.h>

//#define VGA_CAM_DEBUG 
//#define VGA_VT_CAPTURE

#ifdef VGA_CAM_DEBUG
#define dev_dbg	dev_err
#endif

#define S5KA3DFX_DRIVER_NAME	"S5KA3DFX"

/* Default resolution & pixelformat. plz ref s5ka3dfx_platform.h */
#define DEFAULT_RES			WVGA				/* Index of resoultion */
#define DEFAUT_FPS_INDEX	S5KA3DFX_15FPS
#define DEFAULT_FMT			V4L2_PIX_FMT_UYVY	/* YUV422 */

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10 
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1280 (H) x 1024 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 * Max. pixel clock frequency : 48MHz(upto)
 * Internal PLL (6MHz to 27MHz input frequency)
 */

static int s5ka3dfx_init(struct v4l2_subdev *sd, u32 val);		//for fixing build error	//s1_camera [ Defense process by ESD input ]

/* Camera functional setting values configured by user concept */
struct s5ka3dfx_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct s5ka3dfx_state {
	struct s5ka3dfx_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct s5ka3dfx_userset userset;
	int framesize_index;
	int freq;	/* MCLK in KHz */
	int is_mipi;
	int isize;
	int ver;
	int fps;
	enum v4l2_vt_mode vt_mode; /*For VT camera*/	//HD VT : requested by SKT
	int check_dataline;
	int check_previewdata;
};

enum {
	S5KA3DFX_PREVIEW_QVGA,	//S1-KOR [HD VT : requested by SKT]
	S5KA3DFX_PREVIEW_VGA,
} S5KA3DFX_FRAME_SIZE;

struct s5ka3dfx_enum_framesize {
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

struct s5ka3dfx_enum_framesize s5ka3dfx_framesize_list[] = {
	{S5KA3DFX_PREVIEW_QVGA, 320, 240},	//S1-KOR [HD VT : requested by SKT]
	{S5KA3DFX_PREVIEW_VGA, 640, 480}
};

static inline struct s5ka3dfx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5ka3dfx_state, sd);
}

//s1_camera [ Defense process by ESD input ] _[
static int s5ka3dfx_power_on(void)
{
	int err;

	printk(KERN_DEBUG "s5ka3dfx_power_on in s5ka3dfx driver \n");

	/* CAM_VGA_nSTBY - GPJ1(2)  */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPJ1");

	if (err) {
		printk(KERN_ERR "failed to request GPJ1(2) for camera control\n");

		return err;
	}

	/* CAM_VGA_nRST - GPB(6) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB6");

	if (err) {
		printk(KERN_ERR "failed to request GPB6 for camera control\n");

		return err;
	}

	/* CAM_IO_EN - GPB(7) */
	err = gpio_request(GPIO_CAM_IO_EN, "GPB7");

	if(err) {
		printk(KERN_ERR "failed to request GPB7 for camera control\n");

		return err;
	}

	/* set pullup/down disable for output gpio ports of camera */
	if (s3c_gpio_setpull(GPIO_CAM_VGA_nSTBY, S3C_GPIO_PULL_NONE)) {
		printk(KERN_ERR "failed to set pull GPJ1(2) for camera control\n");
	}
	
	if (s3c_gpio_setpull(GPIO_CAM_VGA_nRST, S3C_GPIO_PULL_NONE)) {
		printk(KERN_ERR "failed to set pull GPB6 for camera control\n");
	}

	if(s3c_gpio_setpull(GPIO_CAM_IO_EN, S3C_GPIO_PULL_NONE)) {
		printk(KERN_ERR "failed to request GPB7 for camera control\n");
	}

	// Turn CAM_ISP_SYS_2.8V on
	gpio_direction_output(GPIO_CAM_IO_EN, 0);
	gpio_set_value(GPIO_CAM_IO_EN, 1);

	mdelay(1);

	// Turn CAM_SENSOR_A2.8V on
	Set_MAX8998_PM_OUTPUT_Voltage(LDO13, VCC_2p800);
	Set_MAX8998_PM_REG(ELDO13, 1);

	mdelay(1);

	// Turn CAM_ISP_HOST_2.8V on
	Set_MAX8998_PM_OUTPUT_Voltage(LDO15, VCC_2p800);
	Set_MAX8998_PM_REG(ELDO15, 1);

	mdelay(1);

	// Turn CAM_ISP_RAM_1.8V on
	Set_MAX8998_PM_OUTPUT_Voltage(LDO14, VCC_1p800);
	Set_MAX8998_PM_REG(ELDO14, 1);

	mdelay(1);
	
	gpio_free(GPIO_CAM_IO_EN);	
	mdelay(1);

	// CAM_VGA_nSTBY  HIGH		
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);

	mdelay(1);

	// Mclk enable
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S5PV210_GPE1_3_CAM_A_CLKOUT);

	mdelay(1);

	// CAM_VGA_nRST  HIGH		
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);		

	mdelay(4);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}


static int s5ka3dfx_power_off(void)
{
	int err;

	printk(KERN_DEBUG "s5ka3dfx_power_off in s5ka3dfx driver \n");

	/* CAM_VGA_nSTBY - GPJ1(2)  */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPJ1");

	if (err) {
		printk(KERN_ERR "failed to request GPJ1(2) for camera control\n");

		return err;
	}

	/* CAM_VGA_nRST - GPB(6) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB6");

	if (err) {
		printk(KERN_ERR "failed to request GPB6 for camera control\n");

		return err;
	}


	// CAM_VGA_nRST  LOW		
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);

	mdelay(1);

	// Mclk disable
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);

	mdelay(1);

	// CAM_VGA_nSTBY  LOW		
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);

	mdelay(1);

	/* CAM_IO_EN - GPB(7) */
	err = gpio_request(GPIO_CAM_IO_EN, "GPB7");

	if(err) {
		printk(KERN_ERR "failed to request GPB7 for camera control\n");

		return err;
	}

	// Turn CAM_ISP_HOST_2.8V off
	Set_MAX8998_PM_REG(ELDO15, 0);

	mdelay(1);

	// Turn CAM_SENSOR_A2.8V off
	Set_MAX8998_PM_REG(ELDO13, 0);

	// Turn CAM_ISP_RAM_1.8V off
	Set_MAX8998_PM_REG(ELDO14, 0);

	// Turn CAM_ISP_SYS_2.8V off
	gpio_direction_output(GPIO_CAM_IO_EN, 1);
	gpio_set_value(GPIO_CAM_IO_EN, 0);
	
	gpio_free(GPIO_CAM_IO_EN);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}


static int s5ka3dfx_power_en(int onoff)
{
	if(onoff){
		s5ka3dfx_power_on();
	} else {
		s5ka3dfx_power_off();
	}

	return 0;
}
static int s5ka3dfx_reset(struct v4l2_subdev *sd)
{
	s5ka3dfx_power_en(0);
	mdelay(5);
	s5ka3dfx_power_en(1);
	mdelay(5);
	s5ka3dfx_init(sd, 0);
	return 0;
}
// _]

/*
 * S5KA3DFX register structure : 2bytes address, 2bytes value
 * retry on write failure up-to 5 times
 */
static inline int s5ka3dfx_write(struct v4l2_subdev *sd, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[1];
	unsigned char reg[2];
	int err = 0;
	int retry = 0;


	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = reg;

	reg[0] = addr & 0xff;
	reg[1] = val & 0xff;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return err;	/* Returns here on success */

	/* abnormal case: retry 5 times */
	if (retry < 5) {
		dev_err(&client->dev, "%s: address: 0x%02x%02x, " \
			"value: 0x%02x%02x\n", __func__, \
			reg[0], reg[1], reg[2], reg[3]);
		retry++;
		goto again;
	}

	return err;
}

static int s5ka3dfx_i2c_write(struct v4l2_subdev *sd, unsigned char *i2c_data, unsigned char length)
{
	int ret = -1;
	int retry_count = 2;		// change retry count 1->2
	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[length], i;
	struct i2c_msg msg = {client->addr, 0, length, buf};

	for (i = 0; i < length; i++) {
		buf[i] = i2c_data[i];
	}
	
#ifdef VGA_CAM_DEBUG
	printk("W: ");
	for(i = 0; i < length; i++){
		printk("0x%02x ", buf[i]);
	}
	printk("\n");
#endif

	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		dev_err(&client->dev, "%s: I2C write retry \n", __func__);		// change retry count 1->2
		msleep(10);
	}

	if(ret < 0)
	{
		dev_err(&client->dev, "%s: I2C write FAIL \n", __func__);		// change retry count 1->2
	}

	return (ret == 1) ? 0 : -EIO;
}

static int s5ka3dfx_write_regs(struct v4l2_subdev *sd, const u32 regs[], int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_reg reg_set[size];
	int i=0;
	int err=0;

	v4l_info(client, "%s: i2c write multi register set\n", __func__);

	for (i = 0; i < size; i++) {
		reg_set[i].addr = (unsigned char)((regs[i] & 0x0000ff00)>> 8);
		reg_set[i].val = (unsigned char)(regs[i] & 0x000000ff);
	}

	for (i = 0; i < size; i++) {
		err = s5ka3dfx_i2c_write(sd, (char *)&reg_set[i], sizeof(reg_set[i]));
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n", \
			__func__);

			break;
		}
	}

	if(err < 0)
		return -EIO;	/* FIXME */

	return 0;	/* FIXME */
}

//s1_camera [for VGA capture] _[
#if 0
static int s5ka3dfx_i2c_read(struct v4l2_subdev *sd, unsigned char w_data[],
				unsigned char w_len, unsigned char r_data[], unsigned char r_len)
#else
static int s5ka3dfx_i2c_read(struct v4l2_subdev *sd, unsigned char *w_data,
				unsigned char w_len, unsigned char *r_data, unsigned char r_len)
#endif
{
	int ret = -1;
	int retry_count = 2;

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[w_len], i;
	struct i2c_msg msg = {client->addr, 0, w_len, buf};

	v4l_info(client, "%s() \n", __func__);

	for (i = 0; i < w_len; i++) {
		buf[i] = w_data[i];
	}

#ifdef VGA_CAM_DEBUG
		printk("R: ");
		for(i = 0; i < w_len; i++){
			printk("0x%02x ", buf[i]);
		}
		printk("\n");
#endif

	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		dev_err(&client->dev, "%s: I2C write retry \n", __func__);		// change retry count 1->2
		msleep(10);
	}

	if(ret < 0)
	{
		dev_err(&client->dev, "%s: I2C write FAIL \n", __func__);		// change retry count 1->2
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = r_len;
	msg.buf = r_data;

	retry_count = 2;
	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		dev_err(&client->dev, "%s: I2C read retry \n", __func__);		// change retry count 1->2
		msleep(10);
	}

	if(ret < 0)
	{
		dev_err(&client->dev, "%s: I2C read FAIL \n", __func__);		// change retry count 1->2
	}

#ifdef VGA_CAM_DEBUG
	printk("R: read_data =");
	for(i = 0; i < r_len; i++){
		printk("0x%02x ", r_data[i]);
	}
	printk("\n");
#endif

	return (ret == 1) ? 0 : -EIO;
}
// _]

#if (IS_USE_REGISTER_CONFIGURE_FILE_LSI)		//vga tuning

int parsing_section;

static u32 s5ka3dfx_util_hex_val(char hex)
{
	if ( hex >= 'a' && hex <= 'f' )
	{
		return (hex-'a'+10);
	}
	else if ( hex >= 'A' && hex <= 'F' )
	{
		return (hex - 'A' + 10 );
	}
	else if ( hex >= '0' && hex <= '9' )
	{
		return (hex - '0');
	}
	else
	{
		return 0;
	}
}

static u32 s5ka3dfx_util_gets(char* buffer, char* line, int is_start)
{
	int          i;
	char*        _r_n_ptr;
	static char* buffer_ptr;

	memset(line, 0, 1024);

	if ( is_start )
		buffer_ptr = buffer;

	_r_n_ptr = strstr(buffer_ptr, "\r\n");

	//\n  
	if ( _r_n_ptr )
	{
		for ( i = 0 ; ; i++ )
		{
			if ( buffer_ptr+i == _r_n_ptr )
			{
				buffer_ptr = _r_n_ptr+1;
				break;
			}
			line[i] = buffer_ptr[i];
		}
		line[i] = '\0';

		return 1;
	}
//\n  
	else
	{
		if ( strlen(buffer_ptr) > 0 )
		{
			strcpy(line, buffer_ptr);
			return 0;
		}
		else
		{
			return 0;
		}
	}
}

static u32 s5ka3dfx_util_atoi(char* str)
{
	unsigned int i,j=0;
	unsigned int val_len;
	unsigned int ret_val=0;

	if (str == NULL)
		return 0;

	//decimal
	if(strlen(str) <= 4 || (strstr(str, "0x")==NULL && strstr(str, "0X")==NULL ))
	{
		for( ; ; str++ ) {
			switch( *str ) {
				case '0'...'9':
					ret_val= 10 * ret_val + ( *str - '0' ) ;
					break ;
					
				default:
					break ;
		}
	}

	return ret_val;
	}

	//hex ex:0xa0c
	val_len = strlen(str);

	for (i = val_len-1 ; i >= 2 ; i--)
	{
		ret_val = ret_val + (s5ka3dfx_util_hex_val(str[i])<<(j*4));
		j++;
	}

	return ret_val;
}

static int s5ka3dfx_util_trim(char* buff)
{
	int         left_index;
	int         right_index;
	int         buff_len;
	int         i;

	buff_len	= strlen(buff);
	left_index	= -1;
	right_index = -1;

	if ( buff_len == 0 )
	{
		return 0;
	}

	/* left index(  white space  ) */
	for ( i = 0 ; i < buff_len ; i++ )
	{
		if ( buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
		{
			left_index = i;
			break;
		}
	}

	/* right index(  white space  ) */
	for ( i = buff_len-1 ; i >= 0 ; i-- )
	{
		if ( buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
		{
			right_index = i;
			buff[i+1] = '\0';
			break;
		}
	}

	if ( left_index == -1 && right_index == -1 )
	{
		strcpy(buff, "");
	}
	else if ( left_index <= right_index )
	{
		strcpy(buff, buff+left_index);
	}
	else
	{
		return -EINVAL;
	}

	return 0;
}


static u32 s5ka3dfx_insert_register_table(char* line)
{
	int   i;
	char  reg_val_str[7];
	int   reg_val_str_idx=0;

	unsigned int  reg_val;

	s5ka3dfx_util_trim(line);

	if ( strlen(line) == 0 || (line[0] == '/' && line[1] == '/' ) || (line[0] == '/' && line[1] == '*' ) || line[0] == '{' || line[0] == '}' )
	{
		return 0;
	}

	for (i = 0 ; ; i++)
	{
		if ( line[i] == ' ' || line[i] == '\t' || line[i] == '/' || line[i] == '\0')
			continue;

		if ( line[i] == ',' )
			break;

		reg_val_str[reg_val_str_idx++] = line[i];
	}

	reg_val_str[reg_val_str_idx] = '\0';

	reg_val = s5ka3dfx_util_atoi(reg_val_str);

#if 1	
	if ( parsing_section == REG_INIT_VGA_SECTION)          reg_init_vga_table[reg_init_vga_index++] = reg_val;
	else if ( parsing_section == REG_INIT_QVGA_SECTION)         reg_init_qvga_table[reg_init_qvga_index++] = reg_val;	//S1-KOR [HD VT : requested by SKT]
	else if ( parsing_section == REG_INIT_VGA_VT_SECTION)       reg_init_vga_vt_table[reg_init_vga_vt_index++] = reg_val;
	else if ( parsing_section == REG_INIT_QVGA_VT_SECTION)      reg_init_qvga_vt_table[reg_init_qvga_vt_index++] = reg_val;	//S1-KOR [HD VT : requested by SKT]
	else if ( parsing_section == REG_WB_AUTO_SECTION)           reg_wb_auto_table[reg_wb_auto_index++] = reg_val;
	else if ( parsing_section == REG_WB_DAYLIGHT_SECTION)       reg_wb_daylight_table[reg_wb_daylight_index++] = reg_val;
	else if ( parsing_section == REG_WB_CLOUDY_SECTION)         reg_wb_cloudy_table[reg_wb_cloudy_index++] = reg_val;
	else if ( parsing_section == REG_WB_INCANDESCENT_SECTION)   reg_wb_incandescent_table[reg_wb_incandescent_index++] = reg_val;
	else if ( parsing_section == REG_WB_FLUORESCENT_SECTION)    reg_wb_fluorescent_table[reg_wb_fluorescent_index++] = reg_val;
	else if ( parsing_section == REG_EV_M4_SECTION)                reg_ev_m4_table[reg_ev_m4_index++] = reg_val;
	else if ( parsing_section == REG_EV_M3_SECTION)                reg_ev_m3_table[reg_ev_m3_index++] = reg_val;
	else if ( parsing_section == REG_EV_M2_SECTION)                reg_ev_m2_table[reg_ev_m2_index++] = reg_val;
	else if ( parsing_section == REG_EV_M1_SECTION)                reg_ev_m1_table[reg_ev_m1_index++] = reg_val;
	else if ( parsing_section == REG_EV_DEFAULT_SECTION)       reg_ev_default_table[reg_ev_default_index++] = reg_val;
	else if ( parsing_section == REG_EV_P1_SECTION)                reg_ev_p1_table[reg_ev_p1_index++] = reg_val;
	else if ( parsing_section == REG_EV_P2_SECTION)                reg_ev_p2_table[reg_ev_p2_index++] = reg_val;
	else if ( parsing_section == REG_EV_P3_SECTION)                reg_ev_p3_table[reg_ev_p3_index++] = reg_val;
	else if ( parsing_section == REG_EV_P4_SECTION)                reg_ev_p4_table[reg_ev_p4_index++] = reg_val;
	
	else if ( parsing_section == REG_EV_VT_M4_SECTION)             reg_ev_vt_m4_table[reg_ev_vt_m4_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M3_SECTION)             reg_ev_vt_m3_table[reg_ev_vt_m3_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M2_SECTION)             reg_ev_vt_m2_table[reg_ev_vt_m2_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M1_SECTION)             reg_ev_vt_m1_table[reg_ev_vt_m1_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_DEFAULT_SECTION)    reg_ev_vt_default_table[reg_ev_vt_default_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P1_SECTION)             reg_ev_vt_p1_table[reg_ev_vt_p1_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P2_SECTION)             reg_ev_vt_p2_table[reg_ev_vt_p2_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P3_SECTION)             reg_ev_vt_p3_table[reg_ev_vt_p3_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P4_SECTION)             reg_ev_vt_p4_table[reg_ev_vt_p4_index++]	= reg_val;    
	else if ( parsing_section == REG_EFFECT_NONE_SECTION)       reg_effect_none_table[reg_effect_none_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_GRAY_SECTION)       reg_effect_gray_table[reg_effect_gray_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_RED_SECTION)        reg_effect_red_table[reg_effect_red_index++] = reg_val;    
	else if ( parsing_section == REG_EFFECT_SEPIA_SECTION)      reg_effect_sepia_table[reg_effect_sepia_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_GREEN_SECTION)      reg_effect_green_table[reg_effect_green_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_AQUA_SECTION)       reg_effect_aqua_table[reg_effect_aqua_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_NEGATIVE_SECTION)   reg_effect_negative_table[reg_effect_negative_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_NONE_SECTION)         reg_flip_none_table[reg_flip_none_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_WATER_SECTION)        reg_flip_water_table[reg_flip_water_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_MIRROR_SECTION)       reg_flip_mirror_table[reg_flip_mirror_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_WATER_MIRROR_SECTION) reg_flip_water_mirror_table[reg_flip_water_mirror_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_NONE_SECTION)       reg_pretty_none_table[reg_pretty_none_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL1_SECTION)     reg_pretty_level1_table[reg_pretty_level1_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL2_SECTION)     reg_pretty_level2_table[reg_pretty_level2_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL3_SECTION)     reg_pretty_level3_table[reg_pretty_level3_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_NONE_SECTION)    reg_pretty_vt_none_table[reg_pretty_vt_none_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL1_SECTION)  reg_pretty_vt_level1_table[reg_pretty_vt_level1_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL2_SECTION)  reg_pretty_vt_level2_table[reg_pretty_vt_level2_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL3_SECTION)  reg_pretty_vt_level3_table[reg_pretty_vt_level3_index++] = reg_val;
	else if ( parsing_section == REG_7FPS_VGA_SECTION)          reg_vga_7fps_table[reg_vga_7fps_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_VGA_SECTION)         reg_vga_10fps_table[reg_vga_10fps_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_VGA_SECTION)         reg_vga_15fps_table[reg_vga_15fps_index++] = reg_val;  

	//S1-KOR [HD VT : requested by SKT]
	else if ( parsing_section == REG_7FPS_QVGA_SECTION)          reg_qvga_7fps_table[reg_qvga_7fps_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_QVGA_SECTION)         reg_qvga_10fps_table[reg_qvga_10fps_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_QVGA_SECTION)         reg_qvga_15fps_table[reg_qvga_15fps_index++] = reg_val;  

	//add
	else if ( parsing_section == REG_7FPS_VGA_VT_SECTION)          reg_vga_7fps_vt_table[reg_vga_7fps_vt_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_VGA_VT_SECTION)         reg_vga_10fps_vt_table[reg_vga_10fps_vt_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_VGA_VT_SECTION)         reg_vga_15fps_vt_table[reg_vga_15fps_vt_index++] = reg_val;  
#else
	if      ( parsing_section == REG_INIT_QCIF_SECTION)         reg_init_qcif_table[reg_init_qcif_index++] = reg_val;
	else if ( parsing_section == REG_INIT_CIF_SECTION)          reg_init_cif_table[reg_init_cif_index++] = reg_val;
	else if ( parsing_section == REG_INIT_QVGA_SECTION)         reg_init_qvga_table[reg_init_qvga_index++] = reg_val;
	else if ( parsing_section == REG_INIT_VGA_SECTION)          reg_init_vga_table[reg_init_vga_index++] = reg_val;
	else if ( parsing_section == REG_INIT_QCIF_VT_SECTION)      reg_init_qcif_vt_table[reg_init_qcif_vt_index++] = reg_val;
	else if ( parsing_section == REG_INIT_CIF_VT_SECTION)       reg_init_cif_vt_table[reg_init_cif_vt_index++] = reg_val;
	else if ( parsing_section == REG_INIT_QVGA_VT_SECTION)      reg_init_qvga_vt_table[reg_init_qvga_vt_index++] = reg_val;
	else if ( parsing_section == REG_INIT_VGA_VT_SECTION)       reg_init_vga_vt_table[reg_init_vga_vt_index++] = reg_val;
	else if ( parsing_section == REG_WB_AUTO_SECTION)           reg_wb_auto_table[reg_wb_auto_index++] = reg_val;
	else if ( parsing_section == REG_WB_DAYLIGHT_SECTION)       reg_wb_daylight_table[reg_wb_daylight_index++] = reg_val;
	else if ( parsing_section == REG_WB_CLOUDY_SECTION)         reg_wb_cloudy_table[reg_wb_cloudy_index++] = reg_val;
	else if ( parsing_section == REG_WB_INCANDESCENT_SECTION)   reg_wb_incandescent_table[reg_wb_incandescent_index++] = reg_val;
	else if ( parsing_section == REG_WB_FLUORESCENT_SECTION)    reg_wb_fluorescent_table[reg_wb_fluorescent_index++] = reg_val;
	else if ( parsing_section == REG_EV_M4_SECTION)                reg_ev_m4_table[reg_ev_m4_index++] = reg_val;
	else if ( parsing_section == REG_EV_M3_SECTION)                reg_ev_m3_table[reg_ev_m3_index++] = reg_val;
	else if ( parsing_section == REG_EV_M2_SECTION)                reg_ev_m2_table[reg_ev_m2_index++] = reg_val;
	else if ( parsing_section == REG_EV_M1_SECTION)                reg_ev_m1_table[reg_ev_m1_index++] = reg_val;
	else if ( parsing_section == REG_EV_DEFAULT_SECTION)       reg_ev_default_table[reg_ev_default_index++] = reg_val;
	else if ( parsing_section == REG_EV_P1_SECTION)                reg_ev_p1_table[reg_ev_p1_index++] = reg_val;
	else if ( parsing_section == REG_EV_P2_SECTION)                reg_ev_p2_table[reg_ev_p2_index++] = reg_val;
	else if ( parsing_section == REG_EV_P3_SECTION)                reg_ev_p3_table[reg_ev_p3_index++] = reg_val;
	else if ( parsing_section == REG_EV_P4_SECTION)                reg_ev_p4_table[reg_ev_p4_index++] = reg_val;
	
	else if ( parsing_section == REG_EV_VT_M4_SECTION)             reg_ev_vt_m4_table[reg_ev_vt_m4_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M3_SECTION)             reg_ev_vt_m3_table[reg_ev_vt_m3_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M2_SECTION)             reg_ev_vt_m2_table[reg_ev_vt_m2_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_M1_SECTION)             reg_ev_vt_m1_table[reg_ev_vt_m1_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_DEFAULT_SECTION)    reg_ev_vt_default_table[reg_ev_vt_default_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P1_SECTION)             reg_ev_vt_p1_table[reg_ev_vt_p1_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P2_SECTION)             reg_ev_vt_p2_table[reg_ev_vt_p2_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P3_SECTION)             reg_ev_vt_p3_table[reg_ev_vt_p3_index++]	= reg_val;    
	else if ( parsing_section == REG_EV_VT_P4_SECTION)             reg_ev_vt_p4_table[reg_ev_vt_p4_index++]	= reg_val;    
	else if ( parsing_section == REG_EFFECT_NONE_SECTION)       reg_effect_none_table[reg_effect_none_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_GRAY_SECTION)       reg_effect_gray_table[reg_effect_gray_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_RED_SECTION)        reg_effect_red_table[reg_effect_red_index++] = reg_val;    
	else if ( parsing_section == REG_EFFECT_SEPIA_SECTION)      reg_effect_sepia_table[reg_effect_sepia_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_GREEN_SECTION)      reg_effect_green_table[reg_effect_green_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_AQUA_SECTION)       reg_effect_aqua_table[reg_effect_aqua_index++] = reg_val;
	else if ( parsing_section == REG_EFFECT_NEGATIVE_SECTION)   reg_effect_negative_table[reg_effect_negative_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_NONE_SECTION)         reg_flip_none_table[reg_flip_none_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_WATER_SECTION)        reg_flip_water_table[reg_flip_water_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_MIRROR_SECTION)       reg_flip_mirror_table[reg_flip_mirror_index++] = reg_val;
	else if ( parsing_section == REG_FLIP_WATER_MIRROR_SECTION) reg_flip_water_mirror_table[reg_flip_water_mirror_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_NONE_SECTION)       reg_pretty_none_table[reg_pretty_none_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL1_SECTION)     reg_pretty_level1_table[reg_pretty_level1_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL2_SECTION)     reg_pretty_level2_table[reg_pretty_level2_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_LEVEL3_SECTION)     reg_pretty_level3_table[reg_pretty_level3_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_NONE_SECTION)    reg_pretty_vt_none_table[reg_pretty_vt_none_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL1_SECTION)  reg_pretty_vt_level1_table[reg_pretty_vt_level1_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL2_SECTION)  reg_pretty_vt_level2_table[reg_pretty_vt_level2_index++] = reg_val;
	else if ( parsing_section == REG_PRETTY_VT_LEVEL3_SECTION)  reg_pretty_vt_level3_table[reg_pretty_vt_level3_index++] = reg_val;
	else if ( parsing_section == REG_7FPS_QCIF_SECTION)         reg_qcif_7fps_table[reg_qcif_7fps_index++] = reg_val;  
	else if ( parsing_section == REG_10FPS_QCIF_SECTION)        reg_qcif_10fps_table[reg_qcif_10fps_index++] = reg_val;  
	else if ( parsing_section == REG_15FPS_QCIF_SECTION)        reg_qcif_15fps_table[reg_qcif_15fps_index++] = reg_val;    
	else if ( parsing_section == REG_7FPS_CIF_SECTION)          reg_cif_7fps_table[reg_cif_7fps_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_CIF_SECTION)         reg_cif_10fps_table[reg_cif_10fps_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_CIF_SECTION)         reg_cif_15fps_table[reg_cif_15fps_index++] = reg_val;  
	else if ( parsing_section == REG_7FPS_QVGA_SECTION)          reg_qvga_7fps_table[reg_qvga_7fps_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_QVGA_SECTION)         reg_qvga_10fps_table[reg_qvga_10fps_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_QVGA_SECTION)         reg_qvga_15fps_table[reg_qvga_15fps_index++] = reg_val;  
	else if ( parsing_section == REG_7FPS_VGA_SECTION)          reg_vga_7fps_table[reg_vga_7fps_index++] = reg_val;
	else if ( parsing_section == REG_10FPS_VGA_SECTION)         reg_vga_10fps_table[reg_vga_10fps_index++] = reg_val;
	else if ( parsing_section == REG_15FPS_VGA_SECTION)         reg_vga_15fps_table[reg_vga_15fps_index++] = reg_val;  
#endif
	return 0;
}

static u32 s5ka3dfx_parsing_section(char* line)
{
#if 1
	if ( strstr(line, REG_VGA_INIT) != NULL )
		parsing_section = REG_INIT_VGA_SECTION;    
	else if ( strstr(line, REG_QVGA_INIT) != NULL )	//S1-KOR [HD VT : requested by SKT]
		parsing_section = REG_INIT_QVGA_SECTION;  
	else if ( strstr(line, REG_VGA_VT_INIT) != NULL )
		parsing_section = REG_INIT_VGA_VT_SECTION;    
	else if ( strstr(line, REG_QVGA_VT_INIT) != NULL )	//S1-KOR [HD VT : requested by SKT]
		parsing_section = REG_INIT_QVGA_VT_SECTION;  
	else if ( strstr(line, REG_WB_AUTO) != NULL )
		parsing_section = REG_WB_AUTO_SECTION;
	else if ( strstr(line, REG_WB_DAYLIGHT ) != NULL )
		parsing_section = REG_WB_DAYLIGHT_SECTION;
	else if ( strstr(line, REG_WB_CLOUDY ) != NULL )
		parsing_section = REG_WB_CLOUDY_SECTION;
	else if( strstr(line, REG_WB_INCANDESCENT) != NULL)
		parsing_section = REG_WB_INCANDESCENT_SECTION;
	else if( strstr(line, REG_WB_FLUORESCENT) != NULL)
		parsing_section = REG_WB_FLUORESCENT_SECTION;
	else if ( strstr(line, REG_EV_M4) != NULL )
		parsing_section = REG_EV_M4_SECTION;
	else if ( strstr(line, REG_EV_M3) != NULL )
		parsing_section = REG_EV_M3_SECTION;
	else if ( strstr(line, REG_EV_M2) != NULL )
		parsing_section = REG_EV_M2_SECTION;
	else if ( strstr(line, REG_EV_M1) != NULL )
		parsing_section = REG_EV_M1_SECTION;
	else if ( strstr(line, REG_EV_DEFAULT) != NULL )
		parsing_section = REG_EV_DEFAULT_SECTION;
	else if ( strstr(line, REG_EV_P1) != NULL )
		parsing_section = REG_EV_P1_SECTION;
	else if ( strstr(line, REG_EV_P2) != NULL )
		parsing_section = REG_EV_P2_SECTION;
	else if ( strstr(line, REG_EV_P3) != NULL )
		parsing_section = REG_EV_P3_SECTION;
	else if ( strstr(line, REG_EV_P4) != NULL )
		parsing_section = REG_EV_P4_SECTION;
	else if ( strstr(line, REG_EV_VT_M4) != NULL )
		parsing_section = REG_EV_VT_M4_SECTION;    
	else if ( strstr(line, REG_EV_VT_M3) != NULL )
		parsing_section = REG_EV_VT_M3_SECTION;    
	else if ( strstr(line, REG_EV_VT_M2) != NULL )
		parsing_section = REG_EV_VT_M2_SECTION;    
	else if ( strstr(line, REG_EV_VT_M1) != NULL )
		parsing_section = REG_EV_VT_M1_SECTION;    
	else if ( strstr(line, REG_EV_VT_DEFAULT) != NULL )
		parsing_section = REG_EV_VT_DEFAULT_SECTION;    
	else if ( strstr(line, REG_EV_VT_P1) != NULL )
		parsing_section = REG_EV_VT_P1_SECTION;    
	else if ( strstr(line, REG_EV_VT_P2) != NULL )
		parsing_section = REG_EV_VT_P2_SECTION;    
	else if ( strstr(line, REG_EV_VT_P3) != NULL )
		parsing_section = REG_EV_VT_P3_SECTION;    
	else if ( strstr(line, REG_EV_VT_P4) != NULL )
		parsing_section = REG_EV_VT_P4_SECTION;    
	else if ( strstr(line, REG_EFFECT_NONE) != NULL )
		parsing_section = REG_EFFECT_NONE_SECTION;
	else if ( strstr(line, REG_EFFECT_GRAY) != NULL )
		parsing_section = REG_EFFECT_GRAY_SECTION;
	else if ( strstr(line, REG_EFFECT_RED) != NULL )
		parsing_section = REG_EFFECT_RED_SECTION;    
	else if ( strstr(line, REG_EFFECT_SEPIA) != NULL )
		parsing_section = REG_EFFECT_SEPIA_SECTION;
	else if ( strstr(line, REG_EFFECT_GREEN) != NULL )
		parsing_section = REG_EFFECT_GREEN_SECTION;
	else if ( strstr(line, REG_EFFECT_AQUA) != NULL )
		parsing_section = REG_EFFECT_AQUA_SECTION;
	else if ( strstr(line, REG_EFFECT_NEGATIVE) != NULL )
		parsing_section = REG_EFFECT_NEGATIVE_SECTION;
	else if ( strstr(line, REG_FLIP_NONE) != NULL )
		parsing_section = REG_FLIP_NONE_SECTION;    
	else if ( strstr(line, REG_FLIP_WATER) != NULL )
		parsing_section = REG_FLIP_WATER_SECTION;    
	else if ( strstr(line, REG_FLIP_MIRROR) != NULL )
		parsing_section = REG_FLIP_MIRROR_SECTION;    
	else if ( strstr(line, REG_FLIP_WATER_MIRROR) != NULL )
		parsing_section = REG_FLIP_WATER_MIRROR_SECTION;    
	else if ( strstr(line, REG_PRETTY_NONE) != NULL )
		parsing_section = REG_PRETTY_NONE_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL1) != NULL )
		parsing_section = REG_PRETTY_LEVEL1_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL2) != NULL )
		parsing_section = REG_PRETTY_LEVEL2_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL3) != NULL )
		parsing_section = REG_PRETTY_LEVEL3_SECTION;        
	else if ( strstr(line, REG_PRETTY_VT_NONE) != NULL )
		parsing_section = REG_PRETTY_VT_NONE_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL1) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL1_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL2) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL2_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL3) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL3_SECTION;     
	
	//S1-KOR [HD VT : requested by SKT]
	else if ( strstr(line, REG_QVGA_FPS7) != NULL )
		parsing_section = REG_7FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_QVGA_FPS10) != NULL )
		parsing_section = REG_10FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_QVGA_FPS15) != NULL )
		parsing_section = REG_15FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS7) != NULL )
		parsing_section = REG_7FPS_VGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS10) != NULL )
		parsing_section = REG_10FPS_VGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS15) != NULL )
		parsing_section = REG_15FPS_VGA_SECTION;  
	
	//add
	else if ( strstr(line, REG_VGA_VT_FPS7) != NULL )
		parsing_section = REG_7FPS_VGA_VT_SECTION;   
	else if ( strstr(line, REG_VGA_VT_FPS10) != NULL )
		parsing_section = REG_10FPS_VGA_VT_SECTION;   
	else if ( strstr(line, REG_VGA_VT_FPS15) != NULL )
		parsing_section = REG_15FPS_VGA_VT_SECTION;  
	else
		return -EINVAL;
#else
	if ( strstr(line, REG_QCIF_INIT) != NULL )
		parsing_section = REG_INIT_QCIF_SECTION;
	else if ( strstr(line, REG_CIF_INIT) != NULL )
		parsing_section = REG_INIT_CIF_SECTION;  
	else if ( strstr(line, REG_QVGA_INIT) != NULL )
		parsing_section = REG_INIT_QVGA_SECTION;  
	else if ( strstr(line, REG_VGA_INIT) != NULL )
		parsing_section = REG_INIT_VGA_SECTION;    
	else if ( strstr(line, REG_QCIF_VT_INIT) != NULL )
		parsing_section = REG_INIT_QCIF_VT_SECTION;
	else if ( strstr(line, REG_CIF_VT_INIT) != NULL )
		parsing_section = REG_INIT_CIF_VT_SECTION;  
	else if ( strstr(line, REG_QVGA_VT_INIT) != NULL )
		parsing_section = REG_INIT_QVGA_VT_SECTION;  
	else if ( strstr(line, REG_VGA_VT_INIT) != NULL )
		parsing_section = REG_INIT_VGA_VT_SECTION;     
	else if ( strstr(line, REG_WB_AUTO) != NULL )
		parsing_section = REG_WB_AUTO_SECTION;
	else if ( strstr(line, REG_WB_DAYLIGHT ) != NULL )
		parsing_section = REG_WB_DAYLIGHT_SECTION;
	else if ( strstr(line, REG_WB_CLOUDY ) != NULL )
		parsing_section = REG_WB_CLOUDY_SECTION;
	else if( strstr(line, REG_WB_INCANDESCENT) != NULL)
		parsing_section = REG_WB_INCANDESCENT_SECTION;
	else if( strstr(line, REG_WB_FLUORESCENT) != NULL)
		parsing_section = REG_WB_FLUORESCENT_SECTION;
	else if ( strstr(line, REG_EV_M4) != NULL )
		parsing_section = REG_EV_M4_SECTION;
	else if ( strstr(line, REG_EV_M3) != NULL )
		parsing_section = REG_EV_M3_SECTION;
	else if ( strstr(line, REG_EV_M2) != NULL )
		parsing_section = REG_EV_M2_SECTION;
	else if ( strstr(line, REG_EV_M1) != NULL )
		parsing_section = REG_EV_M1_SECTION;
	else if ( strstr(line, REG_EV_DEFAULT) != NULL )
		parsing_section = REG_EV_DEFAULT_SECTION;
	else if ( strstr(line, REG_EV_P1) != NULL )
		parsing_section = REG_EV_P1_SECTION;
	else if ( strstr(line, REG_EV_P2) != NULL )
		parsing_section = REG_EV_P2_SECTION;
	else if ( strstr(line, REG_EV_P3) != NULL )
		parsing_section = REG_EV_P3_SECTION;
	else if ( strstr(line, REG_EV_P4) != NULL )
		parsing_section = REG_EV_P4_SECTION;
	else if ( strstr(line, REG_EV_VT_M4) != NULL )
		parsing_section = REG_EV_VT_M4_SECTION;    
	else if ( strstr(line, REG_EV_VT_M3) != NULL )
		parsing_section = REG_EV_VT_M3_SECTION;    
	else if ( strstr(line, REG_EV_VT_M2) != NULL )
		parsing_section = REG_EV_VT_M2_SECTION;    
	else if ( strstr(line, REG_EV_VT_M1) != NULL )
		parsing_section = REG_EV_VT_M1_SECTION;    
	else if ( strstr(line, REG_EV_VT_DEFAULT) != NULL )
		parsing_section = REG_EV_VT_DEFAULT_SECTION;    
	else if ( strstr(line, REG_EV_VT_P1) != NULL )
		parsing_section = REG_EV_VT_P1_SECTION;    
	else if ( strstr(line, REG_EV_VT_P2) != NULL )
		parsing_section = REG_EV_VT_P2_SECTION;    
	else if ( strstr(line, REG_EV_VT_P3) != NULL )
		parsing_section = REG_EV_VT_P3_SECTION;    
	else if ( strstr(line, REG_EV_VT_P4) != NULL )
		parsing_section = REG_EV_VT_P4_SECTION;    
	else if ( strstr(line, REG_EFFECT_NONE) != NULL )
		parsing_section = REG_EFFECT_NONE_SECTION;
	else if ( strstr(line, REG_EFFECT_GRAY) != NULL )
		parsing_section = REG_EFFECT_GRAY_SECTION;
	else if ( strstr(line, REG_EFFECT_RED) != NULL )
		parsing_section = REG_EFFECT_RED_SECTION;    
	else if ( strstr(line, REG_EFFECT_SEPIA) != NULL )
		parsing_section = REG_EFFECT_SEPIA_SECTION;
	else if ( strstr(line, REG_EFFECT_GREEN) != NULL )
		parsing_section = REG_EFFECT_GREEN_SECTION;
	else if ( strstr(line, REG_EFFECT_AQUA) != NULL )
		parsing_section = REG_EFFECT_AQUA_SECTION;
	else if ( strstr(line, REG_EFFECT_NEGATIVE) != NULL )
		parsing_section = REG_EFFECT_NEGATIVE_SECTION;
	else if ( strstr(line, REG_FLIP_NONE) != NULL )
		parsing_section = REG_FLIP_NONE_SECTION;    
	else if ( strstr(line, REG_FLIP_WATER) != NULL )
		parsing_section = REG_FLIP_WATER_SECTION;    
	else if ( strstr(line, REG_FLIP_MIRROR) != NULL )
		parsing_section = REG_FLIP_MIRROR_SECTION;    
	else if ( strstr(line, REG_FLIP_WATER_MIRROR) != NULL )
		parsing_section = REG_FLIP_WATER_MIRROR_SECTION;    
	else if ( strstr(line, REG_PRETTY_NONE) != NULL )
		parsing_section = REG_PRETTY_NONE_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL1) != NULL )
		parsing_section = REG_PRETTY_LEVEL1_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL2) != NULL )
		parsing_section = REG_PRETTY_LEVEL2_SECTION;    
	else if ( strstr(line, REG_PRETTY_LEVEL3) != NULL )
		parsing_section = REG_PRETTY_LEVEL3_SECTION;        
	else if ( strstr(line, REG_PRETTY_VT_NONE) != NULL )
		parsing_section = REG_PRETTY_VT_NONE_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL1) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL1_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL2) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL2_SECTION;    
	else if ( strstr(line, REG_PRETTY_VT_LEVEL3) != NULL )
		parsing_section = REG_PRETTY_VT_LEVEL3_SECTION;     
	else if ( strstr(line, REG_QCIF_FPS7) != NULL )
		parsing_section = REG_7FPS_QCIF_SECTION;    
	else if ( strstr(line, REG_QCIF_FPS10) != NULL )
		parsing_section = REG_10FPS_QCIF_SECTION;   
	else if ( strstr(line, REG_QCIF_FPS15) != NULL )
		parsing_section = REG_15FPS_QCIF_SECTION;   
	else if ( strstr(line, REG_CIF_FPS7) != NULL )
		parsing_section = REG_7FPS_CIF_SECTION;   
	else if ( strstr(line, REG_CIF_FPS10) != NULL )
		parsing_section = REG_10FPS_CIF_SECTION;   
	else if ( strstr(line, REG_CIF_FPS15) != NULL )
		parsing_section = REG_15FPS_CIF_SECTION;   
	else if ( strstr(line, REG_QVGA_FPS7) != NULL )
		parsing_section = REG_7FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_QVGA_FPS10) != NULL )
		parsing_section = REG_10FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_QVGA_FPS15) != NULL )
		parsing_section = REG_15FPS_QVGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS7) != NULL )
		parsing_section = REG_7FPS_VGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS10) != NULL )
		parsing_section = REG_10FPS_VGA_SECTION;   
	else if ( strstr(line, REG_VGA_FPS15) != NULL )
		parsing_section = REG_15FPS_VGA_SECTION;  
	else
		return -EINVAL;
#endif
	return 0;
}

static int s5ka3dfx_make_table(void)
{
	char*       buffer = NULL;
	char         line[1024];
	unsigned int file_size = 0;

	struct file *filep = NULL;
	mm_segment_t old_fs;
	int ret = 0;  

	printk("s5ka3dfx_make_table is called...\n");

	filep = filp_open(CAMIF_CONFIGURE_FILE_LSI, O_RDONLY, 0) ;

	if (filep && (filep!= 0xfffffffe))
	{
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		
		file_size = filep->f_op->llseek(filep, 0, SEEK_END);
		filep->f_op->llseek(filep, 0, SEEK_SET);
		
		buffer = (char*)kmalloc(file_size+1, GFP_KERNEL);
		
		filep->f_op->read(filep, buffer, file_size, &filep->f_pos);
		buffer[file_size] = '\0';
		
		filp_close(filep, current->files);

		set_fs(old_fs);

		printk("File size : %d\n", file_size);
	}
	else
	{
		return -EINVAL;
	}

	// init table index
	parsing_section = 0;

	s5ka3dfx_util_gets(buffer, line, 1);
	if ( s5ka3dfx_parsing_section(line) )
	{
		s5ka3dfx_insert_register_table(line);
	}

	while(s5ka3dfx_util_gets(buffer, line, 0))
	{
		if ( s5ka3dfx_parsing_section(line) )
		{
			s5ka3dfx_insert_register_table(line);
		}
	}

	s5ka3dfx_insert_register_table(line);

	kfree(buffer);

	return 0;
}

#endif

static int s5ka3dfx_init_reg_table_index(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	v4l_info(client, "%s: i2c write multi register set\n", __func__);
#if 0
	reg_init_qcif_index = 0;
	reg_init_cif_index = 0;
#endif
	reg_init_qvga_index = 0;	//S1-KOR [HD VT : requested by SKT]
	reg_init_vga_index = 0;
#if 0
	reg_init_qcif_vt_index = 0;
	reg_init_cif_vt_index = 0;
#endif
	reg_init_qvga_vt_index = 0;	//S1-KOR [HD VT : requested by SKT]
	reg_init_vga_vt_index = 0;
	reg_wb_auto_index = 0;
	reg_wb_daylight_index = 0;
	reg_wb_cloudy_index = 0;
	reg_wb_incandescent_index = 0;
	reg_wb_fluorescent_index = 0;
	reg_ev_m4_index = 0;
	reg_ev_m3_index = 0;
	reg_ev_m2_index = 0;
	reg_ev_m1_index = 0;
	reg_ev_default_index = 0;
	reg_ev_p1_index = 0;
	reg_ev_p2_index = 0;
	reg_ev_p3_index = 0;
	reg_ev_p4_index = 0;
	
	reg_ev_vt_m4_index = 0;
	reg_ev_vt_m3_index = 0;
	reg_ev_vt_m2_index = 0;
	reg_ev_vt_m1_index = 0;
	reg_ev_vt_default_index = 0;
	reg_ev_vt_p1_index = 0;
	reg_ev_vt_p2_index = 0;
	reg_ev_vt_p3_index = 0;
	reg_ev_vt_p4_index = 0;
	reg_effect_none_index = 0;
	reg_effect_red_index = 0;
	reg_effect_gray_index = 0;
	reg_effect_sepia_index = 0;
	reg_effect_green_index = 0;
	reg_effect_aqua_index = 0;
	reg_effect_negative_index = 0;
	reg_flip_none_index = 0;
	reg_flip_water_index = 0;
	reg_flip_mirror_index = 0;
	reg_flip_water_mirror_index = 0;
	reg_pretty_none_index = 0;
	reg_pretty_level1_index = 0;
	reg_pretty_level2_index = 0;
	reg_pretty_level3_index = 0;
	reg_pretty_vt_none_index = 0;
	reg_pretty_vt_level1_index = 0;
	reg_pretty_vt_level2_index = 0;
	reg_pretty_vt_level3_index = 0;
#if 0
	reg_qcif_7fps_index = 0;
	reg_qcif_10fps_index = 0;
	reg_qcif_15fps_index = 0;    
	reg_cif_7fps_index = 0;
	reg_cif_10fps_index = 0;
	reg_cif_15fps_index = 0;    
#endif
	//S1-KOR [HD VT : requested by SKT]
	reg_qvga_7fps_index = 0;
	reg_qvga_10fps_index = 0;
	reg_qvga_15fps_index = 0; 

	reg_vga_7fps_index = 0;    
	reg_vga_10fps_index = 0;  
	reg_vga_15fps_index = 0;      
	//add
	reg_vga_7fps_vt_index = 0;    
	reg_vga_10fps_vt_index = 0;  
	reg_vga_15fps_vt_index = 0;      

#if !(IS_USE_REGISTER_CONFIGURE_FILE_LSI)

	/* Section Index */
#if 0
	reg_init_qcif_index = sizeof(reg_init_qcif_table)/sizeof(u32);
	reg_init_cif_index = sizeof(reg_init_cif_table)/sizeof(u32);
#endif
	reg_init_qvga_index = sizeof(reg_init_qvga_table)/sizeof(u32);	//S1-KOR [HD VT : requested by SKT]
	reg_init_vga_index = sizeof(reg_init_vga_table)/sizeof(u32);
#if 0
	reg_init_qcif_vt_index = sizeof(reg_init_qcif_vt_table)/sizeof(u32);
	reg_init_cif_vt_index = sizeof(reg_init_cif_vt_table)/sizeof(u32);
#endif
	reg_init_qvga_vt_index = sizeof(reg_init_qvga_vt_table)/sizeof(u32);	//S1-KOR [HD VT : requested by SKT]
	reg_init_vga_vt_index = sizeof(reg_init_vga_vt_table)/sizeof(u32);
	reg_ev_m4_index = sizeof(reg_ev_m4_table)/sizeof(u32);
	reg_ev_m3_index = sizeof(reg_ev_m3_table)/sizeof(u32);
	reg_ev_m2_index = sizeof(reg_ev_m2_table)/sizeof(u32);
	reg_ev_m1_index = sizeof(reg_ev_m1_table)/sizeof(u32);
	reg_ev_default_index = sizeof(reg_ev_default_table)/sizeof(u32);
	reg_ev_p1_index = sizeof(reg_ev_p1_table)/sizeof(u32);
	reg_ev_p2_index = sizeof(reg_ev_p2_table)/sizeof(u32);
	reg_ev_p3_index = sizeof(reg_ev_p3_table)/sizeof(u32);
	reg_ev_p4_index = sizeof(reg_ev_p4_table)/sizeof(u32);
	
	reg_ev_vt_m4_index = sizeof(reg_ev_vt_m4_table)/sizeof(u32);    
	reg_ev_vt_m3_index = sizeof(reg_ev_vt_m3_table)/sizeof(u32);    
	reg_ev_vt_m2_index = sizeof(reg_ev_vt_m2_table)/sizeof(u32);    
	reg_ev_vt_m1_index = sizeof(reg_ev_vt_m1_table)/sizeof(u32);    
	reg_ev_vt_default_index = sizeof(reg_ev_vt_default_table)/sizeof(u32);    
	reg_ev_vt_p1_index = sizeof(reg_ev_vt_p1_table)/sizeof(u32);    
	reg_ev_vt_p2_index = sizeof(reg_ev_vt_p2_table)/sizeof(u32);    
	reg_ev_vt_p3_index = sizeof(reg_ev_vt_p3_table)/sizeof(u32);    
	reg_ev_vt_p4_index = sizeof(reg_ev_vt_p4_table)/sizeof(u32);    
	reg_wb_auto_index = sizeof(reg_wb_auto_table)/sizeof(u32);
	reg_wb_daylight_index = sizeof(reg_wb_daylight_table)/sizeof(u32);
	reg_wb_cloudy_index = sizeof(reg_wb_cloudy_table)/sizeof(u32);
	reg_wb_incandescent_index = sizeof(reg_wb_incandescent_table)/sizeof(u32);
	reg_wb_fluorescent_index = sizeof(reg_wb_fluorescent_table)/sizeof(u32);
	reg_effect_none_index = sizeof(reg_effect_none_table)/sizeof(u32);
	reg_effect_gray_index = sizeof(reg_effect_gray_table)/sizeof(u32);
	reg_effect_red_index = sizeof(reg_effect_red_table)/sizeof(u32);    
	reg_effect_sepia_index = sizeof(reg_effect_sepia_table)/sizeof(u32);
	reg_effect_green_index = sizeof(reg_effect_green_table)/sizeof(u32);
	reg_effect_aqua_index = sizeof(reg_effect_aqua_table)/sizeof(u32);
	reg_effect_negative_index = sizeof(reg_effect_negative_table)/sizeof(u32);
	reg_flip_none_index = sizeof(reg_flip_none_table)/sizeof(u32);
	reg_flip_water_index = sizeof(reg_flip_water_table)/sizeof(u32);
	reg_flip_mirror_index = sizeof(reg_flip_mirror_table)/sizeof(u32);
	reg_flip_water_mirror_index = sizeof(reg_flip_water_mirror_table)/sizeof(u32);
	reg_pretty_none_index = sizeof(reg_pretty_none_table)/sizeof(u32);
	reg_pretty_level1_index = sizeof(reg_pretty_level1_table)/sizeof(u32);
	reg_pretty_level2_index = sizeof(reg_pretty_level2_table)/sizeof(u32);
	reg_pretty_level3_index = sizeof(reg_pretty_level3_table)/sizeof(u32);   
	reg_pretty_vt_none_index = sizeof(reg_pretty_vt_none_table)/sizeof(u32);
	reg_pretty_vt_level1_index = sizeof(reg_pretty_vt_level1_table)/sizeof(u32);
	reg_pretty_vt_level2_index = sizeof(reg_pretty_vt_level2_table)/sizeof(u32);
	reg_pretty_vt_level3_index = sizeof(reg_pretty_vt_level3_table)/sizeof(u32);
#if 0
	reg_qcif_7fps_index = sizeof(reg_qcif_7fps_table)/sizeof(u32);
	reg_qcif_10fps_index = sizeof(reg_qcif_10fps_table)/sizeof(u32);
	reg_qcif_15fps_index = sizeof(reg_qcif_15fps_table)/sizeof(u32);    
	reg_cif_7fps_index = sizeof(reg_cif_7fps_table)/sizeof(u32);
	reg_cif_10fps_index = sizeof(reg_cif_10fps_table)/sizeof(u32);
	reg_cif_15fps_index = sizeof(reg_cif_15fps_table)/sizeof(u32);    
#endif
	//S1-KOR [HD VT : requested by SKT]
	reg_qvga_7fps_index = sizeof(reg_qvga_7fps_table)/sizeof(u32);
	reg_qvga_10fps_index = sizeof(reg_qvga_10fps_table)/sizeof(u32);
	reg_qvga_15fps_index = sizeof(reg_qvga_15fps_table)/sizeof(u32);    

	reg_vga_7fps_index = sizeof(reg_vga_7fps_table)/sizeof(u32);    
	reg_vga_10fps_index = sizeof(reg_vga_10fps_table)/sizeof(u32);     
	reg_vga_15fps_index = sizeof(reg_vga_15fps_table)/sizeof(u32); 
	//add
	reg_vga_7fps_vt_index = sizeof(reg_vga_7fps_vt_table)/sizeof(u32);    
	reg_vga_10fps_vt_index = sizeof(reg_vga_10fps_vt_table)/sizeof(u32);     
	reg_vga_15fps_vt_index = sizeof(reg_vga_15fps_vt_table)/sizeof(u32); 

#else		//#if !(IS_USE_REGISTER_CONFIGURE_FILE_LSI)

#if 0
	memset(&reg_init_qcif_table, 0, sizeof(reg_init_qcif_table));
	memset(&reg_init_cif_table, 0, sizeof(reg_init_cif_table));
#endif
	memset(&reg_init_qvga_table, 0, sizeof(reg_init_qvga_table));	//S1-KOR [HD VT : requested by SKT]
	memset(&reg_init_vga_table, 0, sizeof(reg_init_vga_table));
#if 0
	memset(&reg_init_qcif_vt_table, 0, sizeof(reg_init_qcif_vt_table));
	memset(&reg_init_cif_vt_table, 0, sizeof(reg_init_cif_vt_table));
#endif
	memset(&reg_init_qvga_vt_table, 0, sizeof(reg_init_qvga_vt_table));	//S1-KOR [HD VT : requested by SKT]
	memset(&reg_init_vga_vt_table, 0, sizeof(reg_init_vga_vt_table));
	memset(&reg_wb_auto_table, 0, sizeof(reg_wb_auto_table));
	memset(&reg_wb_daylight_table, 0, sizeof(reg_wb_daylight_table));
	memset(&reg_wb_cloudy_table, 0, sizeof(reg_wb_cloudy_table));
	memset(&reg_wb_incandescent_table, 0, sizeof(reg_wb_incandescent_table));
	memset(&reg_wb_fluorescent_table, 0, sizeof(reg_wb_fluorescent_table));
	memset(&reg_ev_m4_table, 0, sizeof(reg_ev_m4_table));
	memset(&reg_ev_m3_table, 0, sizeof(reg_ev_m3_table));
	memset(&reg_ev_m2_table, 0, sizeof(reg_ev_m2_table));
	memset(&reg_ev_m1_table, 0, sizeof(reg_ev_m1_table));
	memset(&reg_ev_default_table, 0, sizeof(reg_ev_default_table));
	memset(&reg_ev_p1_table, 0, sizeof(reg_ev_p1_table));
	memset(&reg_ev_p2_table, 0, sizeof(reg_ev_p2_table));
	memset(&reg_ev_p3_table, 0, sizeof(reg_ev_p3_table));
	memset(&reg_ev_p4_table, 0, sizeof(reg_ev_p4_table));
	
	memset(&reg_ev_vt_m4_table, 0, sizeof(reg_ev_vt_m4_table));
	memset(&reg_ev_vt_m3_table, 0, sizeof(reg_ev_vt_m3_table));
	memset(&reg_ev_vt_m2_table, 0, sizeof(reg_ev_vt_m2_table));
	memset(&reg_ev_vt_m1_table, 0, sizeof(reg_ev_vt_m1_table));
	memset(&reg_ev_vt_default_table, 0, sizeof(reg_ev_vt_default_table));
	memset(&reg_ev_vt_p1_table, 0, sizeof(reg_ev_vt_p1_table));
	memset(&reg_ev_vt_p2_table, 0, sizeof(reg_ev_vt_p2_table));
	memset(&reg_ev_vt_p3_table, 0, sizeof(reg_ev_vt_p3_table));
	memset(&reg_ev_vt_p4_table, 0, sizeof(reg_ev_vt_p4_table));
	memset(&reg_effect_none_table, 0, sizeof(reg_effect_none_table));
	memset(&reg_effect_gray_table, 0, sizeof(reg_effect_gray_table));
	memset(&reg_effect_red_table, 0, sizeof(reg_effect_red_table));    
	memset(&reg_effect_sepia_table, 0, sizeof(reg_effect_sepia_table));
	memset(&reg_effect_green_table, 0, sizeof(reg_effect_green_table));
	memset(&reg_effect_aqua_table, 0, sizeof(reg_effect_aqua_table));
	memset(&reg_effect_negative_table, 0, sizeof(reg_effect_negative_table));
	memset(&reg_flip_none_table, 0, sizeof(reg_flip_none_table));
	memset(&reg_flip_water_table, 0, sizeof(reg_flip_water_table));
	memset(&reg_flip_mirror_table, 0, sizeof(reg_flip_mirror_table));
	memset(&reg_flip_water_mirror_table, 0, sizeof(reg_flip_water_mirror_table));
	memset(&reg_pretty_none_table, 0, sizeof(reg_pretty_none_table));
	memset(&reg_pretty_level1_table, 0, sizeof(reg_pretty_level1_table));
	memset(&reg_pretty_level2_table, 0, sizeof(reg_pretty_level2_table));
	memset(&reg_pretty_level3_table, 0, sizeof(reg_pretty_level3_table));
	memset(&reg_pretty_vt_none_table, 0, sizeof(reg_pretty_vt_none_table));
	memset(&reg_pretty_vt_level1_table, 0, sizeof(reg_pretty_vt_level1_table));
	memset(&reg_pretty_vt_level2_table, 0, sizeof(reg_pretty_vt_level2_table));
	memset(&reg_pretty_vt_level3_table, 0, sizeof(reg_pretty_vt_level3_table));  
#if 0
	memset(&reg_qcif_7fps_table, 0, sizeof(reg_qcif_7fps_table));  
	memset(&reg_qcif_10fps_table, 0, sizeof(reg_qcif_10fps_table)); 
	memset(&reg_qcif_15fps_table, 0, sizeof(reg_qcif_15fps_table)); 
	memset(&reg_cif_7fps_table, 0, sizeof(reg_cif_7fps_table));
	memset(&reg_cif_10fps_table, 0, sizeof(reg_cif_10fps_table));
	memset(&reg_cif_15fps_table, 0, sizeof(reg_cif_15fps_table));
#endif
	//S1-KOR [HD VT : requested by SKT]
	memset(&reg_qvga_7fps_table, 0, sizeof(reg_qvga_7fps_table));
	memset(&reg_qvga_10fps_table, 0, sizeof(reg_qvga_10fps_table));
	memset(&reg_qvga_15fps_table, 0, sizeof(reg_qvga_15fps_table));    

	memset(&reg_vga_7fps_table, 0, sizeof(reg_vga_7fps_table));
	memset(&reg_vga_10fps_table, 0, sizeof(reg_vga_10fps_table));
	memset(&reg_vga_15fps_table, 0, sizeof(reg_vga_15fps_table));
	
	//add
	memset(&reg_vga_7fps_vt_table, 0, sizeof(reg_vga_7fps_vt_table));
	memset(&reg_vga_10fps_vt_table, 0, sizeof(reg_vga_10fps_vt_table));
	memset(&reg_vga_15fps_vt_table, 0, sizeof(reg_vga_15fps_vt_table));
#endif    

	return 0;
}

#if 0	// temporary delete
static const char *s5ka3dfx_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};

static const char *s5ka3dfx_querymenu_effect_mode[] = {
	"Effect Sepia", "Effect Aqua", "Effect Monochrome",
	"Effect Negative", "Effect Sketch", NULL
};

static const char *s5ka3dfx_querymenu_ev_bias_mode[] = {
	"-3EV",	"-2,1/2EV", "-2EV", "-1,1/2EV",
	"-1EV", "-1/2EV", "0", "1/2EV",
	"1EV", "1,1/2EV", "2EV", "2,1/2EV",
	"3EV", NULL
};
#endif

static struct v4l2_queryctrl s5ka3dfx_controls[] = {
#if 0	// temporary delete
	{
		/*
		 * For now, we just support in preset type
		 * to be close to generic WB system,
		 * we define color temp range for each preset
		 */
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "White balance in kelvin",
		.minimum = 0,
		.maximum = 10000,
		.step = 1,
		.default_value = 0,	/* FIXME */
	},
	{
		.id = V4L2_CID_WHITE_BALANCE_PRESET,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "White balance preset",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_wb_preset) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Exposure bias",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_ev_bias_mode) - 2,
		.step = 1,
		.default_value = (ARRAY_SIZE(s5ka3dfx_querymenu_ev_bias_mode) - 2) / 2,	/* 0 EV */
	},
	{
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "Image Effect",
		.minimum = 0,
		.maximum = ARRAY_SIZE(s5ka3dfx_querymenu_effect_mode) - 2,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sharpness",
		.minimum = 0,
		.maximum = 4,
		.step = 1,
		.default_value = 2,
	},
#endif	
};

const char **s5ka3dfx_ctrl_get_menu(u32 id)
{
	printk(KERN_DEBUG "s5ka3dfx_ctrl_get_menu is called... id : %d \n", id);

	switch (id) {
#if 0	// temporary delete
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return s5ka3dfx_querymenu_wb_preset;

	case V4L2_CID_COLORFX:
		return s5ka3dfx_querymenu_effect_mode;

	case V4L2_CID_EXPOSURE:
		return s5ka3dfx_querymenu_ev_bias_mode;
#endif
	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *s5ka3dfx_find_qctrl(int id)
{
	int i;

	printk(KERN_DEBUG "s5ka3dfx_find_qctrl is called...  id : %d \n", id);

	for (i = 0; i < ARRAY_SIZE(s5ka3dfx_controls); i++)
		if (s5ka3dfx_controls[i].id == id)
			return &s5ka3dfx_controls[i];

	return NULL;
}

static int s5ka3dfx_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	printk(KERN_DEBUG "s5ka3dfx_queryctrl is called... \n");

	for (i = 0; i < ARRAY_SIZE(s5ka3dfx_controls); i++) {
		if (s5ka3dfx_controls[i].id == qc->id) {
			memcpy(qc, &s5ka3dfx_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int s5ka3dfx_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	printk(KERN_DEBUG "s5ka3dfx_querymenu is called... \n");

	qctrl.id = qm->id;
	s5ka3dfx_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, s5ka3dfx_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int s5ka3dfx_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	printk(KERN_DEBUG "s5ka3dfx_s_crystal_freq is called... \n");

	return err;
}

static int s5ka3dfx_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_g_fmt is called... \n");

	return err;
}

#if 1	//S1-KOR [HD VT : requested by SKT]
static int s5ka3dfx_get_framesize_index(struct v4l2_subdev *sd);
static int s5ka3dfx_set_framesize_index(struct v4l2_subdev *sd, unsigned int index);


static int s5ka3dfx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;
	struct s5ka3dfx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int framesize_index = -1;

	printk(KERN_DEBUG "s5ka3dfx_s_fmt is called... \n");

	if(fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG && fmt->fmt.pix.colorspace != V4L2_COLORSPACE_JPEG){
		dev_err(&client->dev, "%s: mismatch in pixelformat and colorspace\n", __func__);
		return -EINVAL;
	}

	state->pix.width = fmt->fmt.pix.width;
	state->pix.height = fmt->fmt.pix.height;
	
	state->pix.pixelformat = fmt->fmt.pix.pixelformat;

	framesize_index = s5ka3dfx_get_framesize_index(sd);

	err = s5ka3dfx_set_framesize_index(sd, framesize_index);
	if(err < 0){
		dev_err(&client->dev, "%s: set_framesize_index failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}
#else
static int s5ka3dfx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_s_fmt is called... \n");

	return err;
}
#endif

static int s5ka3dfx_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct  s5ka3dfx_state *state = to_state(sd);
	int num_entries = sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize);	
	struct s5ka3dfx_enum_framesize *elem;	
	int index = 0;
	int i = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_framesizes is called... \n");

	/* The camera interface should read this value, this is the resolution
 	 * at which the sensor would provide framedata to the camera i/f
 	 *
 	 * In case of image capture, this returns the default camera resolution (WVGA)
 	 */
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	index = state->framesize_index;

	for(i = 0; i < num_entries; i++){
		elem = &s5ka3dfx_framesize_list[i];
		if(elem->index == index){
			fsize->discrete.width = s5ka3dfx_framesize_list[index].width;
			fsize->discrete.height = s5ka3dfx_framesize_list[index].height;
			return 0;
		}
	}

	return -EINVAL;
}


static int s5ka3dfx_enum_frameintervals(struct v4l2_subdev *sd, 
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_frameintervals is called... \n");
	
	return err;
}

static int s5ka3dfx_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_fmt is called... \n");

	return err;
}

static int s5ka3dfx_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	printk(KERN_DEBUG "s5ka3dfx_enum_fmt is called... \n");

	return err;
}

static int s5ka3dfx_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	return err;
}

static int s5ka3dfx_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s: numerator %d, denominator: %d\n", \
		__func__, param->parm.capture.timeperframe.numerator, \
		param->parm.capture.timeperframe.denominator);

	return err;
}

static int s5ka3dfx_get_framesize_index(struct v4l2_subdev *sd)
{
	int i = 0;
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_enum_framesize *frmsize;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Check for video/image mode */
	for(i = 0; i < (sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize)); i++)
	{
		frmsize = &s5ka3dfx_framesize_list[i];
#if 1	//S1-KOR [HD VT : requested by SKT]
		if(frmsize->width == state->pix.width && frmsize->height == state->pix.height)
		{
			printk(KERN_DEBUG "frmsize->index(%d) \n", frmsize->index);
			return frmsize->index;
		}
#else
		if(frmsize->width >= state->pix.width && frmsize->height >= state->pix.height){
			return frmsize->index;
		} 
#endif
	}
	
	v4l_info(client, "%s: s5ka3dfx_framesize_list[%d].index = %d\n", __func__, i - 1, s5ka3dfx_framesize_list[i].index);
	
	/* FIXME: If it fails, return the last index. */
#if 1	//S1-KOR [HD VT : requested by SKT]
	return S5KA3DFX_PREVIEW_VGA;
#else
	return s5ka3dfx_framesize_list[i-1].index;
#endif
}

/* This function is called from the s_ctrl api
 * Given the index, it checks if it is a valid index.
 * On success, it returns 0.
 * On Failure, it returns -EINVAL
 */
static int s5ka3dfx_set_framesize_index(struct v4l2_subdev *sd, unsigned int index)
{
	int i = 0;
	struct s5ka3dfx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s: index = %d\n", __func__, index);

	/* Check for video/image mode */
	for(i = 0; i < (sizeof(s5ka3dfx_framesize_list)/sizeof(struct s5ka3dfx_enum_framesize)); i++)
	{
		if(s5ka3dfx_framesize_list[i].index == index){
			state->framesize_index = index; 
			state->pix.width = s5ka3dfx_framesize_list[i].width;
			state->pix.height = s5ka3dfx_framesize_list[i].height;
			dev_dbg(&client->dev, "index %d width %d height %d \n", state->framesize_index, state->pix.width, state->pix.height);
			return 0;
		} 
	} 
	
	return -EINVAL;
}

/* set sensor register values for adjusting brightness */
static int s5ka3dfx_set_brightness(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
 	//s1_camera [For VT camera]
	struct s5ka3dfx_state *state = to_state(sd);
//	struct s5ka3dfx_userset userset = state->userset;

	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d state->vt_mode %d \n", __func__, ctrl->value, state->vt_mode);

	if((state->vt_mode == VT_MODE_S) || (state->vt_mode == VT_MODE_NS)) 	//s1_camera [For VT camera]	//HD VT : requested by SKT
	{
		switch(ctrl->value)
		{
		case EV_MINUS_4:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_m4_table, reg_ev_vt_m4_index);
			break;

		case EV_MINUS_3:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_m3_table, reg_ev_vt_m3_index);
			break;

		case EV_MINUS_2:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_m2_table, reg_ev_vt_m2_index);
			break;

		case EV_MINUS_1:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_m1_table, reg_ev_vt_m1_index);
			break;

		case EV_DEFAULT:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_default_table, reg_ev_vt_default_index);
			break;

		case EV_PLUS_1:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_p1_table, reg_ev_vt_p1_index);
			break;

		case EV_PLUS_2:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_p2_table, reg_ev_vt_p2_index);
			break;

		case EV_PLUS_3:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_p3_table, reg_ev_vt_p3_index);
			break;

		case EV_PLUS_4:
			err = s5ka3dfx_write_regs(sd, reg_ev_vt_p4_table, reg_ev_vt_p4_index);
			break;
			
		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	else
	{
		switch(ctrl->value)
		{
		case EV_MINUS_4:
			err = s5ka3dfx_write_regs(sd, reg_ev_m4_table, reg_ev_m4_index);
			break;

		case EV_MINUS_3:
			err = s5ka3dfx_write_regs(sd, reg_ev_m3_table, reg_ev_m3_index);
			break;

		case EV_MINUS_2:
			err = s5ka3dfx_write_regs(sd, reg_ev_m2_table, reg_ev_m2_index);
			break;

		case EV_MINUS_1:
			err = s5ka3dfx_write_regs(sd, reg_ev_m1_table, reg_ev_m1_index);
			break;

		case EV_DEFAULT:
			err = s5ka3dfx_write_regs(sd, reg_ev_default_table, reg_ev_default_index);
			break;

		case EV_PLUS_1:
			err = s5ka3dfx_write_regs(sd, reg_ev_p1_table, reg_ev_p1_index);
			break;

		case EV_PLUS_2:
			err = s5ka3dfx_write_regs(sd, reg_ev_p2_table, reg_ev_p2_index);
			break;

		case EV_PLUS_3:
			err = s5ka3dfx_write_regs(sd, reg_ev_p3_table, reg_ev_p3_index);
			break;

		case EV_PLUS_4:
			err = s5ka3dfx_write_regs(sd, reg_ev_p4_table, reg_ev_p4_index);
			break;
			
		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}

	return err;
}

/* set sensor register values for adjusting whitebalance, both auto and manual */
static int s5ka3dfx_set_wb(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s:  value : %d \n", __func__, ctrl->value);

	switch(ctrl->value)
	{
	case WHITE_BALANCE_AUTO:
		err = s5ka3dfx_write_regs(sd, reg_wb_auto_table, reg_wb_auto_index);
		break;

	case WHITE_BALANCE_SUNNY:
		err = s5ka3dfx_write_regs(sd, reg_wb_daylight_table, reg_wb_daylight_index);
		break;

	case WHITE_BALANCE_CLOUDY:
		err = s5ka3dfx_write_regs(sd, reg_wb_cloudy_table, reg_wb_cloudy_index);
		break;

	case WHITE_BALANCE_TUNGSTEN:
		err = s5ka3dfx_write_regs(sd, reg_wb_incandescent_table, reg_wb_incandescent_index);
		break;

	case WHITE_BALANCE_FLUORESCENT:
		err = s5ka3dfx_write_regs(sd, reg_wb_fluorescent_table, reg_wb_fluorescent_index);
		break;

	default:
		dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
		err = 0;
		break;
	}

	return err;
}

/* set sensor register values for adjusting color effect */
static int s5ka3dfx_set_effect(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);

	switch(ctrl->value)
	{
	case IMAGE_EFFECT_NONE:
		err = s5ka3dfx_write_regs(sd, reg_effect_none_table, reg_effect_none_index);
		break;

	case IMAGE_EFFECT_BNW:		//Gray
		err = s5ka3dfx_write_regs(sd, reg_effect_gray_table, reg_effect_gray_index);
		break;

	case IMAGE_EFFECT_SEPIA:
		err = s5ka3dfx_write_regs(sd, reg_effect_sepia_table, reg_effect_sepia_index);
		break;

	case IMAGE_EFFECT_AQUA:
		err = s5ka3dfx_write_regs(sd, reg_effect_aqua_table, reg_effect_aqua_index);
		break;

	case IMAGE_EFFECT_NEGATIVE:
		err = s5ka3dfx_write_regs(sd, reg_effect_negative_table, reg_effect_negative_index);
		break;

	case IMAGE_EFFECT_RED:		// Red 로 대체
		err = s5ka3dfx_write_regs(sd, reg_effect_red_table, reg_effect_red_index);
		break;

	case IMAGE_EFFECT_GREEN:		// Green 대체
		err = s5ka3dfx_write_regs(sd, reg_effect_green_table, reg_effect_green_index);
		break;

	default:
		dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
		err = 0;
		break;
	}

	return err;
}

/* set sensor register values for flip setting */
static int s5ka3dfx_set_flip(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);

	switch(ctrl->value)
	{
	case 0:	//FLIP_MODE_OFF:
		err = s5ka3dfx_write_regs(sd, reg_flip_none_table, reg_flip_none_index);
		break;

	case 1:	//FLIP_MODE_H:
		err = s5ka3dfx_write_regs(sd, reg_flip_mirror_table, reg_flip_mirror_index);
		break;

	case 2:	//FLIP_MODE_V:
		err = s5ka3dfx_write_regs(sd, reg_flip_water_table, reg_flip_water_index);
		break;

	case 3:	//FLIP_MODE_HV:
		err = s5ka3dfx_write_regs(sd, reg_flip_water_mirror_table, reg_flip_water_mirror_index);
		break;

	default:
		dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
		err = 0;
		break;
	}

	return err;
}


/* set sensor register values for frame rate(fps) setting */
static int s5ka3dfx_set_frame_rate(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);

	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);
	
#if 1	//S1-KOR [HD VT : requested by SKT]
	printk(KERN_DEBUG "state->framesize_index : %d \n", state->framesize_index);

	if(state->framesize_index == S5KA3DFX_PREVIEW_QVGA)
	{
		switch(ctrl->value)
		{
		case 7:
			err = s5ka3dfx_write_regs(sd, reg_qvga_7fps_table, reg_qvga_7fps_index);
			break;

		case 10:
			err = s5ka3dfx_write_regs(sd, reg_qvga_10fps_table, reg_qvga_10fps_index);
			break;

		case 15:
			err = s5ka3dfx_write_regs(sd, reg_qvga_15fps_table, reg_qvga_15fps_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	else if(state->framesize_index == S5KA3DFX_PREVIEW_VGA)
	{
		switch(ctrl->value)
		{
		case 7:
			err = s5ka3dfx_write_regs(sd, reg_vga_7fps_table, reg_vga_7fps_index);
			break;

		case 10:
			err = s5ka3dfx_write_regs(sd, reg_vga_10fps_table, reg_vga_10fps_index);
			break;

		case 15:
			err = s5ka3dfx_write_regs(sd, reg_vga_15fps_table, reg_vga_15fps_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}		
	}
	else
	{
		dev_err(&client->dev, "%s: Not Support Preview Size : %d \n", __func__, state->framesize_index);
	}

#else
	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode);

	//add
	if(state->vt_mode == 1)
	{
		switch(ctrl->value)
		{
		case 7:
			err = s5ka3dfx_write_regs(sd, reg_vga_7fps_vt_table, reg_vga_7fps_vt_index);
			break;

		case 10:
			err = s5ka3dfx_write_regs(sd, reg_vga_10fps_vt_table, reg_vga_10fps_vt_index);
			break;

		case 15:
			err = s5ka3dfx_write_regs(sd, reg_vga_15fps_vt_table, reg_vga_15fps_vt_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	else
	{
		switch(ctrl->value)
		{
		case 7:
			err = s5ka3dfx_write_regs(sd, reg_vga_7fps_table, reg_vga_7fps_index);
			break;

		case 10:
			err = s5ka3dfx_write_regs(sd, reg_vga_10fps_table, reg_vga_10fps_index);
			break;

		case 15:
			err = s5ka3dfx_write_regs(sd, reg_vga_15fps_table, reg_vga_15fps_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}		
	}
#endif

	return err;
}

/* set sensor register values for adjusting blur effect */
static int s5ka3dfx_set_blur(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: value : %d \n", __func__, ctrl->value);
	
	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode);
	if(state->vt_mode == VT_MODE_S)
	{
		switch(ctrl->value)
		{
		case BLUR_LEVEL_0:
			err = s5ka3dfx_write_regs(sd, reg_pretty_vt_none_table, reg_pretty_vt_none_index);
			break;

		case BLUR_LEVEL_1:
			err = s5ka3dfx_write_regs(sd, reg_pretty_vt_level1_table, reg_pretty_vt_level1_index);
			break;

		case BLUR_LEVEL_2:
			err = s5ka3dfx_write_regs(sd, reg_pretty_vt_level2_table, reg_pretty_vt_level2_index);
			break;

		case BLUR_LEVEL_3:
			err = s5ka3dfx_write_regs(sd, reg_pretty_vt_level3_table, reg_pretty_vt_level3_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}

	}
	else
	{
		switch(ctrl->value)
		{
		case BLUR_LEVEL_0:
			err = s5ka3dfx_write_regs(sd, reg_pretty_none_table, reg_pretty_none_index);
			break;

		case BLUR_LEVEL_1:
			err = s5ka3dfx_write_regs(sd, reg_pretty_level1_table, reg_pretty_level1_index);
			break;

		case BLUR_LEVEL_2:
			err = s5ka3dfx_write_regs(sd, reg_pretty_level2_table, reg_pretty_level2_index);
			break;

		case BLUR_LEVEL_3:
			err = s5ka3dfx_write_regs(sd, reg_pretty_level3_table, reg_pretty_level3_index);
			break;

		default:
			dev_dbg(&client->dev, "%s: Not Support value \n", __func__);
			err = 0;
			break;
		}
	}
	return err;
}

static int s5ka3dfx_adjust_indoor_awb(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;
	unsigned char page_buf[2] = {0xEF, 0x00};
	unsigned char write_buf[2] = {0x00, 0x00};
	unsigned char read_buf[1] = {0x00};

	dev_dbg(&client->dev, "%s: \n", __func__);

	// page 3
	page_buf[1] = 0x03;
	err = s5ka3dfx_i2c_write(sd, page_buf, sizeof(page_buf));
	if(err < 0)
	{
		dev_err(&client->dev, "%s: i2c write fail ... 1 \n", __func__);
		return err;
	}

	write_buf[0] = 0x59;
	err = s5ka3dfx_i2c_read(sd, write_buf, 1, read_buf, 1);
	if(err < 0)
	{
		dev_err(&client->dev, "%s: i2c read fail ... 1 \n", __func__);
		return err;
	}

	dev_dbg(&client->dev, "%s: 1st read_buf : %d \n", __func__, read_buf[0]);
	
	if(read_buf[0] != 1)		//if condition is true, adjust register value for indoor awb.
	{
		// page 0
		page_buf[1] = 0x00;
		err = s5ka3dfx_i2c_write(sd, page_buf, sizeof(page_buf));
		if(err < 0)
		{
			dev_err(&client->dev, "%s: i2c write fail ... 2 \n", __func__);
			return err;
		}

		write_buf[0] = 0x42;
		read_buf[0] = 0x00;
		err = s5ka3dfx_i2c_read(sd, write_buf, 1, read_buf, 1);	
		if(err < 0)
		{
			dev_err(&client->dev, "%s: i2c read fail ... 2 \n", __func__);
			return err;
		}

		dev_dbg(&client->dev, "%s: 2nd read_buf : %d \n", __func__, read_buf[0]);
		write_buf[0] = 0x42;
		write_buf[1] = read_buf[0] + 0x05;
		err = s5ka3dfx_i2c_write(sd, write_buf, 2);
		if(err < 0)
		{
			dev_err(&client->dev, "%s: i2c write fail ... 3\n", __func__);
			return err;
		}

		mdelay(200);						//For skip 3-frames

	}

	return 0;
}

static int s5ka3dfx_check_dataline_stop(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	int err = -EINVAL, i;

	struct s5ka3dfx_reg reg_set;
	int reg_test_pattern_index = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	reg_test_pattern_index = sizeof(reg_test_pattern_stop_table)/sizeof(u32);

	for (i = 0; i < reg_test_pattern_index; i++) {
		reg_set.addr = (unsigned char)((reg_test_pattern_stop_table[i] & 0x0000ff00)>> 8);
		reg_set.val = (unsigned char)(reg_test_pattern_stop_table[i] & 0x000000ff);
		
		err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n",	__func__);
			return -EIO;
		}	
	}
	state->check_dataline = 0;

	mdelay(5);

	err = s5ka3dfx_init(sd, 0);
	if (err < 0)
	{
		v4l_info(client, "%s: After TestPattern, init_register set failed\n", __func__);
		return -EIO;
	}

	mdelay(5);

	return err;
}

/* if you need, add below some functions below */

static int s5ka3dfx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_userset userset = state->userset;
	int err = -EINVAL;

	dev_dbg(&client->dev, "%s: id : 0x%08x \n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		err = 0;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		err = 0;
		break;

	case V4L2_CID_WHITE_BALANCE_PRESET:
		ctrl->value = userset.manual_wb;
		err = 0;
		break;

	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		err = 0;
		break;

	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		err = 0;
		break;

	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		err = 0;
		break;

	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.saturation;
		err = 0;
		break;

#if 0
	case V4L2_CID_CAM_FRAMESIZE_INDEX:
		ctrl->value = s5ka3dfx_get_framesize_index(sd);
		err = 0;
		break;
#endif

	default:
		dev_dbg(&client->dev, "%s: NO SUCH in S5KA3DFX_G_CTRL\n", __func__);
		err = 0;		// This is not error
		break;
	}
	
	return err;
}

static int s5ka3dfx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
//	struct s5ka3dfx_userset userset = state->userset;
	int err = -EINVAL;

	printk(KERN_DEBUG "s5ka3dfx_s_ctrl() : ctrl->id 0x%08x, ctrl->value %d \n",ctrl->id, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_BRIGHTNESS:	//V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_BRIGHTNESS\n", __func__);
		err = s5ka3dfx_set_brightness(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:	//V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", __func__);
		err = s5ka3dfx_set_wb(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_EFFECT:	//V4L2_CID_COLORFX:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_EFFECT\n", __func__);
		err = s5ka3dfx_set_effect(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_FRAME_RATE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_FRAME_RATE\n", __func__);
		err = s5ka3dfx_set_frame_rate(sd, ctrl);	
		break;

	case V4L2_CID_CAMERA_VGA_BLUR:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_VGA_BLUR\n", __func__);
		err = s5ka3dfx_set_blur(sd, ctrl);			
		break;

	case V4L2_CID_CAMERA_VT_MODE:
		state->vt_mode = ctrl->value;
#ifdef VGA_VT_CAPTURE
		state->vt_mode = VT_MODE_S;
#endif
		
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_VT_MODE : state->vt_mode %d \n", __func__, state->vt_mode);
		err = 0;
		break;

	case V4L2_CID_CAMERA_CHECK_DATALINE:
		state->check_dataline = ctrl->value;
		err = 0;
		break;	

	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
		err = s5ka3dfx_check_dataline_stop(sd);
		break;

	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if(state->check_previewdata == 0)
		{
			err = 0;
		}
		else
		{
			err = -EIO;	
		}
		break;

	//s1_camera [ Defense process by ESD input ] _[
	case V4L2_CID_CAMERA_RESET:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_RESET \n", __func__);
		err = s5ka3dfx_reset(sd);
		break;
	// _]

	default:
		dev_dbg(&client->dev, "%s: no support control in camera sensor, S5KA3DFX\n", __func__);
		//err = -ENOIOCTLCMD;
		err = 0;
		break;
	}

	if (err < 0)
		goto out;
	else
		return 0;

out:
	dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);
	return err;
}

static int s5ka3dfx_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);

	int err = -EINVAL, i;
	
	struct s5ka3dfx_reg reg_set;

	int reg_test_pattern_index = 0;

	//v4l_info(client, "%s: camera initialization start : state->vt_mode %d \n", __func__, state->vt_mode);
	printk(KERN_DEBUG "camera initialization start, state->vt_mode : %d \n", state->vt_mode); 
	s5ka3dfx_init_reg_table_index(sd);
#if (IS_USE_REGISTER_CONFIGURE_FILE_LSI)		//vga tuning
	s5ka3dfx_make_table();
#endif

	printk(KERN_DEBUG "state->vt_mode : %d \n", state->vt_mode); 	//s1_camera [For VT camera]
	printk(KERN_DEBUG "state->check_dataline : %d \n", state->check_dataline); 

	if((state->vt_mode == VT_MODE_S) || (state->vt_mode == VT_MODE_NS))		//HD VT : requested by SKT
	{
#if 1	//S1-KOR [HD VT : requested by SKT]
		printk(KERN_DEBUG "framesize_index %d \n", state->framesize_index);

		switch(state->framesize_index)
		{
		case S5KA3DFX_PREVIEW_QVGA:
			for (i = 0; i < reg_init_qvga_vt_index; i++) {
				reg_set.addr = (unsigned char)((reg_init_qvga_vt_table[i] & 0x0000ff00)>> 8);
				reg_set.val = (unsigned char)(reg_init_qvga_vt_table[i] & 0x000000ff);

				err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n", __func__);
					break;
				}
			}
			break;

		case S5KA3DFX_PREVIEW_VGA:
		default:
			for (i = 0; i < reg_init_vga_vt_index; i++) {
				reg_set.addr = (unsigned char)((reg_init_vga_vt_table[i] & 0x0000ff00)>> 8);
				reg_set.val = (unsigned char)(reg_init_vga_vt_table[i] & 0x000000ff);

				err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n", __func__);
					break;
				}
			}
			break;
		}
#else
		for (i = 0; i < reg_init_vga_vt_index; i++) {
			reg_set.addr = (unsigned char)((reg_init_vga_vt_table[i] & 0x0000ff00)>> 8);
			reg_set.val = (unsigned char)(reg_init_vga_vt_table[i] & 0x000000ff);

			err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
			if (err < 0)
			{
				v4l_info(client, "%s: register set failed\n", \
				__func__);

				break;
			}
		}
#endif
	}
	else
	{
		if(state->check_dataline)		// Output Test Pattern from VGA sensor
		{

			reg_test_pattern_index = sizeof(reg_test_pattern_start_table)/sizeof(u32);

			for (i = 0; i < reg_test_pattern_index; i++) {
				
				reg_set.addr = (unsigned char)((reg_test_pattern_start_table[i] & 0x0000ff00)>> 8);
				reg_set.val = (unsigned char)(reg_test_pattern_start_table[i] & 0x000000ff);
				
				err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n",	__func__);
					break;
				}	
			}
		}
		else
		{
			
			for (i = 0; i < reg_init_vga_index; i++) {
				reg_set.addr = (unsigned char)((reg_init_vga_table[i] & 0x0000ff00)>> 8);
				reg_set.val = (unsigned char)(reg_init_vga_table[i] & 0x000000ff);

				err = s5ka3dfx_i2c_write(sd, (char *)&reg_set, sizeof(reg_set));
				if (err < 0)
				{
					v4l_info(client, "%s: register set failed\n",	__func__);
					break;
				}
			}
		}
	}

	if (err < 0) {
		//This is preview fail 
		state->check_previewdata = 100;
		v4l_err(client, "%s: camera initialization failed. err(%d)\n", \
			__func__, state->check_previewdata);
		return -EIO;	/* FIXME */	
	}

	//This is preview success
	state->check_previewdata = 0;
	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time. except for version checking
 * NOTE: version checking is optional
 */
static int s5ka3dfx_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5ka3dfx_state *state = to_state(sd);
	struct s5ka3dfx_platform_data *pdata;

	dev_dbg(&client->dev, "fetching platform data\n");

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {
		state->pix.width = pdata->default_width;
		state->pix.height = pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pix.pixelformat = DEFAULT_FMT;
	else
		state->pix.pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = 24000000;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	if (!pdata->is_mipi) {
		state->is_mipi = 0;
		dev_dbg(&client->dev, "parallel mode\n");
	} else
		state->is_mipi = pdata->is_mipi;

	return 0;
}

static const struct v4l2_subdev_core_ops s5ka3dfx_core_ops = {
	.init = s5ka3dfx_init,	/* initializing API */
	.s_config = s5ka3dfx_s_config,	/* Fetch platform data */
	.queryctrl = s5ka3dfx_queryctrl,
	.querymenu = s5ka3dfx_querymenu,
	.g_ctrl = s5ka3dfx_g_ctrl,
	.s_ctrl = s5ka3dfx_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5ka3dfx_video_ops = {
	.s_crystal_freq = s5ka3dfx_s_crystal_freq,
	.g_fmt = s5ka3dfx_g_fmt,
	.s_fmt = s5ka3dfx_s_fmt,
	.enum_framesizes = s5ka3dfx_enum_framesizes,
	.enum_frameintervals = s5ka3dfx_enum_frameintervals,
	.enum_fmt = s5ka3dfx_enum_fmt,
	.try_fmt = s5ka3dfx_try_fmt,
	.g_parm = s5ka3dfx_g_parm,
	.s_parm = s5ka3dfx_s_parm,
};

static const struct v4l2_subdev_ops s5ka3dfx_ops = {
	.core = &s5ka3dfx_core_ops,
	.video = &s5ka3dfx_video_ops,
};

/*
 * s5ka3dfx_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int s5ka3dfx_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct s5ka3dfx_state *state;
	struct v4l2_subdev *sd;

	state = kzalloc(sizeof(struct s5ka3dfx_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, S5KA3DFX_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5ka3dfx_ops);

	dev_dbg(&client->dev, "s5ka3dfx has been probed\n");
	return 0;
}


static int s5ka3dfx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id s5ka3dfx_id[] = {
	{ S5KA3DFX_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5ka3dfx_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = S5KA3DFX_DRIVER_NAME,
	.probe = s5ka3dfx_probe,
	.remove = s5ka3dfx_remove,
	.id_table = s5ka3dfx_id,
};

MODULE_DESCRIPTION("Samsung Electronics S5KA3DFX UXGA camera driver");
MODULE_AUTHOR("Jinsung Yang <jsgood.yang@samsung.com>");
MODULE_LICENSE("GPL");

