/* MAX17043 */
#define MAX17043

/* Slave address */
//#define MAX17043_SLAVE_ADDR	0x6D

/* Register address */
#define VCELL0_REG			0x02
#define VCELL1_REG			0x03
#define SOC0_REG			0x04
#define SOC1_REG			0x05
#define MODE0_REG			0x06
#define MODE1_REG			0x07
#define RCOMP0_REG			0x0C
#define RCOMP1_REG			0x0D
#define CMD0_REG			0xFE
#define CMD1_REG			0xFF

unsigned int FGPureSOC = 0;
unsigned int prevFGSOC = 0;
//unsigned int full_soc = 9540; //old
//unsigned int full_soc = 9680; //new
unsigned int fg_zero_count = 0;
//u8 preData[2]; /* for fg stress test */

int fuel_guage_init = 0;
EXPORT_SYMBOL(fuel_guage_init);

static struct i2c_driver fg_i2c_driver;
static struct i2c_client *fg_i2c_client = NULL;

struct fg_state{
	struct i2c_client	*client;	
};


struct fg_state *fg_state;

//extern unsigned int is_full_charging(void);
//static int is_reset_soc = 0;

static int fg_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg; 

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) 
		return -EIO;

	*data = buf[0];
	
	return 0;
}

static int fg_i2c_write(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[3];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = *data;
	buf[2] = *(data + 1);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) 
		return -EIO;

	return 0;
}

unsigned int fg_read_vcell(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 vcell = 0;

	if(fg_i2c_client==NULL)
		return -1;

	if (fg_i2c_read(client, VCELL0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read VCELL0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, VCELL1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read VCELL1\n", __func__);
		return -1;
	}
	vcell = ((((data[0] << 4) & 0xFF0) | ((data[1] >> 4) & 0xF)) * 125)/100;
	//pr_info("%s: VCELL=%d\n", __func__, vcell);
	return vcell;
}

unsigned int fg_read_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	//unsigned int FGPureSOC = 0;
	unsigned int FGAdjustSOC = 0;
	unsigned int FGSOC = 0;

	if(fg_i2c_client==NULL)
		return -1;

	if (fg_i2c_read(client, SOC0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read SOC0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOC1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read SOC1\n", __func__);
		return -1;
	}
	
	// calculating soc
	FGPureSOC = data[0]*100+((data[1]*100)/256);

	// adjusting soc
#if 0 //adaptive full_soc
	if(is_full_charging())
	{
		full_soc = FGPureSOC;
		if(full_soc < 9000)
			full_soc = 9000;
	}
#endif

#if 0 /* rcomp : C0:9680, B0:9540 */
	//calculating alert level
	// (2 - 1.8)/95.4*100 = 0.21%, so rounded off 0% so, 2% is ok! => apply to sbl source related rcomp

	//if(FGPureSOC >= 180)
	//if(FGPureSOC >= 0)
	if(FGPureSOC >= 90)
	{
		//FGAdjustSOC = ((FGPureSOC - 180)*10000)/9540; //old
		//FGAdjustSOC = ((FGPureSOC - 90)*10000)/9680; //new
		FGAdjustSOC = ((FGPureSOC - 90)*10000)/full_soc; //new and adaptive full_soc
		//FGAdjustSOC = (FGPureSOC*10000)/full_soc; //empty_soc is '0' and adaptive full_soc, old2
		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 40)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* new adjust DF02, rcomp : B0 */
	if(FGPureSOC >= 90)
	{
		// 990 = 900 + 90(empty soc)
		if(FGPureSOC >= 990)
		{
			// -990 = -90(empty soc) + 600 - 1500
			// 8500 = 10000-1500;
			// 8095 = 9085(full soc) -90(empty soc) + 600 - 1500
			FGAdjustSOC = (FGPureSOC - 990)*8500/8095 + 1500;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 90)*1500/900;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 40)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test 2, rcomp : B0 */
	if(FGPureSOC >= 90)
	{
		// 240 = 150 + 90(empty soc)
		if(FGPureSOC >= 240)
		{
			// -240 = -90(empty soc) + 700 - 850
			// 9150 = 10000-850;
			// 8845 = 9085(full soc) -90(empty soc) + 700 - 850
			FGAdjustSOC = (FGPureSOC - 240)*9150/8845 + 850;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 90)*850/150;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 40)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test 3, rcomp : B0 */
	if(FGPureSOC >= 90)
	{
		// 220 = 130 + 90(empty soc)
		if(FGPureSOC >= 220)
		{
			// -220 = -90(empty soc) + 1000 - 1130
			// 8870 = 10000-1130;
			// 8780 = 9000(full soc) -90(empty soc) + 1000 - 1130
			FGAdjustSOC = (FGPureSOC - 220)*8870/8780 + 1130;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 90)*1130/130;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 40)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test 4, comp : B0 */
	if(FGPureSOC >= 90)
	{
		if(FGPureSOC >= 1090)
		{
			// 1090 = 1000 + 90(empty soc)
			// 7500 = 10000 - 2500
			// 7910 = 9000(full soc) -90(empty soc) + 1500 - 2500
			FGAdjustSOC = (FGPureSOC - 1090)*7500/7910 + 2500;
		}
		else if(FGPureSOC >= 220)
		{
			// 220 = 130 + 90(empty soc)
			// -220 = -90(empty soc) + 1000 - 1130
			// 1370 = 2500-1130;
			// 870 = 1090(full soc) -90(empty soc) + 1000 - 1130
			FGAdjustSOC = (FGPureSOC - 220)*1370/870 + 1130;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 90)*1130/130;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 40)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test5, DF05, change the rcomp to C0 */
	//calculating alert level
	// (2 - 1.8)/92.0*100 = 0.22%, so rounded off 0% so, 2% is ok! => apply to sbl source related rcomp
	
	if(FGPureSOC >= 180)
	{
		//FGAdjustSOC = ((FGPureSOC - 180)*10000)/9540; //original
		FGAdjustSOC = ((FGPureSOC - 180)*10000)/9200;
		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		if(FGPureSOC >= 100)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test6, DF05, change the rcomp to C0 */
	if(FGPureSOC >= 160)
	{
		if(FGPureSOC >= 760)
		{
			FGAdjustSOC = (FGPureSOC - 760)*8700/8640 + 1300;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 160)*1300/600;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%	
	}
	else
	{
		if(FGPureSOC >= 100)
			FGAdjustSOC = 100; //1%
		else
			FGAdjustSOC = 0; //0%
	}
#endif
#if 0 /* test7, DF06, change the rcomp to C0 */
	if(FGPureSOC >= 60)
	{
		if(FGPureSOC >= 460)
		{
			FGAdjustSOC = (FGPureSOC - 460)*8650/8740 + 1350;
		}
		else
		{
			FGAdjustSOC = (FGPureSOC - 60)*1350/400;
		}

		if(FGAdjustSOC < 100)
			FGAdjustSOC = 100; //1%
	}
	else
	{
		FGAdjustSOC = 0; //0%
	}
#endif

#if 1 /* test DK24, change the rcomp to FF */
	if(FGPureSOC >= 0)
	{
		FGAdjustSOC = (FGPureSOC*10000)/9300;
	}
	else
	{
		FGAdjustSOC = 0;
	}
#endif

	// rounding off and Changing to percentage.
	FGSOC=FGAdjustSOC/100;

	if(FGAdjustSOC%100 >= 50 )
	{
		FGSOC+=1;
	}

	if(FGSOC>=100)
	{
		FGSOC=100;
	}

	/* we judge real 0% after 3 continuous counting */
	if(FGSOC == 0)
	{
		fg_zero_count++;

		if(fg_zero_count >= 3)
		{
			FGSOC = 0;
			fg_zero_count = 0;
		}
		else
		{
			FGSOC = prevFGSOC;
		}
	}
	else
	{
		fg_zero_count=0;
	}

	if(prevFGSOC != FGSOC) {
		printk("[SOC] C:%d, P:%d\n", prevFGSOC, FGSOC);
	}

	prevFGSOC = FGSOC;

	/* for fg stress test */
	/*
	if((preData[0] != data[0]) || (preData[1] != data[1]))
	{
		printk("soc = %d data[0] = 0x%x, data[1] = 0x%x\n", FGSOC, data[0],data[1]);
	}

	preData[0] = data[0];
	preData[1] = data[1];
	*/

	return FGSOC;
	
}


unsigned int fg_reset_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 rst_cmd[2];
	s32 ret = 0;

	if(fg_i2c_client==NULL)
		return -1;

	//is_reset_soc = 1;
	/* Quick-start */
	rst_cmd[0] = 0x40;
	rst_cmd[1] = 0x00;

	ret = fg_i2c_write(client, MODE0_REG, rst_cmd);
	if (ret)
		pr_info("%s: failed reset SOC(%d)\n", __func__, ret);

	msleep(300);
	//is_reset_soc = 0;
	return ret;
}


void fuel_gauge_rcomp(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 rst_cmd[2];
	s32 ret = 0;
	
	if(fg_i2c_client==NULL)
		return;


#if defined(MAX17043)
	rst_cmd[0] = 0xff;
	//rst_cmd[0] = 0xb0; //new rcomp.
	//rst_cmd[1] = 0x1e; //0x1f : 1% alert, 0x1e : 2% alert
	rst_cmd[1] = 0x1f; //0x1f : 1% alert, 0x1e : 2% alert
#else
	rst_cmd[0] = 0xa0;
	rst_cmd[1] = 0x00;
#endif	

	ret = fg_i2c_write(client, RCOMP0_REG, rst_cmd);
	if (ret)
		pr_info("%s: failed fuel_gauge_rcomp(%d)\n", __func__, ret);
	
	msleep(500);
}

unsigned int fg_read_rcomp(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u16 rcomp = 0;

	if(fg_i2c_client==NULL)
		return -1;

	if (fg_i2c_read(client, RCOMP0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read VCELL0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, RCOMP1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read VCELL1\n", __func__);
		return -1;
	}
	rcomp = (data[0]<<8) | data[1];
	pr_info("%s: read rcomp = 0x%x\n", __func__, rcomp);
	
	return rcomp;
}

static int fg_i2c_remove(struct i2c_client *client)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
	fg_i2c_client = client;
	return 0;
}

static int fg_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct fg_state *fg;

	fg = kzalloc(sizeof(struct fg_state), GFP_KERNEL);
	if (fg == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	fg->client = client;
	i2c_set_clientdata(client, fg);
	
	/* rest of the initialisation goes here. */
	
	printk("Fuel guage attach success!!!\n");

	fg_i2c_client = client;

	fuel_guage_init = 1;

	return 0;
}

static const struct i2c_device_id fg_device_id[] = {
	{"fuelgauge", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fg_device_id);

static struct i2c_driver fg_i2c_driver = {
	.driver = {
		.name = "fuelgauge",
		.owner = THIS_MODULE,
	},
	.probe	= fg_i2c_probe,
	.remove	= fg_i2c_remove,
	.id_table	= fg_device_id,
};
