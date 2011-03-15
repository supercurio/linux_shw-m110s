/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME "jupiter-battery"

#if 0
/*
 * Spica Rev00 board Battery Table
 */
#define BATT_CAL                2447    /* 3.60V */

#define BATT_MAXIMUM            406     /* 4.176V */
#define BATT_FULL               353     /* 4.10V  */
#define BATT_SAFE_RECHARGE  353     /* 4.10V */
#define BATT_ALMOST_FULL        188     /* 3.8641V */       //322   /* 4.066V */
#define BATT_HIGH               112     /* 3.7554V */       //221   /* 3.919V */
#define BATT_MED                66      /* 3.6907V */       //146   /* 3.811V */
#define BATT_LOW                43      /* 3.6566V */       //112   /* 3.763V */
#define BATT_CRITICAL           8       /* 3.6037V */   //(74)  /* 3.707V */
#define BATT_MINIMUM            (-28)   /* 3.554V */        //(38)  /* 3.655V */
#define BATT_OFF                (-128)  /* 3.4029V */       //(-103)    /* 3.45V  */
#endif

/*
 * AriesQ Rev00 board Temperature Table
 */
#if 0 /* eur-feature */
const int temper_table[][2] =  {
    /* ADC, Temperature (C) */
    { 1667,     -70 },
    { 1658,     -60 },
    { 1632,     -50 },
    { 1619,     -40 },
    { 1614,     -30 },
    { 1596,     -20 },
    { 1577,     -10 },
    { 1559,     0   },
    { 1536,     10  },
    { 1513,     20  },
    { 1491,     30  },
    { 1468,     40  },
    { 1445,     50  },
    { 1421,     60  },
    { 1396,     70  },
    { 1372,     80  },
    { 1348,     90  },
    { 1324,     100 },
    { 1299,     110 },
    { 1275,     120 },
    { 1251,     130 },
    { 1226,     140 },
    { 1202,     150 },
    { 1178,     160 },
    { 1155,     170 },
    { 1131,     180 },
    { 1108,     190 },
    { 1084,     200 },
    { 1060,     210 },
    { 1037,     220 },
    { 1013,     230 },
    { 990,      240 },
    { 966,      250 },
    { 943,      260 },
    { 920,      270 },
    { 898,      280 },
    { 875,      290 },
    { 852,      300 },
    { 829,      310 },
    { 806,      320 },
    { 784,      330 },
    { 761,      340 },
    { 738,      350 },
    { 718,      360 },
    { 697,      370 },
    { 677,      380 },
    { 656,      390 },
    { 636,      400 },
    { 615,      410 },
    { 595,      420 },
    { 574,      430 },
    { 554,      440 },
    { 533,      450 },
    { 518,      460 },
    { 503,      470 },
    { 487,      480 },
    { 472,      490 },
    { 457,      500 },
    { 442,      510 },
    { 427,      520 },
    { 411,      530 },
    { 396,      540 },
    { 381,      550 },
    { 368,      560 },
    { 354,      570 },
    { 341,      580 },
    { 324,      590 },
    { 306,      600 },
    { 299,      610 },
    { 293,      620 },
    { 286,      630 },
    { 275,      640 },
    { 264,      650 },
};
#endif /* eur-feature */

#if 0 /* eur-feature */
#define TEMP_IDX_ZERO_CELSIUS   7
#define TEMP_HIGH_BLOCK     temper_table[TEMP_IDX_ZERO_CELSIUS+63][0]
#define TEMP_HIGH_RECOVER       temper_table[TEMP_IDX_ZERO_CELSIUS+58][0]
#define TEMP_LOW_BLOCK          temper_table[TEMP_IDX_ZERO_CELSIUS-4][0]
#define TEMP_LOW_RECOVER        temper_table[TEMP_IDX_ZERO_CELSIUS+1][0]
#endif /* eur-feature */

/* change the Temp. value for user temperature */
const int temper_table[][2] =  {
    /* ADC, Temperature (C) */
    { 1667,     -140 }, //0
    { 1658,     -130 },
    { 1632,     -120 }, //2
    { 1619,     -110 },
    { 1614,     -100 }, //4
    { 1596,     -90  }, //5
    { 1577,     -80  }, //6
    { 1559,     -70  }, //7
    { 1545,     -60  }, //8
    { 1513,     -50  }, //9
    { 1491,     -40  }, //10
    { 1468,     -30  }, //11
    { 1455,     -20  }, //12
    { 1421,     -10  },
    { 1396,     0    }, //14
    { 1372,     10   },
    { 1348,     20   },
    { 1324,     30   },
    { 1299,     40   },
    { 1275,     50   }, //19
    { 1251,     60   },
    { 1226,     70   },
    { 1202,     80   },
    { 1178,     90   },
    { 1155,     100  },
    { 1131,     110  },
    { 1108,     120  },
    { 1084,     130  },
    { 1060,     140  },
    { 1037,     150  },
    { 1013,     160  },
    { 990,      170  },
    { 966,      180  },
    { 943,      190  },
    { 920,      200  },
    { 898,      210  },
    { 875,      220  },
    { 852,      230  },
    { 829,      240  },
    { 806,      250  },
    { 784,      260  },
    { 761,      270  },
    { 738,      270  },
    { 718,      280  }, //43
    { 697,      290  }, //44
    { 677,      300  },
    { 656,      300  },
    { 636,      310  }, //47 
    { 615,      320  },
    { 595,      330  },
    { 574,      330  },
    { 554,      340  }, //51
    { 533,      350  }, //52
    { 515,      360  }, //53
    { 503,      370  }, //54
    { 487,      370  }, //55
    { 472,      380  },
    { 457,      390  }, //57
    { 442,      390  }, //58
    { 427,      400  }, //59
    { 411,      410  }, //60
    { 396,      420  }, //61
    { 381,      420  }, //62
    { 368,      430  }, //63
    { 358,      440  }, //64
    { 345,      450  }, //65
    { 324,      450  },
    { 306,      460  },
    { 299,      470  }, //68
    { 293,      480  }, //69
    { 286,      480  }, //70
    { 275,      490  },
    { 264,      500  },
};

#ifdef BATT_LATONA_FEATURE
#define TEMP_HIGH_BLOCK     	temper_table[65][0] //45'c adc + 40'c delta T adc + alpha_2
#define TEMP_HIGH_RECOVER   	temper_table[53][0] //average 40'c adc and 45'c adc
#define TEMP_LOW_BLOCK      	temper_table[8][0] //-5'c adc
#define TEMP_LOW_RECOVER    	temper_table[12][0] //0'c adc
#define TEMP_EVENT_HIGH_BLOCK	temper_table[72][0] //65'c
#else /* M110S */
#define TEMP_HIGH_BLOCK     temper_table[69][0] //45'c adc + 40'c delta T adc + alpha_2
#define TEMP_HIGH_RECOVER   temper_table[54][0] //average 40'c adc and 45'c adc
#define TEMP_LOW_BLOCK      temper_table[7][0] //-5'c adc
#define TEMP_LOW_RECOVER    temper_table[11][0] //0'c adc
#endif /* BATT_LATONA_FEATURE */

/*
 * AriesQ Rev00 board ADC channel
 */
typedef enum s3c_adc_channel {
    S3C_ADC_BATT_MON = 1,
    S3C_ADC_CHG_CURRENT = 2,
    S3C_ADC_EAR = 3,
    S3C_ADC_TEMPERATURE = 6,
    S3C_ADC_VOLTAGE,
    S3C_ADC_V_F,
    ENDOFADC
} adc_channel_type;

//#define IRQ_TA_CONNECTED_N    IRQ_EINT(19)
//#define IRQ_TA_CHG_N          IRQ_EINT(25)

//#define IRQ_FUEL_INT_N        IRQ_EINT(8) /* It doesn't work */
#define IRQ_FUEL_INT_N      IRQ_EINT8


/******************************************************************************
 * Battery driver features
 * ***************************************************************************/
/* #define __TEMP_ADC_VALUE__ */
/* #define __USE_EGPIO__ */
#define __CHECK_BATTERY_V_F__
/* #define __BATTERY_COMPENSATION__ */ /* eur-feature */
/* #define __CHECK_BOARD_REV__ */
/* #define __BOARD_REV_ADC__ */
/* #define __TEST_DEVICE_DRIVER__ */ /* eur-feature */
/* #define __ALWAYS_AWAKE_DEVICE__  */
/* #define __TEST_MODE_INTERFACE__ */ /* eur-feature */ 
#define __TEMP_BLOCK_ECXEPT__
#define __CHECK_CHG_CURRENT__
#define __CHECK_BATT_VOLTAGE__
#define __POPUP_DISABLE_MODE__
#define __SET_TEST_VALUE__
//#define __PSEUDO_BOOT_COMPLETED__
/*****************************************************************************/

//#define TOTAL_CHARGING_TIME   (6*60*60*1000)  /* 6 hours */
//#define TOTAL_RECHARGING_TIME (1*60*60*1000+30*60*1000)   /* 1.5 hours */
#define TOTAL_CHARGING_TIME (5*60*60*1000)  /* 5 hours */
#define TOTAL_RECHARGING_TIME   (2*60*60*1000)  /* 2 hours */

#ifdef __TEMP_BLOCK_ECXEPT__
#define CALL_TEMP_EXCEPT_BIT        0
#define DMB_TEMP_EXCEPT_BIT         1
#define MUSIC_TEMP_EXCEPT_BIT       2
#define VIDEO_TEMP_EXCEPT_BIT       3
#define CAMERA_TEMP_EXCEPT_BIT      4
#define INTERNEL_TEMP_EXCEPT_BIT    5
#endif /* __TEMP_BLOCK_ECXEPT__ */

#ifdef __BATTERY_COMPENSATION__
#define COMPENSATE_VIBRATOR     19
#define COMPENSATE_CAMERA           25
#define COMPENSATE_MP3              17
#define COMPENSATE_VIDEO            28
#define COMPENSATE_VOICE_CALL_2G    13
#define COMPENSATE_VOICE_CALL_3G    14
#define COMPENSATE_DATA_CALL        25
#define COMPENSATE_LCD              0
#define COMPENSATE_TA               0
#define COMPENSATE_CAM_FALSH        0
#define COMPENSATE_BOOTING      52
#endif /* __BATTERY_COMPENSATION__ */

//#define SOC_LB_FOR_POWER_OFF      27

#define RECHARGE_COND_TIME      (20*1000)   /* 20 seconds */
#define FULL_CHARGE_COND_VOLTAGE    4000
#define RECHARGE_COND_VOLTAGE       4110
//#define LPM_RECHARGE_COND_VOLTAGE 4110
#define CURRENT_OF_FULL_CHG         316     /* 170mA => (code*1.5)mV , refer to code table. */ 
//#define LPM_CURRENT_OF_FULL_CHG   316     /* 170mA => (code*1.5)mV , refer to code table. */

#define ADC_12BIT_RESOLUTION        8056    /* 3300mV/4096 = 0.805664063, * scale factor */
#define ADC_12BIT_SCALE             10000   /* scale factor */
#define ADC_CURRENT_FACTOR          15 /* 1mA = 1.5mV */

#define BAT_DETECTED        1
#define BAT_NOT_DETECTED    0


#ifdef LPM_MODE
void charging_mode_set(unsigned int val);
unsigned int charging_mode_get(void);
#endif
unsigned int get_battery_level(void);
unsigned int is_charging_enabled(void);
#ifdef __TEMP_BLOCK_ECXEPT__
static void batt_set_temper_exception(int bit);
static void batt_clear_temper_exception(int bit);
#endif /* __TEMP_BLOCK_ECXEPT__ */
static int s3c_bat_get_property(struct power_supply *bat_ps, enum power_supply_property psp, union power_supply_propval *val);
static int s3c_power_get_property(struct power_supply *bat_ps,  enum power_supply_property psp, union power_supply_propval *val);
static ssize_t s3c_bat_show_property(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef __BATTERY_COMPENSATION__
static void s3c_bat_set_compesation(int mode, int offset, int compensate_value);
#endif /* __BATTERY_COMPENSATION__ */
//static void s3c_bat_set_vol_cal(int batt_cal);
static int s3c_bat_get_charging_status(void);
static ssize_t s3c_bat_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#ifdef __BATTERY_COMPENSATION__
void s3c_bat_set_compensation_for_drv(int mode, int offset);
#endif /* __BATTERY_COMPENSATION__ */
//void low_battery_power_off(void); /* eur-feature */
static int get_usb_power_state(void);
static int s3c_bat_get_adc_data(adc_channel_type adc_ch);
//static unsigned long calculate_average_adc(adc_channel_type channel, int adc);
static unsigned long s3c_read_temp(void);
static int is_over_abs_time(void);
#ifdef __CHECK_CHG_CURRENT__
//static void check_chg_current(struct power_supply *bat_ps);
static void check_chg_current(void);
static void s3c_check_chg_current(void);
#endif /* __CHECK_CHG_CURRENT__ */
static u32 s3c_get_bat_health(void);
static void s3c_set_bat_health(u32 batt_health);
static void s3c_set_time_for_charging(int mode);
static void s3c_set_chg_en(int enable);
static void s3c_temp_control(int mode);
static void s3c_cable_check_status(void);
void s3c_cable_changed(void);
//void s3c_cable_charging(void);
static int s3c_cable_status_update(void);
static int s3c_get_bat_temp(void);
#ifdef __CHECK_BATT_VOLTAGE__
static void check_batt_voltage(void);
#endif /* __CHECK_BATT_VOLTAGE__ */
static int s3c_get_bat_vol(void);
static int s3c_get_bat_level(void);
static int s3c_bat_need_recharging(void);
static int s3c_bat_is_full_charged(void);
static void s3c_bat_charging_control(void);
static int batt_read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static void s3c_bat_status_update(void);
#ifdef __CHECK_BATTERY_V_F__
//static unsigned int s3c_bat_check_v_f(void);
static void s3c_bat_check_v_f(void);
#endif /* __CHECK_BATTERY_V_F__ */
static void polling_timer_func(unsigned long unused);
static void s3c_store_bat_old_data(void);
static void s3c_bat_work(struct work_struct *work);
static int s3c_bat_create_attrs(struct device * dev);
static void battery_early_suspend(struct early_suspend *h);
static void battery_late_resume(struct early_suspend *h);
#ifdef MAX17043
static irqreturn_t low_battery_isr( int irq, void *_di );
int _low_battery_alarm_(void);
static void fuelgauge_work_handler( struct work_struct *work );
#endif /* MAX17043 */
#ifdef __PSEUDO_BOOT_COMPLETED__
static void boot_complete_work_handler( struct work_struct *work )
#endif /* __PSEUDO_BOOT_COMPLETED__ */
static int __devinit s3c_bat_probe(struct platform_device *pdev);
#ifdef CONFIG_PM
static int s3c_bat_suspend(struct platform_device *pdev, pm_message_t state);
static int s3c_bat_resume(struct platform_device *pdev);
#endif /* CONFIG_PM */

static int __devexit s3c_bat_remove(struct platform_device *pdev);
static int __init s3c_bat_init(void);
static void __exit s3c_bat_exit(void);

