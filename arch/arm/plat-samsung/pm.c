/* linux/arch/arm/plat-s3c/pm.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2004-2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C common power management (suspend to ram) support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/regs-serial.h>
#include <mach/regs-power.h>
#include <mach/regs-clock.h>
#include <mach/regs-irq.h>
#include <asm/irq.h>

#include <plat/pm.h>
#include <mach/pm-core.h>
#include <mach/regs-gpio.h>
#include <mach/gpio-bank.h>
#include <mach/sec_jack.h>

#include <plat/gpio-cfg.h>

#if defined(CONFIG_MACH_S5PC110_P1)
int g_pm_wakeup_stat=PM_WAKEUP_NONE;
#endif // CONFIG_MACH_S5PC110_P1

//#define PM_DEBUG

#undef SLEEP_TEST

#ifndef FEATURE_FTM_SLEEP
#define FEATURE_FTM_SLEEP
#undef FTM_SLEEP_TEST
#endif

#ifdef FEATURE_FTM_SLEEP
#define SMD_FTM_SLEEP_TIMEOUT	20
#define CAL_FTM_SLEEP_TIMEOUT	4
unsigned int is_cal_ftm_sleep = 0;
extern unsigned char ftm_sleep;
unsigned char ftm_sleep_exit=0;
EXPORT_SYMBOL(ftm_sleep_exit);
#ifdef FTM_SLEEP_TEST
unsigned int ftm_sleep_test=1;
#endif
#endif

#define TSADC_ENABLE	0x1

extern int isVoiceCall;

#define USE_DMA_ALLOC

#ifdef USE_DMA_ALLOC
#include <linux/dma-mapping.h>

static unsigned long *regs_save;
static dma_addr_t phy_regs_save;
#endif /* USE_DMA_ALLOC */

void __iomem *rtc_pa_base;
unsigned long s5pc1xx_wakeup_src=0;
unsigned int s5pc11x_pm_wakeup_eint0_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint1_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint2_pend = 0;
unsigned int s5pc11x_pm_wakeup_eint3_pend = 0;
unsigned int s5pc11x_pm_wakeup_wakeup_stat = 0;
static unsigned int  pm_use_rtc_tick = 0;
unsigned int pm_request_rtc_tick = 0; // for driver use

/* for external use */

unsigned long s3c_pm_flags;
extern void s3c_config_sleep_gpio(void);
extern unsigned int HWREV;

//#define DBG(fmt...) printk(fmt)
#define DBG(fmt...)

/* Debug code:
 *
 * This code supports debug output to the low level UARTs for use on
 * resume before the console layer is available.
*/

#ifdef CONFIG_SAMSUNG_PM_DEBUG
extern void printascii(const char *);

#if 0
void s3c_pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}
#endif

static inline void s3c_pm_debug_init(void)
{
	/* restart uart clocks so we can use them to output */
	s3c_pm_debug_init_uart();
}

#else
#define s3c_pm_debug_init() do { } while(0)

#endif /* CONFIG_SAMSUNG_PM_DEBUG */

/* Save the UART configurations if we are configured for debug. */

unsigned char pm_uart_udivslot;

#define ENABLE_UART_SAVE_RESTORE
#ifdef ENABLE_UART_SAVE_RESTORE//CONFIG_SAMSUNG_PM_DEBUG

struct pm_uart_save uart_save[CONFIG_SERIAL_SAMSUNG_UARTS];

static void s3c_pm_save_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	save->ulcon = __raw_readl(regs + S3C2410_ULCON);
	save->ucon = __raw_readl(regs + S3C2410_UCON);
	save->ufcon = __raw_readl(regs + S3C2410_UFCON);
	save->umcon = __raw_readl(regs + S3C2410_UMCON);
	save->ubrdiv = __raw_readl(regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		save->udivslot = __raw_readl(regs + S3C2443_DIVSLOT);

	//S3C_PMDBG("UART[%d]: ULCON=%04x, UCON=%04x, UFCON=%04x, UBRDIV=%04x\n",
		  //uart, save->ulcon, save->ucon, save->ufcon, save->ubrdiv);
}

static void s3c_pm_save_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_save_uart(uart, save);
}

static void s3c_pm_restore_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	s3c_pm_arch_update_uart(regs, save);

	__raw_writel(save->ulcon, regs + S3C2410_ULCON);
	__raw_writel(save->ucon,  regs + S3C2410_UCON);
	__raw_writel(save->ufcon, regs + S3C2410_UFCON);
	__raw_writel(save->umcon, regs + S3C2410_UMCON);
	__raw_writel(save->ubrdiv, regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		__raw_writel(save->udivslot, regs + S3C2443_DIVSLOT);
}

static void s3c_pm_restore_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_restore_uart(uart, save);
}
#else
static void s3c_pm_save_uarts(void) { }
static void s3c_pm_restore_uarts(void) { }
#endif

/* The IRQ ext-int code goes here, it is too small to currently bother
 * with its own file. */

unsigned long s3c_irqwake_intmask	= 0xffffffffL;
unsigned long s3c_irqwake_eintmask	= 0xffffffffL;

int s3c_irqext_wake(unsigned int irqno, unsigned int state)
{
	unsigned long bit = 1L << IRQ_EINT_BIT(irqno);

	if (!(s3c_irqwake_eintallow & bit))
		return -ENOENT;

	printk(KERN_INFO "wake %s for irq %d\n",
	       state ? "enabled" : "disabled", irqno);

	if (!state)
		s3c_irqwake_eintmask |= bit;
	else
		s3c_irqwake_eintmask &= ~bit;

	return 0;
}
/////////////////////////////////

#if 1

#define eint_offset(irq)                (irq)
#define eint_irq_to_bit(irq)            (1 << (eint_offset(irq) & 0x7))
#define eint_conf_reg(irq)              ((eint_offset(irq)) >> 3)
#define eint_filt_reg(irq)              ((eint_offset(irq)) >> 2)
#define eint_mask_reg(irq)              ((eint_offset(irq)) >> 3)
#define eint_pend_reg(irq)              ((eint_offset(irq)) >> 3)

// intr_mode 0x2=>falling edge, 0x3=>rising dege, 0x4=>Both edge
static void s3c_pm_set_eint(unsigned int irq, unsigned int intr_mode)
{
	int offs = (irq);
	int shift;
	u32 ctrl, mask, tmp;
	//u32 newvalue = 0x2; // Falling edge

	shift = (offs & 0x7) * 4;
	if((0 <= offs) && (offs < 8)){
		tmp = readl(S5PV210_GPH0CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH0CON);
		/*pull up disable*/
	}
	else if((8 <= offs) && (offs < 16)){
		tmp = readl(S5PV210_GPH1CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH1CON);
	}
	else if((16 <= offs) && (offs < 24)){
		tmp = readl(S5PV210_GPH2CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH2CON);
	}
	else if((24 <= offs) && (offs < 32)){
		tmp = readl(S5PV210_GPH3CON);
		tmp |= (0xf << shift);
		writel(tmp , S5PV210_GPH3CON);
	}
	else{
		printk(KERN_ERR "No such irq number %d", offs);
		return;
	}

	/*special handling for keypad eint*/
	if( (24 <= irq) && (irq <= 27))
	{// disable the pull up
		tmp = readl(S5PV210_GPH3PUD);
		tmp &= ~(0x3 << ((offs & 0x7) * 2));	
		writel(tmp, S5PV210_GPH3PUD);
		DBG("S5PV210_GPH3PUD = %x\n",readl(S5PV210_GPH3PUD));
	}
	

	/*Set irq type*/
	mask = 0x7 << shift;
	ctrl = readl(S5PV210_EINTCON(eint_conf_reg(irq)));
	ctrl &= ~mask;
	//ctrl |= newvalue << shift;
	ctrl |= intr_mode << shift;

	writel(ctrl, S5PV210_EINTCON(eint_conf_reg(irq)));
	/*clear mask*/
	mask = readl(S5PV210_EINTMASK(eint_mask_reg(irq)));
	mask &= ~(eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTMASK(eint_mask_reg(irq)));

	/*clear pending*/
	mask = readl(S5PV210_EINTPEND(eint_pend_reg(irq)));
	mask &= (eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTPEND(eint_pend_reg(irq)));
	
	/*Enable wake up mask*/
	tmp = readl(S5P_EINT_WAKEUP_MASK);
	tmp &= ~(1 << (irq));
	writel(tmp , S5P_EINT_WAKEUP_MASK);

	DBG("S5PV210_EINTCON = %x\n",readl(S5PV210_EINTCON(eint_conf_reg(irq))));
	DBG("S5PV210_EINTMASK = %x\n",readl(S5PV210_EINTMASK(eint_mask_reg(irq))));
	DBG("S5PV210_EINTPEND = %x\n",readl(S5PV210_EINTPEND(eint_pend_reg(irq))));
	
	return;
}

static void s3c_pm_clear_eint(unsigned int irq)
{
	u32 mask;
	/*clear pending*/
	mask = readl(S5PV210_EINTPEND(eint_pend_reg(irq)));
	mask &= (eint_irq_to_bit(irq));
	writel(mask, S5PV210_EINTPEND(eint_pend_reg(irq)));
}
#endif








//////////////////////////////////
void s5pc11x_pm_enable_rtctic(unsigned int sec) 
{
	unsigned int tmp;

	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	//printk(KERN_DEBUG "rtccon = 0x%x\n", tmp);

	//RTCEN
	tmp |= 0x1<<0;
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON

	//Tick Time disable
	tmp &= ~(0x1<<8); //set RTCCON[0] as 1 for start, 
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	//printk(KERN_DEBUG "rtccon set | 0x0<<8 = 0x%x\n", tmp);

	//TICCKSEL
	tmp &= ~(0xF<<4); // set TICCKSEL as 0000 for 32768Hz
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	//printk(KERN_DEBUG "rtccon set | 0x0<<4 = 0x%x\n", tmp);

	//TICNT
	tmp = (sec * 32768) - 1 ; // set new time tick count value
	__raw_writel(tmp, rtc_pa_base+0x44);
	tmp = __raw_readl(rtc_pa_base+0x44); //TICNT
	//printk(KERN_DEBUG "ticnt set (%dsec * 32768) - 1 = 0x%x\n", sec, tmp);

	//TICEN
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	tmp |= (0x1<<8); //Enables Tick Timer
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40);
	//printk(KERN_DEBUG "rtccon set | (0x1<<8), tick enable = 0x%x\n", tmp);

	//RTCEN clear
	tmp &= ~(0x1<<0);
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	//printk(KERN_DEBUG "rtccon set & ~(0x1<<0), RTCEN clear = 0x%x\n", tmp);

	tmp = __raw_readl(rtc_pa_base+0x30); //INTP, intterrupt pending clear
	tmp |= 0x1<<0; // clear Time TIC bits
	__raw_writel(tmp, rtc_pa_base+0x30);
	tmp = __raw_readl(rtc_pa_base+0x30); //INTP, intterrupt pending clear
	//printk(KERN_DEBUG "intp time tic pending clear? = 0x%x\n", tmp);
}

void s5pc11x_pm_disable_rtctic(void)
{
	unsigned int tmp;

	tmp = __raw_readl(rtc_pa_base+0x40);
	//printk(KERN_DEBUG "rtccon = 0x%x\n", tmp);

	//Tick Time disable
	tmp &= ~(0x1<<8); //set RTCCON[0] as 1 for start, 
	__raw_writel(tmp, rtc_pa_base+0x40);
	tmp = __raw_readl(rtc_pa_base+0x40); //RTCCON
	//printk(KERN_DEBUG "rtccon set | 0x0<<8 = 0x%x\n", tmp);

	tmp = __raw_readl(rtc_pa_base+0x30);
	//printk(KERN_DEBUG "intp = 0x%x\n", tmp);

	tmp |= 0x1<<0;
	__raw_writel(tmp, rtc_pa_base+0x30);
	tmp = __raw_readl(rtc_pa_base+0x30);
	//printk(KERN_DEBUG "intp time tic pending clear? = 0x%x\n", tmp);
}

void s5pc11x_pm_check_auto_wakeup(void)
{
	//printk(KERN_DEBUG "s5pc11x_pm_check_auto_wakeup...\n");
#ifdef FTM_SLEEP_TEST
	if(ftm_sleep_test == 1)
	{
		printk(KERN_DEBUG "ftm sleep test, enable auto wakeup...\n");
		ftm_sleep = 1;
		is_cal_ftm_sleep = 1;
	}
#endif
	
	if(ftm_sleep == 1)
	{
		pm_use_rtc_tick = 1;
		s3c_irqwake_intmask = 0xFFF9; // rtc_alarm,rtc_tick

		if(is_cal_ftm_sleep == 1)
		{
			s5pc11x_pm_enable_rtctic(CAL_FTM_SLEEP_TIMEOUT);
			printk(KERN_DEBUG "CAL FTM Sleep Mode...\n\n");
		}
		else
		{
			s5pc11x_pm_enable_rtctic(SMD_FTM_SLEEP_TIMEOUT);
			printk(KERN_DEBUG "SMD FTM Sleep Mode...\n\n");
		}
	}else if(pm_request_rtc_tick == 1)
	{
		pm_use_rtc_tick = 1;
		s5pc11x_pm_enable_rtctic(30);
		printk(KERN_DEBUG "s5pc11x_pm_enable_rtctic...\n\n");
	}

#ifdef SLEEP_TEST
	pm_use_rtc_tick = 1;
	s3c_irqwake_intmask = 0xFFF9; // rtc_alarm,rtc_tick
	s5pc11x_pm_enable_rtctic(3);
#endif
}

void s5pc11x_pm_check_wakeupsrc(void)
{
#ifdef PM_DEBUG
	printk("wakeup Stat : %x \n",s5pc11x_pm_wakeup_wakeup_stat);
	printk("eint0 pend : %x \n",s5pc11x_pm_wakeup_eint0_pend);
	printk("eint1 pend : %x \n",s5pc11x_pm_wakeup_eint1_pend);
	printk("eint2 pend : %x \n",s5pc11x_pm_wakeup_eint2_pend);
	printk("eint3 pend : %x \n",s5pc11x_pm_wakeup_eint3_pend);
#endif

	if(s5pc11x_pm_wakeup_wakeup_stat & WAKEUP_BIT_EINT)
	{

		if(s5pc11x_pm_wakeup_eint0_pend)
		{
			if(s5pc11x_pm_wakeup_eint0_pend & EINTPEND0_BIT_PROX)
			{
				printk(KERN_DEBUG "ws prox \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_PROX;
			}

			if(s5pc11x_pm_wakeup_eint0_pend & EINTPEND0_BIT_EARJACK)
			{
				printk(KERN_DEBUG "ws earjack \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_EARJACK;
			}

			if(s5pc11x_pm_wakeup_eint0_pend & EINTPEND0_BIT_PMIC)
			{
				printk(KERN_DEBUG "ws pmic \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_PMIC;
			}
		}

		if(s5pc11x_pm_wakeup_eint1_pend)
		{

			if(s5pc11x_pm_wakeup_eint1_pend & EINTPEND1_BIT_FUELGAUGE)
			{
				printk(KERN_DEBUG "ws fg \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_FUELGAUGE;
			}

			if(s5pc11x_pm_wakeup_eint1_pend & EINTPEND1_BIT_ONEDRAM)
			{
				printk(KERN_DEBUG "ws cp \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_ONEDRAM;
			}
		}

		if(s5pc11x_pm_wakeup_eint2_pend)
		{
			if(s5pc11x_pm_wakeup_eint2_pend & EINTPEND2_BIT_WLAN)
			{
				printk(KERN_DEBUG "ws wlan \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_WLAN;
			}

			if(s5pc11x_pm_wakeup_eint2_pend & EINTPEND2_BIT_BT)
			{
				printk(KERN_DEBUG "ws bt \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_BT;
			}

			if(s5pc11x_pm_wakeup_eint2_pend & EINTPEND2_BIT_PWRBUTTON)
			{
				printk(KERN_DEBUG "ws pbt \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_PWRBUTTON;
			}

			if(s5pc11x_pm_wakeup_eint2_pend & EINTPEND2_BIT_MICROUSB)
			{
				printk(KERN_DEBUG "ws musb \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_MICROUSB;
			}
		}

		if(s5pc11x_pm_wakeup_eint3_pend)
		{
			if(s5pc11x_pm_wakeup_eint3_pend & EINTPEND3_BIT_OK)
			{
				printk(KERN_DEBUG "ws ok \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_HOME;
			}

			if(s5pc11x_pm_wakeup_eint3_pend & EINTPEND3_BIT_VOLDN)
			{
				printk(KERN_DEBUG "ws vdn \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_VOLDN;
			}

			if(s5pc11x_pm_wakeup_eint3_pend & EINTPEND3_BIT_VOLUP)
			{
				printk(KERN_DEBUG "ws vup \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_VOLUP;
			}

			if(s5pc11x_pm_wakeup_eint3_pend & EINTPEND3_BIT_SDDET)
			{
				printk(KERN_DEBUG "ws sddet \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_SDDET;
			}

			if(s5pc11x_pm_wakeup_eint3_pend & EINTPEND3_BIT_EARSENDEND)
			{
				printk(KERN_DEBUG "ws sendend \n");
				s5pc1xx_wakeup_src |= WAKEUP_SRC_EARSENDEND;
			}
		}
	}

	if(s5pc11x_pm_wakeup_wakeup_stat & WAKEUP_BIT_RTC_ALARM)
	{
		printk(KERN_DEBUG "ws alarm \n");
		s5pc1xx_wakeup_src |= WAKEUP_SRC_RTC_ALARM;
	}


	if(s5pc11x_pm_wakeup_wakeup_stat & WAKEUP_BIT_RTC_TICK)
	{
		printk(KERN_DEBUG "ws tick \n");
		s5pc1xx_wakeup_src |= WAKEUP_SRC_RTC_TICK;
	}		
}

unsigned long s5pc11x_pm_get_wakeup_src(void)
{
	return s5pc1xx_wakeup_src;
}

void s5pc11x_pm_clear_wakeup_src(void)
{
	s5pc1xx_wakeup_src=0;
}

void s5pc11x_pm_do_dump(void)
{
	printk("Clock Reg \n");
	printk("APLL_LOCK : %x \n",readl(S5P_APLL_LOCK));
	printk("MPLL_LOCK : %x \n",readl(S5P_MPLL_LOCK));
	printk("EPLL_LOCK : %x \n",readl(S5P_EPLL_LOCK));
	printk("VPLL_LOCK : %x \n",readl(S5P_VPLL_LOCK));

	printk("APLL_CON : %x \n",readl(S5P_APLL_CON));
	printk("MPLL_CON : %x \n",readl(S5P_MPLL_CON));
	printk("EPLL_CON : %x \n",readl(S5P_EPLL_CON));
	printk("VPLL_CON : %x \n",readl(S5P_VPLL_CON));

	printk("CLK_SRC0 : %x \n",readl(S5P_CLK_SRC0));
	printk("CLK_SRC1 : %x \n",readl(S5P_CLK_SRC1));
	printk("CLK_SRC2 : %x \n",readl(S5P_CLK_SRC2));
	printk("CLK_SRC3 : %x \n",readl(S5P_CLK_SRC3));
	printk("CLK_SRC4 : %x \n",readl(S5P_CLK_SRC4));
	printk("CLK_SRC5 : %x \n",readl(S5P_CLK_SRC5));
	printk("CLK_SRC6 : %x \n",readl(S5P_CLK_SRC6));

	printk("CLK_SRC_MASK0 : %x \n",readl(S5P_CLK_SRC_MASK0));
	printk("CLK_SRC_MASK1 : %x \n",readl(S5P_CLK_SRC_MASK1));

	printk("CLK_DIV0 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV1 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV2 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV3 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV4 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV5 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV6 : %x \n",readl(S5P_CLK_DIV0));
	printk("CLK_DIV7 : %x \n",readl(S5P_CLK_DIV0));

	printk("CLKGATE_MAIN0 : %x \n",readl(S5P_CLKGATE_MAIN0));
	printk("CLKGATE_MAIN1 : %x \n",readl(S5P_CLKGATE_MAIN1));
	printk("CLKGATE_MAIN2 : %x \n",readl(S5P_CLKGATE_MAIN2));

	printk("CLKGATE_PERI0 : %x \n",readl(S5P_CLKGATE_PERI0));
	printk("CLKGATE_PERI1 : %x \n",readl(S5P_CLKGATE_PERI1));

	printk("CLKGATE_SCLK0 : %x \n",readl(S5P_CLKGATE_SCLK0));
	printk("CLKGATE_SCLK1 : %x \n",readl(S5P_CLKGATE_SCLK1));

	printk("CLKGATE_IP0 : %x \n",readl(S5P_CLKGATE_IP0));
	printk("CLKGATE_IP1 : %x \n",readl(S5P_CLKGATE_IP1));
	printk("CLKGATE_IP2 : %x \n",readl(S5P_CLKGATE_IP2));
	printk("CLKGATE_IP3 : %x \n",readl(S5P_CLKGATE_IP3));
	printk("CLKGATE_IP4 : %x \n",readl(S5P_CLKGATE_IP4));

	printk("CLKGATE_BLOCK : %x \n",readl(S5P_CLKGATE_BLOCK));
#if defined(CONFIG_CPU_S5PV210_EVT1)
	printk("CLKGATE_IP5 : %x \n",readl(S5P_CLKGATE_IP5));
#else
	printk("CLKGATE_BUS0 : %x \n",readl(S5P_CLKGATE_BUS0));
	printk("CLKGATE_BUS1 : %x \n",readl(S5P_CLKGATE_BUS1));
#endif


	printk("CLK_OUT : %x \n",readl(S5P_CLK_OUT));
	printk("CLK_DIV_STAT0 : %x \n",readl(S5P_CLK_DIV_STAT0));
	printk("CLK_DIV_STAT1 : %x \n",readl(S5P_CLK_DIV_STAT1));
	printk("CLK_MUX_STAT0 : %x \n",readl(S5P_CLK_MUX_STAT0));
	printk("CLK_MUX_STAT1 : %x \n",readl(S5P_CLK_MUX_STAT1));

	printk("MIXER_OUT_SEL : %x \n",readl(S5P_MIXER_OUT_SEL));
	printk("MDNIE_SEL : %x \n",readl(S5P_MDNIE_SEL));

	printk("POWER Reg \n");

	printk("OSC_CON : %x \n",readl(S5P_OSC_CON));
	printk("RST_STAT : %x \n",readl(S5P_RST_STAT));
	printk("PWR_CFG : %x \n",readl(S5P_PWR_CFG));
	printk("EINT_WAKEUP_MASK : %x \n",readl(S5P_EINT_WAKEUP_MASK));
	printk("WAKEUP_MASK : %x \n",readl(S5P_WAKEUP_MASK));
	printk("PWR_MODE : %x \n",readl(S5P_PWR_MODE));
	printk("NORMAL_CFG : %x \n",readl(S5P_NORMAL_CFG));
	printk("IDLE_CFG : %x \n",readl(S5P_IDLE_CFG));
	printk("STOP_CFG : %x \n",readl(S5P_STOP_CFG));
	printk("STOP_MEM_CFG : %x \n",readl(S5P_STOP_MEM_CFG));
	printk("SLEEP_CFG : %x \n",readl(S5P_SLEEP_CFG));
	printk("OSC_FREQ : %x \n",readl(S5P_OSC_FREQ));
	printk("OSC_STABLE : %x \n",readl(S5P_OSC_STABLE));
	printk("PWR_STABLE : %x \n",readl(S5P_PWR_STABLE));
	printk("MTC_STABLE : %x \n",readl(S5P_MTC_STABLE));
	printk("CLAMP_STABLE : %x \n",readl(S5P_CLAMP_STABLE));
	printk("WAKEUP_STAT : %x \n",readl(S5P_WAKEUP_STAT));
	printk("BLK PWR STAT : %x \n",readl(S5P_BLK_PWR_STAT));
	printk("OTHERS : %x \n",readl(S5P_OTHERS));
	printk("OM_STAT : %x \n",readl(S5P_OM_STAT));
	printk("MIE_CONTROL : %x \n",readl(S5P_MIE_CONTROL));
	printk("HDMI_CONTROL : %x \n",readl(S5P_HDMI_CONTROL));
	printk("USB_PHY_CONTROL : %x \n",readl(S5P_USB_PHY_CONTROL));
	printk("DAC_CONTROL : %x \n",readl(S5P_DAC_CONTROL));
	printk("MIPI_PHY_CONTROL : %x \n",readl(S5P_MIPI_PHY_CONTROL));
	printk("ADC_CONTROL : %x \n",readl(S5P_ADC_CONTROL));
	printk("PSHOLD_CONTROL : %x \n",readl(S5P_PSHOLD_CONTROL));

	printk("INFORM0 : %x \n",readl(S5P_INFORM0));
	printk("INFORM1 : %x \n",readl(S5P_INFORM1));
	printk("INFORM2 : %x \n",readl(S5P_INFORM2));
	printk("INFORM3 : %x \n",readl(S5P_INFORM3));
	printk("INFORM4 : %x \n",readl(S5P_INFORM4));
	printk("INFORM5 : %x \n",readl(S5P_INFORM5));
	printk("INFORM6 : %x \n",readl(S5P_INFORM6));
	printk("INFORM7 : %x \n",readl(S5P_INFORM7));
}

/* helper functions to save and restore register state */

/**
 * s3c_pm_do_save() - save a set of registers for restoration on resume.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Run through the list of registers given, saving their contents in the
 * array for later restoration when we wakeup.
 */
void s3c_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
	//	S3C_PMDBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/**
 * s3c_pm_do_restore() - restore register values from the save list.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Restore the register values saved from s3c_pm_do_save().
 *
 * Note, we do not use S3C_PMDBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		//printk(KERN_DEBUG "restore %p (restore %08lx, was %08x)\n",
		  //     ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/**
 * s3c_pm_do_restore_core() - early restore register values from save list.
 *
 * This is similar to s3c_pm_do_restore() except we try and minimise the
 * side effects of the function in case registers that hardware might need
 * to work has been restored.
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

void s3c_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		__raw_writel(ptr->val, ptr->reg);
}

/* s3c2410_pm_show_resume_irqs
 *
 * print any IRQs asserted at resume time (ie, we woke from)
*/
static void s3c_pm_show_resume_irqs(int start, unsigned long which,
				    unsigned long mask)
{
	int i;

	which &= ~mask;

	for (i = 0; i <= 31; i++) {
		if (which & (1L<<i)) {
			S3C_PMDBG("IRQ %d asserted at resume\n", start+i);
		}
	}
}

bool s3c_pm_check_pending_interrupt(void)
{
	bool ret=true;

	s5pc11x_pm_wakeup_eint1_pend =__raw_readl(S5PV210_EINT1PEND);
	s5pc11x_pm_wakeup_eint2_pend =__raw_readl(S5PV210_EINT2PEND);

	if(s5pc11x_pm_wakeup_eint1_pend) {	
		if(s5pc11x_pm_wakeup_eint1_pend & EINTPEND1_BIT_ONEDRAM) {
			printk(KERN_DEBUG "%s: cp interrupt pending.\n", __func__);
			ret=false;
		}
	}

	if(s5pc11x_pm_wakeup_eint2_pend) {
		if(s5pc11x_pm_wakeup_eint2_pend & EINTPEND2_BIT_MICROUSB) {
			printk(KERN_DEBUG "%s: micro usb interrupt pending.\n", __func__);
			ret=false;
		}
	}

	return ret;
}


#if defined(CONFIG_MACH_S5PC110_P1)
/* s3c_pm_set_wakeup_stat
 *
 * MIDAS 2010/05/27
 * Set Wakeup stat from sleep
*/
void s3c_pm_set_wakeup_stat()
{
	if(!gpio_get_value(GPIO_nPOWER))
	{
		g_pm_wakeup_stat = PM_POWER_KEY;
	}
}

/* s3c_pm_set_wakeup_stat
 *
 * MIDAS 2010/05/27
 * Get Wakeup stat
*/
int s3c_pm_get_wakeup_stat()
{
	return g_pm_wakeup_stat;
}

/* s3c_pm_set_wakeup_stat
 *
 * MIDAS 2010/05/27
 * Clear Wakeup stat
*/
void s3c_pm_clear_wakeup_stat()
{
	g_pm_wakeup_stat = PM_WAKEUP_NONE;
}

EXPORT_SYMBOL(s3c_pm_get_wakeup_stat);
EXPORT_SYMBOL(s3c_pm_clear_wakeup_stat);
#endif // CONFIG_MACH_S5PC110_P1


void (*pm_cpu_prep)(void);
void (*pm_cpu_sleep)(void);

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

extern short gp2a_get_proximity_enable(void); 

#if defined CONFIG_M115S
void confirm_to_gpio_sleep(void)
{
	printk("%s : Sleep gpio setting confirmed. \n",__func__);
}
EXPORT_SYMBOL(confirm_to_gpio_sleep);
#endif
	
/* s3c_pm_enter
 *
 * central control for sleep/resume process
*/

static int s3c_pm_enter(suspend_state_t state)
{
#ifndef USE_DMA_ALLOC
	static unsigned long regs_save[16];
#endif /* !USE_DMA_ALLOC */
	unsigned int tmp,audiodomain_On;
#if defined(CONFIG_M115S)
    unsigned int gpio;
#endif
	unsigned int gpio;

    if(HWREV >=11) {
        // 20110207 - check pending interrupt to wakeup device
	    if(!s3c_pm_check_pending_interrupt())
	    {
		    printk(KERN_ERR "interrupt pending. wakeup!!(1)\n", __func__);	
		    return -EINVAL;
	    }

		// 20110113 - dukho.kim : control power of moviNAND at PM and add 400ms delay for stabilization of moviNAND. 
		gpio = readl(S5PV210_GPJ2DAT);
		writel(gpio & (~0x80), S5PV210_GPJ2DAT);
		mdelay(400);

        // 20110207 - check pending interrupt to wakeup device
	    if(!s3c_pm_check_pending_interrupt())
	    {
		    printk(KERN_ERR "interrupt pending. wakeup!!(2)\n", __func__);	
		    return -EINVAL;
	    }
	}

	/* ensure the debug is initialised (if enabled) */

	s3c_pm_debug_init();

	S3C_PMDBG("%s(%d)\n", __func__, state);

	if (pm_cpu_prep == NULL || pm_cpu_sleep == NULL) {
		printk(KERN_ERR "%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	s5pc1xx_wakeup_src = 0;
	s5pc11x_pm_wakeup_wakeup_stat = 0;
	s5pc11x_pm_wakeup_eint0_pend = 0;
	s5pc11x_pm_wakeup_eint1_pend = 0;
	s5pc11x_pm_wakeup_eint2_pend = 0;
	s5pc11x_pm_wakeup_eint3_pend = 0;

	/* check if we have anything to wake-up with... bad things seem
	 * to happen if you suspend with no wakeup (system will often
	 * require a full power-cycle)
	*/
	s3c_irqwake_intmask = 0xFFFD; // rtc_alarm

	if (!any_allowed(s3c_irqwake_intmask, s3c_irqwake_intallow) &&
	    !any_allowed(s3c_irqwake_eintmask, s3c_irqwake_eintallow)) {
		printk(KERN_ERR "%s: No wake-up sources!\n", __func__);
		printk(KERN_ERR "%s: Aborting sleep\n", __func__);
		return -EINVAL;
	}

	/* store the physical address of the register recovery block */

#ifndef USE_DMA_ALLOC
	s3c_sleep_save_phys = virt_to_phys(regs_save);
#else
	__raw_writel(phy_regs_save, S5P_INFORM2);
#endif /* !USE_DMA_ALLOC */

	/* set flag for sleep mode idle2 flag is also reserved */
	__raw_writel(SLEEP_MODE, S5P_INFORM1);

#if defined(CONFIG_M115S)
	// 20101005 - dukho.kim : control power of iNAND at PM for stabilization
	gpio_set_value(GPIO_MASSMEMORY_EN, 0);	

	// 20101018 - dukho.kim : set CMD/CLK/DATA0~7 OUTPUT LOW after turing off iNAND power.
	for (gpio = S5PV210_GPG1(3); gpio <= S5PV210_GPG1(6); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(gpio, 0);	
	}
	for (gpio = S5PV210_GPG0(0); gpio <= S5PV210_GPG0(6); gpio++) {
		if(gpio != S5PV210_GPG0(2)) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
			s3c_gpio_setpin(gpio, 0);	
		}
	}
	mdelay(50);	// for iNAND power stabilization 
#endif

	S3C_PMDBG("s3c_sleep_save_phys=0x%08lx\n", s3c_sleep_save_phys);

	/* save all necessary core registers not covered by the drivers */

	s3c_pm_save_gpios();
	s3c_pm_save_uarts();
	s3c_pm_save_core();

	/* set the irq configuration for wake */

	s3c_pm_configure_extint();

	S3C_PMDBG("sleep: irq wakeup masks: %08lx,%08lx\n",
			s3c_irqwake_intmask, s3c_irqwake_eintmask);

	/*Set EINT as wake up source*/
#if defined(CONFIG_OPTICAL_GP2A)
	if(gp2a_get_proximity_enable())
	{
		s3c_pm_set_eint(2, INTR_MODE_EDGE_BOTH); // Proximity
	}
#endif
	s3c_pm_set_eint( 6, INTR_MODE_EDGE_BOTH); // det_3.5
	//s3c_pm_set_eint( 7, 0x2); // pmic
	s3c_pm_set_eint(8,INTR_MODE_EDGE_FALLING); //Fuel Gauge Low Bat
	s3c_pm_set_eint(11, INTR_MODE_EDGE_FALLING); // onedram
	if(gpio_get_value(GPIO_WLAN_nRST))
	{
//		Host Wake operation is not working, kyungjin.moon	
		s3c_pm_set_eint(20,INTR_MODE_EDGE_BOTH); //WLAN Host Wake, WLAN Driver Do this
	}
	if(gpio_get_value(GPIO_BT_nRST))
	{
		s3c_pm_set_eint(21,INTR_MODE_EDGE_RISING); //BT Host Wake	
	}
	s3c_pm_set_eint(22,INTR_MODE_EDGE_FALLING); // power key
	s3c_pm_set_eint(23,INTR_MODE_EDGE_FALLING); // microusb
	s3c_pm_set_eint(24,INTR_MODE_EDGE_FALLING); //OK key
	if(isVoiceCall)
	{
		printk(KERN_DEBUG "call state vol down,up waekup en \n");
		s3c_pm_set_eint(25,INTR_MODE_EDGE_FALLING); //VOL down
		s3c_pm_set_eint(26,INTR_MODE_EDGE_FALLING); //VOL up
	}
	s3c_pm_set_eint(28,INTR_MODE_EDGE_BOTH); // T_FLASH_DETECT
   	if(get_headset_status() & SEC_HEADSET_4_POLE_DEVICE)
	{
	    s3c_pm_set_eint(30,INTR_MODE_EDGE_BOTH); //sendend
	}
    else
    {
        s3c_pm_clear_eint(30);
    }

	printk(KERN_DEBUG "rtc_pa_base = 0x%x, 0x%x\n", *(int *)rtc_pa_base, rtc_pa_base); //rtc_pa_base
	s5pc11x_pm_check_auto_wakeup();

	//s3c_config_sleep_gpio();

	//s3c_pm_arch_prepare_irqs();
	

	/* call cpu specific preparation */

	pm_cpu_prep();

	/* flush cache back to ram */

	flush_cache_all();

	s3c_pm_check_store();

	__raw_writel(s3c_irqwake_intmask, S5P_WAKEUP_MASK); //0xFFDD:key, RTC_ALARM	
	

	/*clear for next wakeup*/
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	__raw_writel(tmp, S5P_WAKEUP_STAT);

	s3c_config_sleep_gpio();
#if defined CONFIG_M115S
	confirm_to_gpio_sleep();
#endif

	// Enable PS_HOLD pin to avoid reset failure */
        __raw_writel((0x5 << 12 | 0x1<<9 | 0x1<<8 | 0x1<<0),S5P_PSHOLD_CONTROL);


	/* send the cpu to sleep... */

	s3c_pm_arch_stop_clocks();

	/* s3c_cpu_save will also act as our return point from when
	 * we resume as it saves its own register state and restores it
	 * during the resume.  */

	s3c_cpu_save(regs_save);

	/* restore the cpu state using the kernel's cpu init code. */

	cpu_init();
	

	/* restore the system state */

	s3c_pm_restore_core();

	/*Reset the uart registers*/
	__raw_writel(0x0, S3C24XX_VA_UART3+S3C2410_UCON);
	__raw_writel(0xf, S3C24XX_VA_UART3+S5P_UINTM);
	__raw_writel(0xf, S3C24XX_VA_UART3+S5P_UINTSP);
	__raw_writel(0xf, S3C24XX_VA_UART3+S5P_UINTP);
	__raw_writel(0x0, S3C24XX_VA_UART2+S3C2410_UCON);
	__raw_writel(0xf, S3C24XX_VA_UART2+S5P_UINTM);
	__raw_writel(0xf, S3C24XX_VA_UART2+S5P_UINTSP);
	__raw_writel(0xf, S3C24XX_VA_UART2+S5P_UINTP);
	__raw_writel(0x0, S3C24XX_VA_UART1+S3C2410_UCON);
	__raw_writel(0xf, S3C24XX_VA_UART1+S5P_UINTM);
	__raw_writel(0xf, S3C24XX_VA_UART1+S5P_UINTSP);
	__raw_writel(0xf, S3C24XX_VA_UART1+S5P_UINTP);
	__raw_writel(0x0, S3C24XX_VA_UART0+S3C2410_UCON);
	__raw_writel(0xf, S3C24XX_VA_UART0+S5P_UINTM);
	__raw_writel(0xf, S3C24XX_VA_UART0+S5P_UINTSP);
	__raw_writel(0xf, S3C24XX_VA_UART0+S5P_UINTP);

	s3c_pm_restore_uarts();
	s3c_pm_restore_gpios();

	tmp = readl(S5P_NORMAL_CFG);
	if(!(tmp & S5PC110_POWER_DOMAIN_AUDIO)) {
		tmp = tmp | S5PC110_POWER_DOMAIN_AUDIO;
		writel(tmp , S5P_NORMAL_CFG);
		audiodomain_On = 1;
	} else {
		audiodomain_On = 0;
	}

	/* enable gpio, uart, mmc */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= (1<<31) | (1<<30) | (1<<28) | (1<<29);
	__raw_writel(tmp, S5P_OTHERS);

	tmp = readl(S5P_NORMAL_CFG);
	if (audiodomain_On) {
		tmp = tmp & ~S5PC110_POWER_DOMAIN_AUDIO;
		writel(tmp , S5P_NORMAL_CFG);
	}

	__raw_writel(TSADC_ENABLE,S5P_ADC_CONTROL); //WA for ADC Read Lockup

	s5pc11x_pm_wakeup_wakeup_stat =__raw_readl(S5P_WAKEUP_STAT);
	s5pc11x_pm_wakeup_eint0_pend =__raw_readl(S5PV210_EINT0PEND);	
	s5pc11x_pm_wakeup_eint1_pend =__raw_readl(S5PV210_EINT1PEND);
	s5pc11x_pm_wakeup_eint2_pend =__raw_readl(S5PV210_EINT2PEND);
	s5pc11x_pm_wakeup_eint3_pend =__raw_readl(S5PV210_EINT3PEND);

	/*clear for next wakeup*/
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	//printk("\nS5P_WAKEUP_STAT=%x\n",tmp);
	__raw_writel(tmp, S5P_WAKEUP_STAT);

	s5pc11x_pm_check_wakeupsrc();
	if(s5pc1xx_wakeup_src & WAKEUP_SRC_PWRBUTTON)
	{
		s3c_pm_clear_eint(22);
		DBG("clear eint pending 22 \n");
	}else if(s5pc1xx_wakeup_src & WAKEUP_SRC_HOME)
	{
		s3c_pm_clear_eint(24);
		DBG("clear eint pending 24 \n");
	}else if(s5pc1xx_wakeup_src & WAKEUP_SRC_VOLDN)
	{
		s3c_pm_clear_eint(25);
		DBG("clear eint pending 25 \n");
	}else if(s5pc1xx_wakeup_src & WAKEUP_SRC_VOLUP)
	{
		s3c_pm_clear_eint(26);
		DBG("clear eint pending 26 \n");
	}

	/* check what irq (if any) restored the system */
	s3c_pm_debug_init();

	s3c_pm_arch_show_resume_irqs();



#if defined(CONFIG_MACH_S5PC110_P1)
	// Set wakeup stat
	s3c_pm_set_wakeup_stat();
#endif // CONFIG_MACH_S5PC110_P1

	//printk("Int pending register before =%d\n",readl(S5PV210_EINTPEND(eint_pend_reg(22))));

	//printk("Int pending register after =%d\n",readl(S5PV210_EINTPEND(eint_pend_reg(22))));

	//S3C_PMDBG("%s: post sleep, preparing to return\n", __func__);
	//printk("%s: post sleep, preparing to return\n", __func__);

	/* LEDs should now be 1110 */
	//s3c_pm_debug_smdkled(1 << 1, 0);
	if (pm_use_rtc_tick == 1)
    {
        pm_use_rtc_tick = 0;
        s5pc11x_pm_disable_rtctic();
		printk(KERN_DEBUG "s5pc11x_pm_disable_rtctic...\n");
    }

	if(ftm_sleep == 1)
	{
		ftm_sleep_exit = 1;
		printk(KERN_DEBUG "Exit FTM Sleep...\n");
	}

	s3c_pm_check_restore();

	//mdelay(500);

	/* ok, let's return from sleep */
	printk(KERN_ERR "\n%s:%d\n", __func__, __LINE__);

	S3C_PMDBG("S3C PM Resume (post-restore)\n");
	return 0;
}

/* callback from assembly code */
void s3c_pm_cb_flushcache(void)
{
	flush_cache_all();
}

static int s3c_pm_prepare(void)
{
	/* prepare check area if configured */

	s3c_pm_check_prepare();
	return 0;
}

static void s3c_pm_finish(void)
{
	s3c_pm_check_cleanup();
}

static struct platform_suspend_ops s3c_pm_ops = {
	.enter		= s3c_pm_enter,
	.prepare	= s3c_pm_prepare,
	.finish		= s3c_pm_finish,
	.valid		= suspend_valid_only_mem,
};

/* s3c_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s3c_pm_init(void)
{
	printk("S3C Power Management, Copyright 2004 Simtec Electronics\n");

#ifdef USE_DMA_ALLOC
	regs_save = dma_alloc_coherent(NULL, 4096, &phy_regs_save, GFP_KERNEL);
	if (regs_save == NULL) {
		printk(KERN_ERR "DMA alloc error\n");
		return -1;
	}
#endif /* USE_DMA_ALLOC */

	rtc_pa_base = ioremap(S5PV210_PA_RTC, 0x100); //0xe2800000 //S5PV210_PA_RTC
    if(!rtc_pa_base) {
            printk(KERN_DEBUG "Unable to allocate rtc memory\n");
            return -1;
    }

	suspend_set_ops(&s3c_pm_ops);
	return 0;
}
