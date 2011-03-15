/* linux/drivers/input/keyboard/s3c-keypad.h 
 *
 * Driver header for Samsung SoC keypad.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef _S3C_KEYPAD_H_
#define _S3C_KEYPAD_H_

static void __iomem *key_base;

#define KEYPAD_COLUMNS	2
#define KEYPAD_ROWS		3
#define MAX_KEYPAD_NR	6

#define KEYCODE_UNKNOWN 		10 
#define KEYCODE_VOLUME_UP 		42
#define KEYCODE_VOLUME_DOWN 	58
#define KEYCODE_HOME			50
#define KEYCODE_POWER			26
#define KEYCODE_ENTER			34
#define KEYCODE_FOCUS			3
#define KEYCODE_CAMERA			8
#define KEYCODE_CLEAR			19

int keypad_keycode[] = {
	KEYCODE_HOME,KEYCODE_VOLUME_DOWN,KEYCODE_VOLUME_UP,KEYCODE_UNKNOWN,KEYCODE_VOLUME_DOWN,KEYCODE_VOLUME_UP,
};

#if CONFIG_ANDROID
#ifdef CONFIG_CPU_S3C6410
#define KEYPAD_ROW_GPIOCON      S3C64XX_GPKCON1
#define KEYPAD_ROW_GPIOPUD      S3C64XX_GPKPUD
#define KEYPAD_COL_GPIOCON      S3C64XX_GPLCON
#define KEYPAD_COL_GPIOPUD      S3C64XX_GPLPUD
#elif defined( CONFIG_CPU_S5PC100 )
#define KEYPAD_ROW_GPIOCON      S5PC1XX_GPH3CON
#define KEYPAD_ROW_GPIOPUD      S5PC1XX_GPH3PUD
#define KEYPAD_COL_GPIOCON      S5PC1XX_GPH2CON
#define KEYPAD_COL_GPIOPUD      S5PC1XX_GPH2PUD
#elif defined( CONFIG_CPU_S5PC110 ) || defined (CONFIG_CPU_S5PV210 )
#define KEYPAD_ROW_GPIOCON      S5PV210_GPH3CON
#define KEYPAD_ROW_GPIOPUD      S5PV210_GPH3PUD
#define KEYPAD_COL_GPIOCON      S5PV210_GPH2CON
#define KEYPAD_COL_GPIOPUD      S5PV210_GPH2PUD
#endif
#endif /* CONFIG_ANDROID */

#ifdef CONFIG_CPU_S3C6410
#define KEYPAD_DELAY		(50)
#elif defined(CONFIG_CPU_S5PC100) || defined(CONFIG_CPU_S5PC110) || defined(CONFIG_CPU_S5PV210)
#define KEYPAD_DELAY		(300)  //600
#endif

#define	KEYIFCOL_CLEAR		(readl(key_base+S3C_KEYIFCOL) & ~0xffff)
#define	KEYIFCON_CLEAR		(readl(key_base+S3C_KEYIFCON) & ~0x1f)
#define 	KEYIFFC_DIV		(readl(key_base+S3C_KEYIFFC) | 0x1)

struct s3c_keypad {
	struct input_dev *dev;
	int nr_rows;	
	int no_cols;
	int total_keys; 
	int keycodes[MAX_KEYPAD_NR];
};

extern void s3c_setup_keypad_cfg_gpio(int rows, int columns);

#endif				/* _S3C_KEYIF_H_ */
