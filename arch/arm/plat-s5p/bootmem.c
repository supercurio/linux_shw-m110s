/* linux/arch/arm/plat-s5p/bootmem.c
 *
 * Copyright (c) 2009 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Bootmem helper functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/swap.h>
#include <asm/setup.h>
#include <linux/io.h>
#include <mach/memory.h>
#include <mach/map.h>
#include <plat/media.h>

static struct s3c_media_device media_devs[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
	{
		.id = S3C_MDEV_MFC,
		.name = "mfc",
		.bank = 0,
		.memsize = MFC0_MEM_SIZE,//CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
		.paddr = MFC0_MEM_BASE,//0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
	{
		.id = S3C_MDEV_MFC,
		.name = "mfc",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
	{
		.id = S3C_MDEV_FIMC0,
		.name = "fimc0",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
	{
		.id = S3C_MDEV_FIMC1,
		.name = "fimc1",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
	{
		.id = S3C_MDEV_FIMC2,
		.name = "fimc2",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV
	{
		.id = S3C_MDEV_TV,
		.name = "tv",
		.bank = 0,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef	CONFIG_ANDROID_PMEM_MEMSIZE_PMEM
/* pmem */
	{
		.id = S3C_MDEV_PMEM,
		.name = "pmem",
		.memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K,
		.paddr = 0,
		.bank = 0, //OneDRAM
	},
#endif

#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1
        {
                .id = S3C_MDEV_PMEM_GPU1,
                .name = "pmem_gpu1",
                .memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K,
                .paddr = 0,
		.bank = 0, //OneDRAM 
        },
        {
                .id = S3C_MDEV_PMEM_ADSP,
                .name = "pmem_adsp",
                .memsize = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_ADSP * SZ_1K,
                .paddr = 0,
		.bank = 0, //OneDRAM
        },
#endif


#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
	{
		.id = S3C_MDEV_JPEG,
		.name = "jpeg",
		.bank = 0,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM
	{
		.id = S3C_MDEV_TEXSTREAM,
		.name = "texstream",
		.bank = 0,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM * SZ_1K,
		.paddr = 0,
	},
#endif

#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
#if defined (CONFIG_FB_S3C_WINDOW0) || (CONFIG_FB_S3C_DEFAULT_WINDOW == 0)
	{
		.id = S3C_MDEV_FIMD_FB0,
		.name = "fimd",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
		.paddr = 0,
	},
#endif
#if defined (CONFIG_FB_S3C_WINDOW1) || (CONFIG_FB_S3C_DEFAULT_WINDOW == 1)
	{
		.id = S3C_MDEV_FIMD_FB1,
		.name = "fimd",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
		.paddr = 0,
	},
#endif
#if defined (CONFIG_FB_S3C_WINDOW2) || (CONFIG_FB_S3C_DEFAULT_WINDOW == 2)
	{
		.id = S3C_MDEV_FIMD_FB2,
		.name = "fimd",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
		.paddr = 0,
	},
#endif
#if defined (CONFIG_FB_S3C_WINDOW3) || (CONFIG_FB_S3C_DEFAULT_WINDOW == 3)
	{
		.id = S3C_MDEV_FIMD_FB3,
		.name = "fimd",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
		.paddr = 0,
	},
#endif
#if defined (CONFIG_FB_S3C_WINDOW4) || (CONFIG_FB_S3C_DEFAULT_WINDOW == 4)
	{
		.id = S3C_MDEV_FIMD_FB4,
		.name = "fimd",
		.bank = 1,
		.memsize = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
		.paddr = 0,
	},
#endif
#endif

#ifdef CONFIG_WLAN_SAMSUNG_MEMSIZE_BCM
	{
		.id = S3C_MDEV_WIFI,
		.name = "wifi",
		.bank = 0,
		.memsize = CONFIG_WLAN_SAMSUNG_MEMSIZE_BCM * SZ_1K,
		.paddr = 0,
	},
#endif
};

static struct s3c_media_device *s3c_get_media_device(int dev_id, int bank)
{
	struct s3c_media_device *mdev = NULL;
	int i = 0, found = 0, nr_devs;
	nr_devs = sizeof(media_devs) / sizeof(media_devs[0]);

	if (dev_id < 0)
		return NULL;

	while (!found && (i < nr_devs)) {
		mdev = &media_devs[i];
		if (mdev->id == dev_id && mdev->bank == bank)
			found = 1;
		else
			i++;
	}

	if (!found)
		mdev = NULL;

	return mdev;
}

dma_addr_t s3c_get_media_memory_bank(int dev_id, int bank)
{
	struct s3c_media_device *mdev;

	mdev = s3c_get_media_device(dev_id, bank);
	if (!mdev) {
		printk(KERN_ERR "invalid media device\n");
		return 0;
	}

	if (!mdev->paddr) {
		printk(KERN_ERR "no memory for %s\n", mdev->name);
		return 0;
	}

	return mdev->paddr;
}
EXPORT_SYMBOL(s3c_get_media_memory_bank);

size_t s3c_get_media_memsize_bank(int dev_id, int bank)
{
	struct s3c_media_device *mdev;

	mdev = s3c_get_media_device(dev_id, bank);
	if (!mdev) {
		printk(KERN_ERR "invalid media device\n");
		return 0;
	}

	return mdev->memsize;
}
EXPORT_SYMBOL(s3c_get_media_memsize_bank);

void s5pv210_reserve_bootmem(void)
{
	struct s3c_media_device *mdev;
	int i, nr_devs;

	nr_devs = sizeof(media_devs) / sizeof(media_devs[0]);
	for (i = 0; i < nr_devs; i++) {
		mdev = &media_devs[i];
		if (mdev->memsize <= 0)
			continue;
        if (mdev->paddr != 0)
            continue;

		mdev->paddr = virt_to_phys(__alloc_bootmem(mdev->memsize,
				PAGE_SIZE, meminfo.bank[mdev->bank].start));

#if 0
		printk(KERN_INFO "s5pv210: %lu bytes system memory reserved "
			"for %s at 0x%08x\n", (unsigned long) mdev->memsize,
			mdev->name, mdev->paddr);
#else
        printk(KERN_INFO "memory reserved for %s, \t0x%08X ~ 0x%08X (%8lu bytes)\n",
            mdev->name, mdev->paddr, mdev->paddr + (unsigned long)mdev->memsize - 1, (unsigned long)mdev->memsize);
#endif
	}
}

void reserve_fixed_bootmem(void)
{
    // bellows will be allocated in OneDRAM
    struct fixed_memory_map fixed_map[] = {
        {
            .name  = "mfc",
            .size  = MFC0_MEM_SIZE,
            .paddr = MFC0_MEM_BASE,
        },

        {
            .name  = "fbuffer",
            .size  = LOGO_MEM_SIZE,
            .paddr = LOGO_MEM_BASE,
        },
    };

    int i, nr_devs = sizeof(fixed_map) / sizeof(fixed_map[0]);

    for (i = 0; i < nr_devs; i++) 
    {
        if (reserve_bootmem(fixed_map[i].paddr, fixed_map[i].size, BOOTMEM_EXCLUSIVE) < 0)
    		printk(KERN_ERR "failed to reserve VRAM for %s\n", fixed_map[i].name);
        else
            printk(KERN_INFO "memory reserved for %s, \t0x%08X ~ 0x%08X (%8lu bytes)\n", 
                fixed_map[i].name, fixed_map[i].paddr, fixed_map[i].paddr+fixed_map[i].size-1, fixed_map[i].size);
	}
}


/* FIXME: temporary implementation to avoid compile error */
int dma_needs_bounce(struct device *dev, dma_addr_t addr, size_t size)
{
	return 0;
}

