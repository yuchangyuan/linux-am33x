/*
 * linux/drivers/video/ssd1306fb.c -- FB driver for SSD1306 OLED controller
 *
 * based on st7856fb.c
 * Copyright (C) 2012 Yu Changyuan
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <video/ssd1306fb.h>

#define DRVNAME		"ssd1306fb"
#define WIDTH		128
#define HEIGHT		64

#define NATIVE_VMEM_SIZE	(WIDTH * HEIGHT / 8)
#define VMEM_SIZE		(WIDTH * HEIGHT / 8)

static struct fb_fix_screeninfo ssd1306fb_fix __devinitdata = {
	.id		= "SSD1306",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.line_length	= WIDTH/8,
	.accel		= FB_ACCEL_NONE,
};


static struct fb_var_screeninfo ssd1306fb_var __devinitdata = {
	.xres		= WIDTH,
	.yres		= HEIGHT,
	.xres_virtual	= WIDTH,
	.yres_virtual	= HEIGHT,
	.bits_per_pixel	= 1,
	.nonstd		= 1,
};


static int ssd1306_write(struct ssd1306fb_par *par,
			 u8 *txbuf, int size)
{
	int ret = 0;

	/* Write entire buffer */
	ret = spi_write(par->spi, txbuf, size);

	return ret;
}

static int ssd1306_write_cmd(struct ssd1306fb_par *par,
			      u8 *cmd, int size)
{
	int ret = 0;
	int i;

	/* Set command mode */
	gpio_set_value(par->dc, 0);

	ret = ssd1306_write(par, cmd, size);
	if (ret < 0) {
		pr_err("%s: write %d bytes command failed with status %d\n",
		       par->info->fix.id, size, ret);
		for (i = 0; i < size; ++i)  pr_err("  %02x", cmd[i]);
	}

	return ret;
}

static int ssd1306_write_data(struct ssd1306fb_par *par,
			       u8 *data, int size)
{
	int ret = 0;

	/* Set data mode */
	gpio_set_value(par->dc, 1);

	ret = ssd1306_write(par, data, size);

	if (ret < 0)
		pr_err("%s: write %d bytes data failed with status %d\n",
			par->info->fix.id, size, ret);

	return ret;
}

static void ssd1306_set_page_column(struct ssd1306fb_par *par,
				    int page, int column)
{
	u8 cmd[] = {
		/* set page */
		0xB0 | page,
		/* set column */
		0x00 | column % 0xf,
		0x10 | column >> 4,
	};

	ssd1306_write_cmd(par, cmd , sizeof(cmd));
}


static void ssd1306_clear(struct ssd1306fb_par *par)
{
	u8 *buf;

	buf = kzalloc(NATIVE_VMEM_SIZE, GFP_KERNEL);

	ssd1306_set_page_column(par, 0, 0);
	ssd1306_write_data(par, buf, NATIVE_VMEM_SIZE);

	kfree(buf);
}



static void ssd1306_reset(struct ssd1306fb_par *par)
{
	/* Reset controller */
	gpio_set_value(par->rst, 0);
	mdelay(10);
	gpio_set_value(par->rst, 1);
	mdelay(50);
}

static void ssd1306fb_update_display(struct ssd1306fb_par *par)
{
	int ret = 0;
	u8 *vmem = par->info->screen_base;
	u8 *buf = par->buf;
	int i, j;

	memset(buf, 0, NATIVE_VMEM_SIZE);
	/* translate vmem to buf
	   - page0: colum0 ~ colum127
	   - ...
	   - page7: colum0 ~ colum127
	 */
	for (i = 0; i < HEIGHT; ++i) {
		u8 mask = 1 << (i % 8);
		for (j = 0; j < WIDTH / 8; ++j) {
			u8 v = vmem[i * WIDTH / 8 + j];
			u8 *p = buf + i / 8 * WIDTH + j * 8;
			if (v & 0x80) p[0] |= mask;
			if (v & 0x40) p[1] |= mask;
			if (v & 0x20) p[2] |= mask;
			if (v & 0x10) p[3] |= mask;
			if (v & 0x08) p[4] |= mask;
			if (v & 0x04) p[5] |= mask;
			if (v & 0x02) p[6] |= mask;
			if (v & 0x01) p[7] |= mask;
		}
	}

	ssd1306_set_page_column(par, 0, 0);

	/* Blast framebuffer to SSD1306 internal display RAM */
	ret = ssd1306_write_data(par, buf, NATIVE_VMEM_SIZE);

	if (ret < 0)
		pr_err("%s: spi_write failed to update display buffer\n",
			par->info->fix.id);
}

static void ssd1306fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	ssd1306fb_update_display(info->par);
}

static int ssd1306fb_init_display(struct ssd1306fb_par *par)
{
	u8 cmd_init[] = {
		/* display off */
		0xAE,
		/* set display clock */
		0xD5, 0xF1,
		/* set 1/64 duty */
		0xA8, 0x3F,
		/* shift mapping ram counter */
		0xD3, 0x00,
		0x40,
		0x8D, 0x14,
		0x20, 0x00,
		0xA1,
		0xC8,
		0xDA, 0x12,
		0x81, 0x80,
		0xD9, 0xF1,
		0xDB, 0x40,
		0xA4,
		0xA6,
	};

	u8 cmd_display_on[] = {
		/* display on */
		0xAF,
	};

        /* Request GPIOs and initialize to default values */
        gpio_request(par->rst, "SSD1306 Reset Pin");
	gpio_direction_output(par->rst, 1);
        gpio_request(par->dc, "SSD1306 D/C Pin");
	gpio_direction_output(par->dc, 0);

	ssd1306_reset(par);

	/* init */
	ssd1306_write_cmd(par, cmd_init, sizeof(cmd_init));

	/* clear */
	ssd1306_clear(par);

	/* display on */
	ssd1306_write_cmd(par, cmd_display_on, sizeof(cmd_display_on));


	return 0;
}

void ssd1306fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct ssd1306fb_par *par = info->par;

	sys_fillrect(info, rect);

	ssd1306fb_update_display(par);
}

void ssd1306fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct ssd1306fb_par *par = info->par;

	sys_copyarea(info, area);

	ssd1306fb_update_display(par);
}

void ssd1306fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct ssd1306fb_par *par = info->par;

	sys_imageblit(info, image);

	ssd1306fb_update_display(par);
}

static ssize_t ssd1306fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ssd1306fb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	ssd1306fb_update_display(par);

	return (err) ? err : count;
}


static struct fb_ops ssd1306fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= ssd1306fb_write,
	.fb_fillrect	= ssd1306fb_fillrect,
	.fb_copyarea	= ssd1306fb_copyarea,
	.fb_imageblit	= ssd1306fb_imageblit,
};

static struct fb_deferred_io ssd1306fb_defio = {
	.delay		= HZ / 20,
	.deferred_io	= ssd1306fb_deferred_io,
};

static int __devinit ssd1306fb_probe (struct spi_device *spi)
{
	/* TODO, ??
	  int chip = spi_get_device_id(spi)->driver_data;
	*/
	struct ssd1306fb_platform_data *pdata = spi->dev.platform_data;
	int vmem_size = VMEM_SIZE;
	u8 *vmem;
	struct fb_info *info;
	struct ssd1306fb_par *par;
	int retval = -ENOMEM;

	/* TODO, ??
	if (chip != SSD1306_DISPLAY_GENERIC_LCD) {
		pr_err("%s: only the %s device is supported\n", DRVNAME,
			to_spi_driver(spi->dev.driver)->id_table->name);
		return -EINVAL;
	}
	*/

	if (!pdata) {
		pr_err("%s: platform data required for rst and dc info\n",
			DRVNAME);
		return -EINVAL;
	}

	vmem = (u8 *)kmalloc(vmem_size, GFP_KERNEL);
	if (!vmem)
		return retval;

	info = framebuffer_alloc(sizeof(struct ssd1306fb_par), &spi->dev);
	if (!info)
		goto fballoc_fail;

	info->screen_base = vmem;
	info->fbops = &ssd1306fb_ops;
	info->fix = ssd1306fb_fix;
	info->fix.smem_start = virt_to_phys(vmem);
	info->fix.smem_len = vmem_size;
	info->var = ssd1306fb_var;
	/* The SSD1306 packed pixel format does not translate well here */
	/* FIXME - change to mono reporting */
	/*
	info->var.red.offset = 11;
	info->var.red.length = 5;
	info->var.green.offset = 5;
	info->var.green.length = 6;
	info->var.blue.offset = 0;
	info->var.blue.length = 5;
	info->var.transp.offset = 0;
	info->var.transp.length = 0;
	*/
	info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
	info->fbdefio = &ssd1306fb_defio;
	fb_deferred_io_init(info);

	par = info->par;
	par->info = info;
	par->spi = spi;
	par->rst = pdata->rst_gpio;
	par->dc = pdata->dc_gpio;
	par->buf = (u8 *)kmalloc(NATIVE_VMEM_SIZE, GFP_KERNEL);
	if (par->buf < 0)
		goto fballoc_fail;

	retval = register_framebuffer(info);
	if (retval < 0)
		goto fbreg_fail;

	spi_set_drvdata(spi, info);

	retval = ssd1306fb_init_display(par);
	if (retval < 0)
		goto init_fail;

	printk(KERN_INFO
		"fb%d: %s frame buffer device, using %d KiB of video memory\n",
		info->node, info->fix.id, vmem_size/1024);

	return 0;

init_fail:
	spi_set_drvdata(spi, NULL);

fbreg_fail:
	kfree(par->buf);
	framebuffer_release(info);
fballoc_fail:
	kfree(vmem);

	return retval;
}


static int __devexit ssd1306fb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);

	spi_set_drvdata(spi, NULL);

	if (info) {
		struct ssd1306fb_par *par = info->par;

		unregister_framebuffer(info);
		kfree(info->screen_base);
		kfree(par->buf);
		gpio_free(par->rst);
		gpio_free(par->dc);
		framebuffer_release(info);
	}

	return 0;
}


static struct spi_driver ssd1306fb_driver = {
	.driver = {
		.name = "ssd1306fb",
		.owner = THIS_MODULE,
	},
	.probe = ssd1306fb_probe,
	.remove = __devexit_p(ssd1306fb_remove),
};

static int __init ssd1306fb_init(void)
{
	return spi_register_driver(&ssd1306fb_driver);
}

static void __exit ssd1306fb_exit(void)
{
	spi_unregister_driver(&ssd1306fb_driver);
}

/* ------------------------------------------------------------------------- */

module_init(ssd1306fb_init);
module_exit(ssd1306fb_exit);

MODULE_DESCRIPTION("FB driver for SSD1306 OLED controller");
MODULE_AUTHOR("Yu Changyuan");
MODULE_LICENSE("GPL");
