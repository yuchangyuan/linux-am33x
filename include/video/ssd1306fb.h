/*
 * linux/include/video/ssd1306fb.h -- FB driver for SSD1306 OLED controller
 *
 * Copyright (C) 2012, Yu Changyuan
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

struct ssd1306fb_par {
	struct spi_device *spi;
	struct fb_info *info;
	int rst;
	int dc;
	u8 *buf;
};

struct ssd1306fb_platform_data {
	int rst_gpio;
	int dc_gpio;
};
