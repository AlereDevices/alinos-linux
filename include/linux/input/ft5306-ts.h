/*
 * Driver for FocalTech FT5306DE touchscreen controller
 *
 * Copyright (c) 2013 Alere International Ltd <rob.voisey@alere.com>
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

#ifndef __FT5306DE_TS_H__
#define __FT5306DE_TS_H__

/*
 * @gpio_int		interrupt gpio
 * @gpio_rst		reset gpio
 * @gpio_wake		wake up gpio
 * @x_max			x-resolution
 * @y_max			y-resolution
 */
struct ft5306de_ts_platdata {
	int gpio_int;
	int gpio_rst;
	int gpio_wake;

	int int_setting;

	unsigned int x_max;
	unsigned int y_max;
};

#endif
