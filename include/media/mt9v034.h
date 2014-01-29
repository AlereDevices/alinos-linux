/* mt9v034 Camera
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MT9V034_H__
#define __MT9V034_H__

//#include <media/v4l2-subdev.h>


/* 
 Slave address modes
 -------------------
 ADR1 = 0, ADR0 = 0. Slave address = 0x90
 ADR1 = 0, ADR0 = 1. Slave address = 0x98
 ADR1 = 1, ADR0 = 0. Slave address = 0xB0
 ADR1 = 1, ADR0 = 1. Slave address = 0xB8 
 SW3 default config = 11
*/
#define MT9V034_I2C_ADDR	0x48 //(0x90 >> 1)
//#define MT9V034_I2C_ADDR	0x5C //(0xB8 >> 1)

struct v4l2_subdev;

enum {
	MT9V034_COLOR_VERSION,
	MT9V034_MONOCHROME_VERSION,
};

struct mt9v034_platform_data {
	int (*set_xclk)(struct v4l2_subdev *subdev, int hz);
	int (*reset)(struct v4l2_subdev *subdev, int active);
	int ext_freq; /* input frequency to the mt9v034 for PLL dividers */
	int target_freq; /* frequency target for the PLL */
	int version;
};
#endif /* __MT9V034_H__ */
