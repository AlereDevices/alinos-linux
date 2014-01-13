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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input/ft5306-ts.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define FT5306_REG_DEVICE_MODE			0x00
#define FT5306_REG_GEST_ID				0x01
#define FT5306_REG_TD_STATUS			0x02
#define FT5306_REG_TOUCH1_XH			0x03
#define FT5306_REG_TOUCH1_XL			0x04
#define FT5306_REG_TOUCH1_YH			0x05
#define FT5306_REG_TOUCH1_YL			0x06
#define FT5306_REG_TOUCH2_XH			0x09
#define FT5306_REG_TOUCH2_XL			0x0a
#define FT5306_REG_TOUCH2_YH			0x0b
#define FT5306_REG_TOUCH2_YL			0x0c
#define FT5306_REG_TOUCH3_XH			0x0f
#define FT5306_REG_TOUCH3_XL			0x10
#define FT5306_REG_TOUCH3_YH			0x11
#define FT5306_REG_TOUCH3_YL			0x12
#define FT5306_REG_TOUCH4_XH			0x15
#define FT5306_REG_TOUCH4_XL			0x16
#define FT5306_REG_TOUCH4_YH			0x17
#define FT5306_REG_TOUCH4_YL			0x18
#define FT5306_REG_TOUCH5_XH			0x1b
#define FT5306_REG_TOUCH5_XL			0x1c
#define FT5306_REG_TOUCH5_YH			0x1d
#define FT5306_REG_TOUCH5_YL			0x1e
#define FT5306_REG_ID_G_THGROUP			0x80
#define FT5306_REG_ID_G_THPEAK			0x81
#define FT5306_REG_ID_G_THCAL			0x82
#define FT5306_REG_ID_G_THWATER			0x83
#define FT5306_REG_ID_G_TEMP			0x84
#define FT5306_REG_ID_G_THDIFF			0x85
#define FT5306_REG_ID_G_CTRL			0x86
#define FT5306_REG_ID_G_MON_TIME		0x87
#define FT5306_REG_ID_G_PERIODACTIVE	0x88
#define FT5306_REG_ID_G_PERIODMONITOR	0x89
#define FT5306_REG_ID_G_AUTO_CALIB		0xa0
#define FT5306_REG_ID_G_VERSION_H		0xa1
#define FT5306_REG_ID_G_VERSION_L		0xa2
#define FT5306_REG_ID_G_CIPHER			0xa3
#define FT5306_REG_ID_G_MODE			0xa4
#define FT5306_REG_ID_G_PMODE			0xa5
#define FT5306_REG_ID_G_FIRMID			0xa6
#define FT5306_REG_ID_G_STATE			0xa7
#define FT5306_REG_ID_G_FT5201ID		0xa8
#define FT5306_REG_ID_G_ERR				0xa9
#define FT5306_REG_ID_G_CALIB			0xaa
#define FT5306_REG_LOG_MSG_CNT			0xfe
#define FT5306_REG_LOG_CUR_CHA			0xff

#define FT5306_REPORT_POINTS			5
#define FT5306_INT_ENABLE				0x00
#define FT5306_INT_DISABLE				0x01

#define FT5306_EVENT_DOWN				0x00
#define FT5306_EVENT_UP					0x01
#define FT5306_EVENT_ON					0x02
#define FT5306_EVENT_RESERVED			0x03

struct ft5306de_ts {
	struct i2c_client					*client;
	struct input_dev					*input;
	const struct ft5306de_ts_platdata	*pdata;
	char								phys[32];
	wait_queue_head_t					wait;
	bool								stopped;
};

/**
 *	Bottom half handler
 */
static irqreturn_t ft5306de_interrupt(int irq, void *dev_id)
{
	struct ft5306de_ts *ts = dev_id;
	struct i2c_client *client = ts->client;
	char addr = 0x03;
	char data[0x1e];
	int i, type, x, y;

	struct i2c_msg msgs[] = {
		{	/* Set read address to start of report data */
			.addr = client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = &addr,
		},
		{	/* Read touch data block */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len   = 0x1e,
			.buf   = data,
		},
	};

	/* Get report set */
	if (i2c_transfer(client->adapter, msgs, 2) != 2) {
		dev_err(&client->dev, "unable to transfer touch data\n");
		return IRQ_HANDLED;
	}

	/* Parse reports */
	for (i = 0; i < FT5306_REPORT_POINTS; i++) {
		u8 *buf = &data[i * 6];

		type = buf[0] >> 6;
		if (type == FT5306_EVENT_RESERVED)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;

		input_mt_slot(ts->input, i);

		if (type == FT5306_EVENT_UP) {
			/* Released */
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		}
		else {
			/* New or sustained touch */
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		}
	}

	input_mt_report_pointer_emulation(ts->input, true);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

/**
 *	Control the generation of interrupts on the device side
 *
 *	The documentation for FT5306_REG_ID_G_MODE appears to be wrong,
 *	as this function does not correctly re-enable interrupts.
 *
 *	TODO Fix this
 */
static int ft5306de_int_control(struct ft5306de_ts *ts, bool enable)
{
#if 0
	struct i2c_client *client = ts->client;
	int mode;
	int ret;

	if (enable)
		mode = FT5306_INT_ENABLE;
	else
		mode = FT5306_INT_DISABLE;

	ret = i2c_smbus_write_byte_data(client, FT5306_REG_ID_G_MODE,
					mode);
	if (ret < 0) {
		dev_err(&client->dev, "unable to write reg %Xh, %d\n",
				FT5306_REG_ID_G_MODE, mode);
		return ret;
	}
#endif
	return 0;
}

static int ft5306de_start(struct ft5306de_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;

	/* Enable interrupt handling */
	ts->stopped = false;
	mb();
	enable_irq(client->irq);

	/* Enable interrupt generation */
	ret = ft5306de_int_control(ts, 1);
	if (ret < 0) {
		dev_err(&client->dev, "could not enable interrupt, %d\n",
			ret);
		disable_irq(client->irq);
		return ret;
	}

	return 0;
}

static int ft5306de_stop(struct ft5306de_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;

	/* Disable interrupt generation */
	ret = ft5306de_int_control(ts, 0);
	if (ret < 0) {
		dev_err(&client->dev, "could not disable interrupt, %d\n",
			ret);
		return ret;
	}

	/* Disable interrupt handling */
	disable_irq(client->irq);
	ts->stopped = true;
	mb();
	wake_up(&ts->wait);

	return 0;
}

static int ft5306de_input_open(struct input_dev *dev)
{
	struct ft5306de_ts *ts = input_get_drvdata(dev);

	return ft5306de_start(ts);
}

static void ft5306de_input_close(struct input_dev *dev)
{
	struct ft5306de_ts *ts = input_get_drvdata(dev);

	ft5306de_stop(ts);

	return;
}

#ifdef CONFIG_PM_SLEEP
static int ft5306de_suspend(struct device *dev)
{
	/* TODO */

	return 0;
}

static int ft5306de_resume(struct device *dev)
{
	/* TODO */

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5306de_pm_ops,
			 ft5306de_suspend, ft5306de_resume);

#ifdef CONFIG_OF
/**
 *	Generate platform data from the device tree if possible
 */
static struct ft5306de_ts_platdata *ft5306de_parse_dt(struct device *dev)
{
	struct ft5306de_ts_platdata *pdata;
	struct device_node *np = dev->of_node;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	pdata->gpio_rst = of_get_named_gpio(np, "reset-gpios", 0);
	if (!gpio_is_valid(pdata->gpio_rst)) {
		pdata->gpio_rst = 0;
	}

	pdata->gpio_int = of_get_named_gpio(np, "interrupt-gpios", 0);
	if (!gpio_is_valid(pdata->gpio_int)) {
		dev_err(dev, "failed to get interrupt gpio\n");
		return ERR_PTR(-EINVAL);
	}

	pdata->gpio_wake = of_get_named_gpio(np, "wake-gpios", 0);
	if (!gpio_is_valid(pdata->gpio_wake)) {
		pdata->gpio_wake = 0;
	}

	if (of_property_read_u32(np, "x-size", &pdata->x_max)) {
		dev_err(dev, "failed to get x-size property\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(np, "y-size", &pdata->y_max)) {
		dev_err(dev, "failed to get y-size property\n");
		return ERR_PTR(-EINVAL);
	}

	return pdata;
}
#else
static struct ft5306de_ts_platdata *ft5306de_parse_dt(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}
#endif

static void ft5306de_reset(void *data)
{
	struct ft5306de_ts *ts = data;

	gpio_set_value(ts->pdata->gpio_rst, 0);
	mdelay(50);
	gpio_set_value(ts->pdata->gpio_rst, 1);
}

static int ft5306de_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	const struct ft5306de_ts_platdata *pdata;
	struct ft5306de_ts *ts;
	struct input_dev *input_dev;
	int error;

	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
		pdata = ft5306de_parse_dt(&client->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ts = devm_kzalloc(&client->dev,
			  sizeof(struct ft5306de_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "could not allocate input device\n");
		return -ENOMEM;
	}

	ts->pdata = pdata;
	ts->client = client;
	ts->input = input_dev;
	ts->stopped = true;
	init_waitqueue_head(&ts->wait);

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "FT5306DE touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->open = ft5306de_input_open;
	input_dev->close = ft5306de_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->y_max, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     pdata->y_max, 0, 0);

	input_set_drvdata(ts->input, ts);

	error = devm_gpio_request_one(&client->dev, pdata->gpio_int,
				      GPIOF_DIR_IN, "ft5306de_ts_int");
	if (error) {
		dev_err(&client->dev, "request of gpio %d failed, %d\n",
			pdata->gpio_int, error);
		return error;
	}

	error = devm_gpio_request_one(&client->dev, pdata->gpio_wake,
				      GPIOF_DIR_OUT, "ft5306de_ts_wake");
	if (error) {
		dev_err(&client->dev, "request of gpio %d failed, %d\n",
			pdata->gpio_wake, error);
		return error;
	}

	/* Enable controller */
	gpio_set_value(ts->pdata->gpio_wake, 1);

	/* Reset controller if reset pin is defined */
	if (pdata->gpio_rst) {
		error = devm_gpio_request_one(&client->dev, pdata->gpio_rst,
									  GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
								"ft5306de_ts_rst");
		if (error) {
			dev_err(&client->dev, "request of gpio %d failed, %d\n",
					pdata->gpio_rst, error);
			return error;
		}

		error = devm_add_action(&client->dev, ft5306de_reset, ts);
		if (error) {
			ft5306de_reset(ts);
			dev_err(&client->dev, "failed to register reset action, %d\n",
				error);
			return error;
		}

		msleep(200);
	}

	error = gpio_to_irq(pdata->gpio_int);
	if (error < 0) {
		dev_err(&client->dev, "could not get irq for gpio %d, %d\n",
				pdata->gpio_int, error);
		return error;
	}

	client->irq = error;

	/* Register a threaded IRQ handler */
	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, ft5306de_interrupt,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  input_dev->name, ts);
	if (error) {
		dev_err(&client->dev, "irq %d request failed, %d\n",
			client->irq, error);
		return error;
	}

	/* Stop device and disable interrupts until it is opened */
	error = ft5306de_stop(ts);
	if (error)
		return error;

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev, "could not register input device, %d\n",
			error);
		return error;
	}

	i2c_set_clientdata(client, ts);

	return 0;
}

static const struct i2c_device_id ft5306de_idtable[] = {
	{ "ft5306de_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5306de_idtable);

#ifdef CONFIG_OF
static struct of_device_id ft5306de_ts_dt_idtable[] = {
	{ .compatible = "ft,ft5306de_ts" },
	{},
};
MODULE_DEVICE_TABLE(of, ft5306de_ts_dt_idtable);
#endif

static struct i2c_driver ft5306de_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ft5306de_ts",
		.pm	= &ft5306de_pm_ops,
		.of_match_table	= of_match_ptr(ft5306de_ts_dt_idtable),
	},
	.probe		= ft5306de_probe,
	.id_table	= ft5306de_idtable,
};

module_i2c_driver(ft5306de_driver);

MODULE_DESCRIPTION("FT5306 touchscreen driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alere International Ltd <rob.voisey@alere.com>");
