/*! \file sx932x.c
 * \brief  SX932x Driver
 *
 * Driver for the SX932x
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#define DRIVER_NAME "sx932x"
#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <situation.h>
#include "sx932x.h"

#define IDLE			0
#define ACTIVE			1

#define SX932x_ID_ERROR	    1
#define SX932x_NIRQ_ERROR	2
#define SX932x_CONN_ERROR	3
#define SX932x_I2C_ERROR	4

typedef struct sx932x {
	pbuttonInformation_t pbuttonInformation;
	psx932x_platform_data_t hw;
} sx932x_t,  *psx932x_t;

static int irq_gpio_num;
static int status_num;
static bool testcard_or_usercard = false;
static struct wake_lock irq_wakelock;
static psx93XX_t pthis = NULL;

static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;

	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c, buffer, 2);
#ifdef DEBUG
		dev_info(&i2c->dev, "write_register Address: 0x%x Value: 0x%x Return: %d\n",
						address, value, returnValue);
#endif
	}
	return returnValue;
}

static int read_register(psx93XX_t this,  u8 address,  u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c, address);

#ifdef DEBUG
		dev_info(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",
						address, returnValue);
#endif

		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

static int read_regStat(psx93XX_t this)
{
	u8 data = 0;

	if (this) {
		if (read_register(this, SX932x_IRQSTAT_REG, &data) == 0)
		dev_info(this->pdev,  "read_regStat %d\n", (data & 0x00FF));

		return (data & 0x00FF);
	}
	return 0;
}

static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this, SX932x_STAT2_REG, 0x0F);
	return returnValue;
}

static ssize_t manual_offset_calibration_show(struct device *dev,
								struct device_attribute *attr,  char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	dev_info(this->pdev,  "Reading IRQSTAT_REG\n");
	read_register(this, SX932x_IRQSTAT_REG, &reg_value);
	return snprintf(buf, PAGE_SIZE, "%d\n", reg_value);
}

static ssize_t manual_offset_calibration_store(struct device *dev,
			struct device_attribute *attr, const char *buf,  size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf,  0,  &val))
		return -EINVAL;
	if (val) {
		dev_info(this->pdev,  "Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}

static ssize_t identify_diff_card_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", testcard_or_usercard);
}

static ssize_t identify_diff_card(struct device *dev,
			struct device_attribute *attr, const char *buf,  size_t count)
{
	int val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf,  "%d",   &val) != 1) {
		pr_err("[SX932x]: %s -   number of data are wrong\n", __func__);
		return -EINVAL;
	}
	if (val == 1) {
		dev_info(this->pdev,  "Test card inserted!\n");
		testcard_or_usercard = true;
	} else if (val == 0) {
		dev_info(this->pdev,  "User card inserted!\n");
		testcard_or_usercard = false;
	}
	return count;
}

static int sx932x_Hardware_Check(psx93XX_t this)
{
	u8 loop = 0;

	this->failStatusCode = 0;

	while (this->get_nirq_low && this->get_nirq_low()) {
		read_regStat(this);
		mdelay(100);
		if (++loop > 10) {
			this->failStatusCode = SX932x_NIRQ_ERROR;
			break;
		}
	}

	dev_info(this->pdev, "sx932x status = 0x%x\n", this->failStatusCode);
	return (int)this->failStatusCode;
}

static int sx932x_global_variable_init(psx93XX_t this)
{
	this->irq_disabled = 0;
	this->failStatusCode = 0;
	this->reg_in_dts = true;
	return 0;
}

static ssize_t sx932x_register_write_store(struct device *dev,
			struct device_attribute *attr,  const char *buf,  size_t count)
{
	int reg_address = 0,  val = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	if (sscanf(buf,  "%x, %x",  &reg_address,  &val) != 2) {
		pr_err("[SX932x]: %s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}

	write_register(this,  (unsigned char)reg_address,  (unsigned char)val);
	pr_info("[SX932x]: %s - Register(0x%x) data(0x%x)\n", __func__,  reg_address,  val);

	return count;
}
/* read registers not include the advanced one */
static u8 register_read_addr;
static ssize_t sx932x_register_read_store(struct device *dev,
			struct device_attribute *attr,  const char *buf,  size_t count)
{
	int regist = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_info(this->pdev,  "Reading register\n");

	if (sscanf(buf,  "%x",  &regist) != 1) {
		pr_err("[SX932x]: %s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}
	register_read_addr = regist;

	return count;
}

static ssize_t sx932x_register_read_show(struct device *dev,
					struct device_attribute *attr,  char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	u8 val = 0;

	read_register(this, register_read_addr, &val);
	pr_info("[SX932x]: %s - Register(0x%x) data(0x%x)\n", __func__,  register_read_addr, val);
	return snprintf(buf, PAGE_SIZE, "Reg[0x%x]: 0x%x\n", register_read_addr, val);
}

static void read_rawData(psx93XX_t this)
{
	u8 msb = 0, lsb = 0, temp = 0;
	u8 csx;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;

	if (this) {
		read_register(this, SX932x_CPSRD, &temp);
		for (csx = 0; csx < 4; csx++) {
			/* here to check the CS1,  also can read other channel */
			write_register(this, SX932x_CPSRD, csx);
			read_register(this, SX932x_USEMSB, &msb);
			read_register(this, SX932x_USELSB, &lsb);
			useful = (s32)((msb << 8) | lsb);

			read_register(this, SX932x_AVGMSB, &msb);
			read_register(this, SX932x_AVGLSB, &lsb);
			average = (s32)((msb << 8) | lsb);

			read_register(this, SX932x_DIFFMSB, &msb);
			read_register(this, SX932x_DIFFLSB, &lsb);
			diff = (s32)((msb << 8) | lsb);

			read_register(this, SX932x_OFFSETMSB, &msb);
			read_register(this, SX932x_OFFSETLSB, &lsb);
			offset = (u16)((msb << 8) | lsb);
			if (useful > 32767)
				useful -= 65536;
			if (average > 32767)
				average -= 65536;
			if (diff > 32767)
				diff -= 65536;
			dev_info(this->pdev, "[CS: %d] Useful = %d Average = %d, DIFF = %d Offset = %d\n",
					csx, useful, average, diff, offset);
		}
		write_register(this, SX932x_CPSRD, temp);
	}
}

static ssize_t sx932x_raw_data_show(struct device *dev,
						struct device_attribute *attr,  char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);

	read_rawData(this);
	return 0;
}

static ssize_t sx932x_regval_show(struct device *dev,
						struct device_attribute *attr,  char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);

	unsigned char debug_reg[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
	0x10, 0x11, 0x14, 0x15,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x2A, 0x2B, 0x2C, 0x2D,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
	0x50, 0x51, 0x52, 0x53, 0x54,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A,
	0x9F, 0xFA, 0xFE
	};
	int debug_index = 0;
	unsigned char debug_reg_val = 0;
	int reg_count = sizeof(debug_reg);
	int return_size = 0;

	for (debug_index = 0; debug_index < reg_count; debug_index++) {
		read_register(this, debug_reg[debug_index], &debug_reg_val);
		return_size += snprintf(buf + return_size, 32,
				"Reg[0x%x]: 0x%x\n", debug_reg[debug_index], debug_reg_val);
	}

	return return_size;
}

static u8 diff_cs_num = 1;
static ssize_t diff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	u8 msb = 0,  lsb = 0, temp = 0;
	s32 diff = 0;
	s32 useful = 0;
	s32 average = 0;

	if ((diff_cs_num > 3) || (diff_cs_num < 0)) {
		dev_err(this->pdev, "cs_num %d error!\n", diff_cs_num);
		return snprintf(buf, 64, "cs_num %d error!Should between 0-3.", diff_cs_num);
	}

	if (this) {
		read_register(this, SX932x_CPSRD, &temp);
		/* here to check the CS1,if want check other CS ,can modify here*/
		write_register(this, SX932x_CPSRD, diff_cs_num);
		read_register(this, SX932x_USEMSB, &msb);
		read_register(this, SX932x_USELSB, &lsb);
		useful = (s32)((msb << 8) | lsb);

		read_register(this, SX932x_AVGMSB, &msb);
		read_register(this, SX932x_AVGLSB, &lsb);
		average = (s32)((msb << 8) | lsb);

		read_register(this, SX932x_DIFFMSB, &msb);
		read_register(this, SX932x_DIFFLSB, &lsb);
		diff = (s32)((msb << 8) | lsb);

		if (useful > 32767)
			useful -= 65536;
		if (average > 32767)
			average -= 65536;
		if (diff > 32767)
			diff -= 65536;
		dev_info(this->pdev,
			"[CS%d] Useful = %d Average = %d, DIFF = %d\n", diff_cs_num, useful, average, diff);

		write_register(this, SX932x_CPSRD, temp);
	}
		return snprintf(buf, 64, "[CS%d] %d,%d,%d\n", diff_cs_num, useful, average, diff);
}

static ssize_t diff_cs_store(struct device *dev,
			struct device_attribute *attr,  const char *buf,  size_t count)
{
	int regist = 0;

	if (sscanf(buf, "%d", &regist) != 1) {
		pr_err("[SX932x]: %s - The number of data are wrong\n", __func__);
		return -EINVAL;
	}
	diff_cs_num = regist;

	return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	psx93XX_t chip = dev_get_drvdata(dev);
	u8 touchStatus = 0;

	read_register(chip, SX932x_IRQ_ENABLE_REG, &touchStatus);
	if (touchStatus == 0)
		return snprintf(buf, 64, "0\n");

	if (status_num) {
		return snprintf(buf, 64, "1\n");
	} else {
		return snprintf(buf, 64, "0\n");
	}
}

static DEVICE_ATTR(status, 0440, status_show, NULL);
static DEVICE_ATTR(manual_calibrate,  0664,  manual_offset_calibration_show, manual_offset_calibration_store);
static DEVICE_ATTR(register_write,   0664,  NULL, sx932x_register_write_store);
static DEVICE_ATTR(register_read, 0664,  sx932x_register_read_show, sx932x_register_read_store);
static DEVICE_ATTR(regval, 0664, sx932x_regval_show, NULL);
static DEVICE_ATTR(raw_data, 0664, sx932x_raw_data_show, NULL);
static DEVICE_ATTR(card_type, 0664, identify_diff_card_show, identify_diff_card);
static DEVICE_ATTR(diff, 0664, diff_show, diff_cs_store);

static struct attribute *sx932x_attributes[] = {
	&dev_attr_manual_calibrate.attr,
	&dev_attr_register_write.attr,
	&dev_attr_register_read.attr,
	&dev_attr_regval.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_status.attr,
	&dev_attr_card_type.attr,
	&dev_attr_diff.attr,
	NULL,
};

static struct attribute_group sx932x_attr_group = {
	.attrs = sx932x_attributes,
};

static void sx932x_reg_init(psx93XX_t this)
{
	psx932x_t pDevice = 0;
	psx932x_platform_data_t pdata = 0;
	int i = 0;

	if (this) {
		pDevice = this->pDevice;
		pdata = pDevice->hw;

		if (this->reg_in_dts == true) {
			while (i < pdata->i2c_reg_num) {
				//dev_info(this->pdev,  "Going to Write Reg from dts: 0x%x Value: 0x%x\n",
				//pdata->pi2c_reg[i].reg, pdata->pi2c_reg[i].val);
				write_register(this, pdata->pi2c_reg[i].reg, pdata->pi2c_reg[i].val);
				i++;
			}
		} else {
			while (i < ARRAY_SIZE(sx932x_i2c_reg_setup)) {
				//dev_info(this->pdev,  "Going to Write Reg: 0x%x Value: 0x%x\n",
				//sx932x_i2c_reg_setup[i].reg, sx932x_i2c_reg_setup[i].val);
				write_register(this,  sx932x_i2c_reg_setup[i].reg, sx932x_i2c_reg_setup[i].val);
				i++;
			}
		}
	} else {
		dev_err(this->pdev,  "ERROR! platform data 0x%p\n", pDevice->hw);
	}

}

static int initialize(psx93XX_t this)
{
	int ret;

	if (this) {
		this->irq_disabled = 1;
		disable_irq(this->irq);

		write_register(this, SX932x_SOFTRESET_REG, SX932x_SOFTRESET);
		dev_info(this->pdev,  "Sent Software Reset. Waiting until device is back from reset to continue.\n");
		mdelay(50);

		ret = sx932x_global_variable_init(this);

		sx932x_reg_init(this);
		manual_offset_calibration(this);
		mdelay(100);

		enable_irq(this->irq);

		read_regStat(this);
		return 0;
	}
	return -ENOMEM;
}

static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u8 status = 0;
	psx932x_t pDevice = NULL;
	int numberOfButtons = 0;
	struct _buttonInfo *buttons = NULL, *pCurrentButton  = NULL;
	uint32_t report_data[3] = {0};

	if (this) {
		pDevice = this->pDevice;
		read_register(this, SX932x_STAT0_REG, &status);
		dev_info(this->pdev, "Enter %s. state_reg = %x\n", __func__, status);

		buttons = pDevice->pbuttonInformation->buttons;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;
		if (unlikely((buttons == NULL))) {
			dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton == NULL) {
				dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
				return;
			}
			switch (pCurrentButton->state) {
			case IDLE: /* Button is not being touched! */
				if (((status & pCurrentButton->mask) == pCurrentButton->mask)) {
					dev_info(this->pdev, "cap button %d touched\n", counter);
					report_data[0] = 1;
					report_data[1] = pCurrentButton->ph_num;
					sar_data_report(report_data);
					pCurrentButton->state = ACTIVE;
					status_num = 1;
				} else {
					dev_info(this->pdev, "Button %d already released.\n", counter);
				}
				break;
			case ACTIVE: /* Button is being touched! */
				if (((status & pCurrentButton->mask) != pCurrentButton->mask)) {
					dev_info(this->pdev, "cap button %d released\n", counter);
					report_data[0] = 0;
					report_data[1] = pCurrentButton->ph_num;
					sar_data_report(report_data);
					pCurrentButton->state = IDLE;
					status_num = 0;
				} else {
					dev_info(this->pdev, "Button %d still touched.\n", counter);
				}
				break;
			default:
				break;
			};
		}
	}
}

static int sx932x_parse_dt(struct sx932x_platform_data *pdata,  struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;

	if (dNode == NULL)
		return -ENODEV;

	pdata->irq_gpio = of_get_named_gpio_flags(dNode, "Semtech,nirq-gpio",  0,  &flags);
	irq_gpio_num = pdata->irq_gpio;
	if (pdata->irq_gpio < 0) {
		pr_err("[SENSOR]: %s - get irq_gpio error\n",  __func__);
		return -ENODEV;
	}

	of_property_read_u32(dNode, "used-ph-num", &pdata->used_ph_num);
	if ((pdata->used_ph_num < 0) || (pdata->used_ph_num > 15)) {
		pr_err("Invalid used-ph-num %d! Should be 0\1\2\3!\n", pdata->used_ph_num);
		pdata->used_ph_num = 0;
	}
	pr_info("[SX932x]:%s -  used-ph-num %d\n", __func__, pdata->used_ph_num);
	of_property_read_u32(dNode, "Semtech,reg-num", &pdata->i2c_reg_num);

	if (pdata->i2c_reg_num > 0) {
		 pdata->pi2c_reg = devm_kzalloc(dev, sizeof(struct smtc_reg_data)*pdata->i2c_reg_num,  GFP_KERNEL);
		 if (unlikely(pdata->pi2c_reg == NULL)) {
			return -ENOMEM;
		}

		if (of_property_read_u8_array(dNode, "Semtech,reg-init",
			(u8 *)&(pdata->pi2c_reg[0]), sizeof(struct smtc_reg_data)*pdata->i2c_reg_num))
		return -ENOMEM;
	}

	return 0;
}

static int sx932x_init_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx932x *pDevice = NULL;
	struct sx932x_platform_data *pdata = NULL;

	int rc = 0;

	if (this) {
		pDevice = this->pDevice;
		pdata = pDevice->hw;
		if (gpio_is_valid(pdata->irq_gpio)) {
			rc = gpio_request(pdata->irq_gpio,  "sx932x_irq_gpio");
			if (rc < 0) {
				dev_err(this->pdev,  "SX932x Request gpio. Fail![%d]\n",  rc);
				return rc;
			}
			rc = gpio_direction_input(pdata->irq_gpio);
			if (rc < 0) {
				dev_err(this->pdev,  "SX932x Set gpio direction. Fail![%d]\n",  rc);
				return rc;
			}
			this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
		} else {
			dev_err(this->pdev,  "SX932x Invalid irq gpio num.(init)\n");
		}
	} else {
		pr_err("[SX932x] : %s - Do not init platform HW",  __func__);
	}

	return rc;
}

static void sx932x_exit_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx932x *pDevice = NULL;
	struct sx932x_platform_data *pdata = NULL;

	if (this) {
		pDevice = this->pDevice;
		pdata = pDevice->hw;
		if (gpio_is_valid(pdata->irq_gpio)) {
			gpio_free(pdata->irq_gpio);
		} else {
			dev_err(this->pdev,  "Invalid irq gpio num.(exit)\n");
		}
	}
}

static int sx932x_get_nirq_state(void)
{
	return  !gpio_get_value(irq_gpio_num);
}

static void sx93XX_schedule_work(psx93XX_t this,  unsigned long delay)
{
	unsigned long flags;

	if (this) {
		dev_info(this->pdev,  "sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock, flags);

		cancel_delayed_work(&this->dworker);

		schedule_delayed_work(&this->dworker, delay);
		spin_unlock_irqrestore(&this->lock, flags);
	} else {
		dev_err(this->pdev, "sx93XX_schedule_work,  NULL psx93XX_t\n");
	}
}

static irqreturn_t sx93XX_irq(int irq,  void *pvoid)
{
	psx93XX_t this = 0;

	if (pvoid) {
		this = (psx93XX_t)pvoid;
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
		sx93XX_schedule_work(this, 0);
		} else{
			dev_err(this->pdev,  "sx93XX_irq - nirq read high\n");
		}
	} else{
		dev_err(this->pdev, "sx93XX_irq,  NULL pvoid\n");
	}
	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;

	wake_lock(&irq_wakelock);
	if (work) {
		this = container_of(work, sx93XX_t, dworker.work);

		if (!this) {
			dev_err(this->pdev, "sx93XX_worker_func,  NULL sx93XX_t\n");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low()) {
				nirqLow = 1;
			}
		}

		status = this->refreshStatus(this);
		counter = -1;
		dev_dbg(this->pdev,  "Worker - Refresh Status %d\n", status);

		while ((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
				dev_info(this->pdev,  "SX932x Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		/* Early models and if RATE=0 for newer models require a penup timer */
		if (unlikely(this->useIrqTimer && nirqLow)) {
			sx93XX_schedule_work(this, msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		dev_err(this->pdev, "sx93XX_worker_func,  NULL work_struct\n");
	}
	wake_unlock(&irq_wakelock);
}

static int sx93XX_remove(psx93XX_t this)
{
	if (this) {
		cancel_delayed_work_sync(&this->dworker);
		free_irq(this->irq,  this);
		kfree(this);
		return 0;
	}
	return -ENOMEM;
}

static void sx93XX_suspend(psx93XX_t this)
{
	if (this) {
		if (testcard_or_usercard == true) {
			enable_irq_wake(this->irq);
			dev_info(this->pdev, "sx93XX_suspend with test card!\n");
		} else {
			disable_irq(this->irq);
			dev_info(this->pdev, "sx93XX_suspend with user card or no card!\n");
		}
	}
}

static void sx93XX_resume(psx93XX_t this)
{
	if (this) {
		if (testcard_or_usercard == true) {
			sx93XX_schedule_work(this, 0);
			dev_info(this->pdev, "sx93XX_resume with test card!\n");
			disable_irq_wake(this->irq);
			} else {
			sx93XX_schedule_work(this, 0);
			dev_info(this->pdev, "sx93XX_resume with user card or no card!\n");
			enable_irq(this->irq);
		}
	}
}

static int sx93XX_IRQ_init(psx93XX_t this)
{
	int err = 0;

	if (this && this->pDevice) {
		spin_lock_init(&this->lock);
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
							this->pdev->driver->name,  this);
		if (err) {
			dev_err(this->pdev, "irq %d busy?\n", this->irq);
			return err;
		}
		dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
	}
	return -ENOMEM;
}

static int sar_open_report_data(int open)
{
	s32 returnValue = 0;
	u8 temp = 0;

	returnValue = read_register(pthis, SX932x_CTRL1_REG, &temp);

	if (open == 1) {
		returnValue = write_register(pthis, SX932x_CTRL1_REG, (temp | pthis->used_ph_num));
	} else if (open == 0) {
		returnValue = write_register(pthis, SX932x_CTRL1_REG, (temp & 0xF0));
	} else {
		dev_err(pthis->pdev, "irq %d busy?\n", pthis->irq);
		returnValue = -1;
		dev_info(pthis->pdev, "Wrong open param %d!\n", open);
	}

	return returnValue;
}

static int sar_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int sar_flush(void)
{
	situation_flush_report(ID_SAR);
	return 0;
}

static int sar_get_data(int *value, int *status)
{
	pr_err("%s not supported!\n", __func__);
	return 0;
}

extern int sensorlist_register_deviceinfo(int sensor, struct sensorInfo_NonHub_t *devinfo);

static int sx932x_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	int err = 0;
	u8 failcode = 0;
	psx93XX_t this = 0;
	psx932x_t pDevice = 0;
	psx932x_platform_data_t pplatData = 0;
	struct totalButtonInformation *pButtonInformationData = NULL;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	struct sensorInfo_NonHub_t devinfo = {"sx9325"};

	dev_info(&client->dev,  "sx932x_probe()\n");

	if (!i2c_check_functionality(adapter,  I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev,  "Check i2c functionality.Fail!\n");
		err = -EIO;
		return err;
	}

	this = devm_kzalloc(&client->dev, sizeof(sx93XX_t), GFP_KERNEL);
	if (this == NULL) {
		dev_err(&client->dev, "this is required!\n");
		return -EPERM;
	}

	pButtonInformationData = devm_kzalloc(&client->dev,  sizeof(struct totalButtonInformation),  GFP_KERNEL);
	if (!pButtonInformationData) {
		dev_err(&client->dev,  "Failed to allocate memory(totalButtonInformation)\n");
		err = -ENOMEM;
		return err;
	}

	pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
	pButtonInformationData->buttons =  psmtcButtons;

	pplatData = devm_kzalloc(&client->dev, sizeof(struct sx932x_platform_data), GFP_KERNEL);
	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	pplatData->get_is_nirq_low = sx932x_get_nirq_state;
	pplatData->pbuttonInformation = pButtonInformationData;
	pplatData->init_platform_hw = sx932x_init_platform_hw;
	pplatData->exit_platform_hw = sx932x_exit_platform_hw;
	client->dev.platform_data = pplatData;

	err = sx932x_parse_dt(pplatData,  &client->dev);
	if (err) {
		dev_err(&client->dev,  "could not setup pin\n");
		return -ENODEV;
	}
	/*wake up lock for irq process when suspend*/
	wake_lock_init(&irq_wakelock, WAKE_LOCK_SUSPEND, "IRQ_wakelock");

	this->init = initialize;
	this->refreshStatus = read_regStat;
	this->get_nirq_low = pplatData->get_is_nirq_low;
	this->used_ph_num = pplatData->used_ph_num;
	this->irq = client->irq;
	this->useIrqTimer = 0;

	if (MAX_NUM_STATUS_BITS >= 8) {
		this->statusFunc[0] = 0; /* TXEN_STAT */
		this->statusFunc[1] = 0; /* UNUSED */
		this->statusFunc[2] = 0; /* UNUSED */
		this->statusFunc[3] = read_rawData; /* CONV_STAT */
		this->statusFunc[4] = 0; /* COMP_STAT */
		this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
		this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
		this->statusFunc[7] = 0; /* RESET_STAT */
	}

	this->bus = client;
	i2c_set_clientdata(client, this);
	this->pdev = &client->dev;

	this->pDevice = pDevice = devm_kzalloc(&client->dev, sizeof(sx932x_t),  GFP_KERNEL);

	/* Check I2C Connection and chip ID */
	err = read_register(this,  SX932x_WHOAMI_REG,  &failcode);
	if (err < 0) {
		dev_info(this->pdev,  "sx932x failcode = 0x%x\n", SX932x_I2C_ERROR);
		return -EIO;
	}
	if (failcode != SX932x_WHOAMI_VALUE) {
		dev_info(this->pdev,  "sx932x failcode = 0x%x\n", SX932x_ID_ERROR);
		goto check_id_fail;
	}

	if (pDevice) {
		err = sysfs_create_group(&client->dev.kobj,  &sx932x_attr_group);

		pDevice->hw = pplatData;

		if (pplatData->init_platform_hw)
		    pplatData->init_platform_hw(client);

		pDevice->pbuttonInformation = pplatData->pbuttonInformation;
	}

	sx93XX_IRQ_init(this);

	if (this->init) {
		this->init(this);
	} else {
		dev_err(this->pdev, "No init function!!!!\n");
		return -ENOMEM;
	}

	sx932x_Hardware_Check(this); /* check NIRQ level */
	ctl.open_report_data = sar_open_report_data;
	ctl.batch = sar_batch;
	ctl.flush = sar_flush;
	ctl.is_support_wake_lock = true; // To be considered later
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_SAR);
	if (err) {
		pr_err("register sar contrl path err\n");
		return err;
	}

	data.get_data = sar_get_data;
	err = situation_register_data_path(&data, ID_SAR);
	if (err) {
		pr_err("register sar data path err\n");
		return err;
	}

	sensorlist_register_deviceinfo(ID_SAR, &devinfo);
	pthis = this;

	dev_info(&client->dev,  "sx932x_probe() Done\n");

	return 0;
check_id_fail:
	dev_info(&client->dev,  "sx932x_probe() Check ID fail!\n");
	wake_lock_destroy(&irq_wakelock);
	return -ENXIO;
}

static int sx932x_i2c_remove(struct i2c_client *client)
{
	psx932x_platform_data_t pplatData = 0;
	psx932x_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);

	if (this) {
		pDevice = this->pDevice;

		sysfs_remove_group(&client->dev.kobj, &sx932x_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw(client);
		kfree(this->pDevice);
	}
	return sx93XX_remove(this);
}

#if SUSPEND_SUPPORT
static int sx932x_suspend(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	sx93XX_suspend(this);
	return 0;
}
static int sx932x_resume(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	sx93XX_resume(this);
	return 0;
}
#else
#define sx932x_suspend		NULL
#define sx932x_resume		NULL
#endif

static struct i2c_device_id sx932x_idtable[] = {
	{DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sx932x_idtable);

static struct of_device_id sx932x_match_table[] = {
	{ .compatible = "Semtech,sx932x", },
	{ },
};

static const struct dev_pm_ops sx932x_pm_ops = {
	.suspend = sx932x_suspend,
	.resume = sx932x_resume,
};

static struct i2c_driver sx932x_i2c_driver = {
	.driver = {
		.owner			= THIS_MODULE,
		.name			= DRIVER_NAME,
		.of_match_table	= sx932x_match_table,
		.pm				= &sx932x_pm_ops,
	},
	.id_table = sx932x_idtable,
	.probe = sx932x_i2c_probe,
	.remove = sx932x_i2c_remove,
};

static int sar_sx932x_local_init(void)
{
	pr_err("%s enter\n", __func__);
    if (i2c_add_driver(&sx932x_i2c_driver)) {
        pr_err("add sx932x i2c driver error\n");
        return -EPERM;
    }
    return 0;
}

static int sar_sx932x_remove(void)
{
    i2c_del_driver(&sx932x_i2c_driver);
    return 0;
}

static struct situation_init_info sar_sx932x_init_info = {
    .name = "sar_sx932x",
    .init = sar_sx932x_local_init,
    .uninit = sar_sx932x_remove,
};

static int __init sar_sx932x_init(void)
{
    situation_driver_add(&sar_sx932x_init_info, ID_SAR);
    return 0;
}

static void __exit sar_sx932x_exit(void)
{
    pr_err("%s\n", __func__);
}

module_init(sar_sx932x_init);
module_exit(sar_sx932x_exit);

MODULE_DESCRIPTION("SX932x Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
