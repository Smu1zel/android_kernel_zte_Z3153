/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"


#ifndef AW3644_DTNAME_I2C
#define AW3644_DTNAME_I2C "mediatek,strobe_main"
#endif
/* define device tree */
/* TODO: modify temp device tree name */
#ifndef AW3644_DTNAME
#define AW3644_DTNAME "mediatek,flashlights_aw3644"
#endif
/* TODO: define driver name */
#define AW3644_NAME "flashlights-aw3644"




#define NOERR 0
#define ERR_RET 1

/* define registers */
#define AW3644_REG_SILICON_REVISION (0x0C)
#define AW3644_REG_ENABLE (0x01)
#define AW3644_REG_FLASH_LEVEL_LED1 (0x03)
#define AW3644_REG_FLASH_LEVEL_LED2 (0x04)
#define AW3644_REG_TORCH_LEVEL_LED1 (0x05)
#define AW3644_REG_TORCH_LEVEL_LED2 (0x06)
#define AW3644_REG_BOOST_CONFIG		0x07
#define AW3644_REG_TIMING_CONFIGURATION (0x08)

/*enable bit code*/
#define AW3644_ENABLE_STANDBY (0x00)
#define AW3644_ENABLE_TORCH (0x0B)
#define AW3644_ENABLE_FLASH (0x0F)
#define AW3644_FLASH_TIMEOUT_MAX (0x1F)  /*max timeout is 1600ms*/
/* define level */
#define AW3644_LEVEL_NUM 15
#define AW3644_HW_TIMEOUT 1600 /* ms */
#define AW3644_WAIT_TIME 3
/* define pinctrl */
#define AW3644_PINCTRL_PIN_HWEN 0
#define AW3644_PINCTRL_PINSTATE_LOW 0
#define AW3644_PINCTRL_PINSTATE_HIGH 1
#define AW3644_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define AW3644_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *aw3644_pinctrl;
static struct pinctrl_state *aw3644_hwen_high;
static struct pinctrl_state *aw3644_hwen_low;




/* define mutex and work queue */
static DEFINE_MUTEX(aw3644_mutex);
static struct work_struct aw3644_work;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *AW3644_i2c_client;

/* platform data */
struct aw3644_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* aw3644 chip data */
struct aw3644_chip_data {
	struct i2c_client *client;
	struct aw3644_platform_data *pdata;
	struct mutex lock;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw3644_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw3644_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3644_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3644_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	aw3644_hwen_high = pinctrl_lookup_state(aw3644_pinctrl, AW3644_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(aw3644_hwen_high)) {
		pr_err("Failed to init (%s)\n", AW3644_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(aw3644_hwen_high);
	}
	aw3644_hwen_low = pinctrl_lookup_state(aw3644_pinctrl, AW3644_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(aw3644_hwen_low)) {
		pr_err("Failed to init (%s)\n", AW3644_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(aw3644_hwen_low);
	}

	return ret;
}

static int aw3644_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(aw3644_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -ERR_RET;
	}

	switch (pin) {
	case AW3644_PINCTRL_PIN_HWEN:
		if (state == AW3644_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3644_hwen_low))
			pinctrl_select_state(aw3644_pinctrl, aw3644_hwen_low);
		else if (state == AW3644_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3644_hwen_high))
			pinctrl_select_state(aw3644_pinctrl, aw3644_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_err("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * aw3644 operations
 *****************************************************************************/
#define AW3644_LEVEL_FLASH 15

static const int aw3644_current[AW3644_LEVEL_NUM] = {
	23,   70,   93,  152,  225,  293,  316,  375,
	425,  475,  525,  575,  625,  675,  725
};

static const unsigned char ktd2684_flash_level[AW3644_LEVEL_FLASH] = {
	0x1,  0x4,  0x7,  0xa,  0xD, 0x10, 0x13, 0x16,
	0x19, 0x1c, 0x1f, 0x22, 0x25, 0x29, 0x2c
};



static const unsigned char aw3644_flash_level[AW3644_LEVEL_FLASH] = {
	0x1,  0x5,  0x9,  0xD,  0x12, 0x16, 0x1A, 0x1F,
	0x23, 0x27, 0x2B, 0x30, 0x34, 0x38, 0x3C
};

#define AW3644_LEVEL_TORCH 8
#define SY7806_LEVEL_TORCH 8

static const unsigned char aw3644_torch_level[AW3644_LEVEL_TORCH] = {
	0x1,  0x5,  0x9,  0xD,  0x12, 0x16, 0x1A, 0x1F,
};

static const unsigned char sy7806_torch_level[SY7806_LEVEL_TORCH] = {
	0x3,  0xB,  0x13,  0x1C,  0x26, 0x2E, 0x36, 0x41,
};



static int aw3644_level = -1;

static int aw3644_is_torch(int level)
{
	if (level >= AW3644_LEVEL_TORCH)
		return -ERR_RET;

	return NOERR;
}

static int aw3644_verify_level(int level)
{
	if (level < 0) {
		level = 0;
	} else if (level >= AW3644_LEVEL_NUM) {
		level = AW3644_LEVEL_NUM - 1;
	}

	return level;
}

/* i2c wrapper function */
static int aw3644_flash_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw3644_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}


static int aw3644_flash_read(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct aw3644_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}


/* flashlight enable function */
static int aw3644_enable(void)
{
	unsigned char reg;

	reg = AW3644_REG_ENABLE;
	if (!aw3644_is_torch(aw3644_level)) {
		/* torch mode */
		aw3644_flash_write(AW3644_i2c_client, reg, 0x0b);
	} else {
		/* flash mode */
		aw3644_flash_write(AW3644_i2c_client, reg, 0x0f);
	}
	return 0;
}

/* flashlight disable function */
static int aw3644_disable(void)
{
	unsigned char reg, val;

	reg = AW3644_REG_ENABLE;
	val = AW3644_ENABLE_STANDBY;

	return aw3644_flash_write(AW3644_i2c_client, reg, val);
}

static int aw3644_set_timeout_configuration(void)
{
	return aw3644_flash_write(AW3644_i2c_client,
		AW3644_REG_TIMING_CONFIGURATION,
		AW3644_FLASH_TIMEOUT_MAX);
}

static int is_ktd_flash = 0;
static int is_sy7806_flash = 0;


/* set flashlight level */
static int aw3644_set_level(int level)
{
	unsigned char val;

	level = aw3644_verify_level(level);
	aw3644_level = level;
	if (!aw3644_is_torch(aw3644_level)) {
	       /* torch mode */
	       if(is_sy7806_flash) {
	           val = sy7806_torch_level[level];
		       aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1, val);
		       aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2, val); 
	       } else {
		       val = aw3644_torch_level[level];
		       aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED1, val);
		       if(is_ktd_flash) {
				    val = val - 0x1;
	           }
		       aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TORCH_LEVEL_LED2, val); 
	      }
	} else {
		/* flash mode */
		val = aw3644_flash_level[level];
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED1, val);
		if(is_ktd_flash) {
			val = ktd2684_flash_level[level];
		}
		aw3644_flash_write(AW3644_i2c_client, AW3644_REG_FLASH_LEVEL_LED2, val);
		aw3644_set_timeout_configuration();
	}

	return 0;
}



#define SY7806_ADDR 0x67
#define KTD_DEV_ID 0x1
#define SY7806_DEV_ID 0x3

void verify_flash_i2c_addr(void)
{
	int reg_data = 0;
	reg_data = aw3644_flash_read(AW3644_i2c_client, 0x0);
	pr_debug("mtk_test: flash addr 0x0 reg_data = %d\n", reg_data);
	if(reg_data < 0) {
		pr_debug("mtk_test: set flash i2c 0x67\n");
		AW3644_i2c_client->addr = SY7806_ADDR;
		is_sy7806_flash = 1;
	}
	else {
		reg_data = aw3644_flash_read(AW3644_i2c_client, 0x0c);
		reg_data = reg_data >> 3;
		pr_debug("mtk_test: flash add1 0x0c = 0x%d\n", reg_data);
		if(reg_data == KTD_DEV_ID) {
			is_ktd_flash = 1;
		}
	}

	return;
}

/* flashlight init */
int aw3644_init(void)
{
	pr_info("%s\n", __func__);

	aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN,
		AW3644_PINCTRL_PINSTATE_HIGH);
	mdelay(AW3644_WAIT_TIME);
	verify_flash_i2c_addr();
	aw3644_flash_write(AW3644_i2c_client, AW3644_REG_ENABLE, 0x00);
	aw3644_flash_write(AW3644_i2c_client, AW3644_REG_BOOST_CONFIG, 0x09);
	aw3644_flash_write(AW3644_i2c_client, AW3644_REG_TIMING_CONFIGURATION, 0x1f);

	return 0;
}

/* flashlight uninit */
int aw3644_uninit(void)
{
	aw3644_disable();
	aw3644_pinctrl_set(AW3644_PINCTRL_PIN_HWEN, AW3644_PINCTRL_PINSTATE_LOW);

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3644_timer;
static unsigned int aw3644_timeout_ms;

static void aw3644_work_disable(struct work_struct *data)
{
	pr_err("work queue callback\n");
	aw3644_disable();
}

static enum hrtimer_restart aw3644_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3644_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3644_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_err("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3644_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_err("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3644_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_err("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw3644_timeout_ms) {
				s = aw3644_timeout_ms / 1000;
				ns = aw3644_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&aw3644_timer, ktime, HRTIMER_MODE_REL);
			}
			aw3644_enable();
		} else {
			aw3644_disable();
			hrtimer_cancel(&aw3644_timer);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_err("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = AW3644_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_err("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = AW3644_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = aw3644_verify_level(fl_arg->arg);
		pr_err("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = aw3644_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_err("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = AW3644_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3644_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3644_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int aw3644_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&aw3644_mutex);
	if (set) {
		if (!use_count)
			ret = aw3644_init();
		use_count++;
		pr_err("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw3644_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_err("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&aw3644_mutex);

	return ret;
}

static ssize_t aw3644_strobe_store(struct flashlight_arg arg)
{
	aw3644_set_driver(1);
	aw3644_set_level(arg.level);
	aw3644_timeout_ms = 0;
	aw3644_enable();
	msleep(arg.dur);
	aw3644_disable();
	aw3644_set_driver(0);

	return 0;
}

static struct flashlight_operations aw3644_ops = {
	aw3644_open,
	aw3644_release,
	aw3644_ioctl,
	aw3644_strobe_store,
	aw3644_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw3644_chip_init(struct aw3644_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw3644_init();
	 */

	return 0;
}

static int aw3644_parse_dt(struct device *dev,
		struct aw3644_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, AW3644_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw3644_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw3644_chip_data *chip;
	int err;

	pr_err("i2c probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw3644_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	i2c_set_clientdata(client, chip);
	AW3644_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	aw3644_chip_init(chip);

	pr_err("i2c probe done.\n");

	return 0;

err_out:
	return err;
}

static int aw3644_i2c_remove(struct i2c_client *client)
{
	struct aw3644_chip_data *chip = i2c_get_clientdata(client);

	pr_err("Remove start.\n");

	client->dev.platform_data = NULL;

	/* free resource */
	kfree(chip);

	pr_err("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw3644_i2c_id[] = {
	{AW3644_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, aw3644_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id aw3644_i2c_of_match[] = {
	{.compatible = AW3644_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, aw3644_i2c_of_match);
#endif

static struct i2c_driver aw3644_i2c_driver = {
	.driver = {
		.name = AW3644_NAME,
#ifdef CONFIG_OF
		.of_match_table = aw3644_i2c_of_match,
#endif
	},
	.probe = aw3644_i2c_probe,
	.remove = aw3644_i2c_remove,
	.id_table = aw3644_i2c_id,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw3644_probe(struct platform_device *pdev)
{
	struct aw3644_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct aw3644_chip_data *chip = NULL;
	int err;
	int i;

	pr_err("Probe start.\n");

	/* init pinctrl */
	if (aw3644_pinctrl_init(pdev)) {
		pr_err("Failed to init pinctrl.\n");
		return -ERR_RET;
	}

	if (i2c_add_driver(&aw3644_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -ERR_RET;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		pdev->dev.platform_data = pdata;
		err = aw3644_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err_free;
	}

	/* init work queue */
	INIT_WORK(&aw3644_work, aw3644_work_disable);

	/* init timer */
	hrtimer_init(&aw3644_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3644_timer.function = aw3644_timer_func;
	aw3644_timeout_ms = 1600;

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &aw3644_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(AW3644_NAME, &aw3644_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_err("Probe done.\n");

	return 0;
err_free:
	chip = i2c_get_clientdata(AW3644_i2c_client);
	i2c_set_clientdata(AW3644_i2c_client, NULL);
	kfree(chip);
	return err;
}

static int aw3644_remove(struct platform_device *pdev)
{
	struct aw3644_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_err("Remove start.\n");

	i2c_del_driver(&aw3644_i2c_driver);

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(AW3644_NAME);

	/* flush work queue */
	flush_work(&aw3644_work);

	pr_err("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3644_of_match[] = {
	{.compatible = AW3644_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3644_of_match);
#else
static struct platform_device aw3644_platform_device[] = {
	{
		.name = AW3644_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3644_platform_device);
#endif

static struct platform_driver aw3644_platform_driver = {
	.probe = aw3644_probe,
	.remove = aw3644_remove,
	.driver = {
		.name = AW3644_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3644_of_match,
#endif
	},
};

static int __init flashlight_aw3644_init(void)
{
	int ret;

	pr_err("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3644_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3644_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_err("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3644_exit(void)
{
	pr_err("Exit start.\n");

	platform_driver_unregister(&aw3644_platform_driver);

	pr_err("Exit done.\n");
}

module_init(flashlight_aw3644_init);
module_exit(flashlight_aw3644_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xi Chen <xixi.chen@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight AW3644 Driver");

