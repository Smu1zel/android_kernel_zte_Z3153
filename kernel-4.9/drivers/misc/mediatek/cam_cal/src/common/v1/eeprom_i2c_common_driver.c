/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

/* Include platform define if necessary */
#ifdef EEPROM_PLATFORM_DEFINE
#include "eeprom_platform_def.h"
#endif

#if defined(HI846Z5156CC132_MIPI_RAW)
#define HI846_AF_OTP_ADDR 0x0CBA
#define HI846_AF_OTP_LEHGTH 0X0E
extern void hi846z5156cc132_otp_setting(void);
extern void hi846z5156cc132_otp_disable(void);
extern int hi846z5156cc132_otp_read_(u32 offset, u8 *data, u32 size);
DEFINE_MUTEX(HI846_OTP_MUTEX);
#endif
#if defined(HI846Z5157V_MIPI_RAW)
#define HI846_AF_OTP_ADDR 0x0CBA
#define HI846_AF_OTP_LEHGTH 0X0E
extern void hi846z5157v_otp_setting(void);
extern void hi846z5157v_otp_disable(void);
extern int hi846z5157v_otp_read_(u32 offset, u8 *data, u32 size);
DEFINE_MUTEX(HI846_OTP_MUTEX);
#endif


#if defined(GC8034Z5156CC132_MIPI_RAW) || defined(GC8034Z5157V_MIPI_RAW)
#define GC8034_AF_OTP_ADDR 0x00
#define GC8034_AF_OTP_LEHGTH 0X0E
extern void Gc8034_Send_AF_OTP(u8 *data, u32 size);
#endif

#if defined(HI846Z6250CC134_MIPI_RAW)
extern unsigned int hi846z6250cc134_moduleID;
//extern unsigned int hi846z6250cc134_otp_read_baseinfo(void);
//extern void hi846z6250cc134_otp_setting(void);
//extern void hi846z6250cc134_otp_disable(void);
#endif


static DEFINE_SPINLOCK(g_spinLock);


/************************************************************
 * I2C read function (Common)
 ************************************************************/
static struct i2c_client *g_pstI2CclientG;

/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2
#ifndef EEPROM_I2C_READ_MSG_LENGTH_MAX
#define EEPROM_I2C_READ_MSG_LENGTH_MAX 32
#endif

static int Read_I2C_CAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];


	if (ui4_length > EEPROM_I2C_READ_MSG_LENGTH_MAX) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 EEPROM_I2C_READ_MSG_LENGTH_MAX);
		return -1;
	}
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr =
		g_pstI2CclientG->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_spinLock);

	msg[0].addr = g_pstI2CclientG->addr;
	msg[0].flags = g_pstI2CclientG->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = puReadCmd;

	msg[1].addr = g_pstI2CclientG->addr;
	msg[1].flags = g_pstI2CclientG->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = ui4_length;
	msg[1].buf = a_puBuff;

	i4RetValue = i2c_transfer(g_pstI2CclientG->adapter,
				msg,
				EEPROM_I2C_MSG_SIZE_READ);

	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = g_pstI2CclientG->addr & I2C_MASK_FLAG;
	spin_unlock(&g_spinLock);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read data failed!!\n");
		return -1;
	}

	return 0;
}

int iReadData_CAM_CAL(unsigned int ui4_offset,
	unsigned int ui4_length, unsigned char *pinputdata)
{
	int i4RetValue = 0;
	int i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		if (i4ResidueDataLength >= EEPROM_I2C_READ_MSG_LENGTH_MAX) {
			i4RetValue = Read_I2C_CAM_CAL(
				(u16) u4CurrentOffset,
				EEPROM_I2C_READ_MSG_LENGTH_MAX, pBuff);
			if (i4RetValue != 0) {
				pr_debug("I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += EEPROM_I2C_READ_MSG_LENGTH_MAX;
			i4ResidueDataLength -= EEPROM_I2C_READ_MSG_LENGTH_MAX;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			i4RetValue =
			    Read_I2C_CAM_CAL(
			    (u16) u4CurrentOffset, i4ResidueDataLength, pBuff);
			if (i4RetValue != 0) {
				pr_debug("I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += i4ResidueDataLength;
			i4ResidueDataLength = 0;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);



	return 0;
}

unsigned int Common_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	g_pstI2CclientG = client;
	if (iReadData_CAM_CAL(addr, size, data) == 0)
		return size;
	else
		return 0;
}

#if defined(HI846Z6250CC134_MIPI_RAW)
//read hi846 wide-angle   moduleID
#define HI846_OTP_BASE 0x0201
unsigned int HI846_wide_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
#if defined(HI846Z6250CC134_MIPI_RAW)
	unsigned int module_id = 0;


	module_id = hi846z6250cc134_moduleID;
//	hi846z6250cc134_otp_setting();
//	module_id = hi846z6250cc134_otp_read_baseinfo();
//	hi846z6250cc134_otp_disable();
	pr_debug("[test] %s addr = 0x%x, size = %d, module_id = 0x%x\n",
		__func__, addr, size, module_id);

	data[0] = module_id;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	return size;
#endif
	return Common_read_region(client, addr, data, size);
}
#endif

#if defined(HI846Z5156CC132_MIPI_RAW) || defined(HI846Z5157V_MIPI_RAW)
unsigned int HI846_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
#if defined(HI846Z5156CC132_MIPI_RAW)
	int i;
	/*hi846 transform AF otp*/
	pr_debug("[test] %s addr = 0x%x, size = %d\n",
		__func__, addr, size);
	if((addr >= HI846_AF_OTP_ADDR) &&
		(addr <= HI846_AF_OTP_ADDR + HI846_AF_OTP_LEHGTH - 1)){
		mutex_lock(&HI846_OTP_MUTEX);
		hi846z5156cc132_otp_setting();
		hi846z5156cc132_otp_read_(addr, data, size);
		hi846z5156cc132_otp_disable();
		mutex_unlock(&HI846_OTP_MUTEX);
		for(i = 0; i < size ; i++){
			pr_debug("[test] %s data[%d] = 0x%0x\n",
				__func__, i, data[i]);
		}
		return 0;
	} else {
		return size;
	}
#endif

#if defined(HI846Z5157V_MIPI_RAW)
		int i;
		/*hi846 transform AF otp*/
		pr_debug("[test] %s addr = 0x%x, size = %d\n",
			__func__, addr, size);
		if((addr >= HI846_AF_OTP_ADDR) &&
			(addr <= HI846_AF_OTP_ADDR + HI846_AF_OTP_LEHGTH - 1)){
			mutex_lock(&HI846_OTP_MUTEX);
			hi846z5157v_otp_setting();
			hi846z5157v_otp_read_(addr, data, size);
			hi846z5157v_otp_disable();
			mutex_unlock(&HI846_OTP_MUTEX);
			for(i = 0; i < size ; i++){
				pr_debug("[test] %s data[%d] = 0x%0x\n",
					__func__, i, data[i]);
			}
			return 0;
		} else {
			return size;
		}
#endif


	return Common_read_region(client, addr, data, size);
}
#endif
#if defined(GC8034Z5156CC132_MIPI_RAW) || defined(GC8034Z5157V_MIPI_RAW)
unsigned int GC8034_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
#if defined(GC8034Z5156CC132_MIPI_RAW) || defined(GC8034Z5157V_MIPI_RAW)
	int i;
	pr_debug("[test] %s addr = 0x%x, size = %d\n",
		__func__, addr, size);
	if((addr >= GC8034_AF_OTP_ADDR) &&
		(addr <= GC8034_AF_OTP_LEHGTH + GC8034_AF_OTP_LEHGTH - 1)){
		Gc8034_Send_AF_OTP(data, size);
		for(i = 0; i < size ; i++){
			pr_debug("[test] %s data[%d] = 0x%0x\n",
				__func__, i, data[i]);
		}
		return 0;
	} else {
		return size;
	}
#endif

	return Common_read_region(client, addr, data, size);
}
#endif

