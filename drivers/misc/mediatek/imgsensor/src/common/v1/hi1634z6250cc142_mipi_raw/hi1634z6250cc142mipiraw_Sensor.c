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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <mt-plat/mtk_boot.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi1634z6250cc142mipiraw_Sensor.h"

#define PFX "hi1634z6250cc142mipiraw_Sensor"

/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_err(PFX "[%s] " format, __func__, ##args)
#define Hi1634_MaxGain 16

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI1634_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0xb42138aa,	/* checksum value for Camera Auto Test */

	.pre = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3755,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 339200000,
		.max_framerate = 300,
		},

	.cap = {
		.pclk = 80000000,
		.linelength = 710,		    //0x0206 0x02C6
		.framelength = 3755,	    //0x020E 0x0EAB
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 3456,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 678400000,
		.max_framerate = 300,
	},

	.normal_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3755,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 3456,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 678400000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3755,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 339200000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3755,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1152,
		.grabwindow_height = 864,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 169600000,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3755,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4224,
		.grabwindow_height = 3168,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 678400000,
		.max_framerate = 300,
	},

	.margin = 4,		/* sensor framelength & shutter margin check*/
	.min_shutter = 4,	/* min shutter */
	.min_gain = 64,
	.max_gain = 1024,
//	.min_gain_iso = 100,
//	.gain_step = 1,
//	.gain_type = 2,/*sony:0,OV:1,Samsung:2,Hynix:3,GC:4*/
	.max_frame_length = 0xffff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle,
					 * 2 frame with ispGain_delay-shut_delay=2-0=2
					 */
	.ae_sensor_gain_delay_frame = 0,	/* sensor gain delay frame for AE cycle,
						 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
						 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 6,	/* support sensor mode num ,don't support Slow motion */

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */
	.custom1_delay_frame = 3,	/* enter high speed video  delay frame num */
	.isp_driving_current = ISP_DRIVING_6MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_CSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,	/* 0,MIPI_SETTLEDELAY_AUTO;
					 * 1,MIPI_SETTLEDELAY_MANNUAL
					 */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x42, 0xff},	/* record sensor support all write id addr,
							 * only supprt 4must end with 0xff*/
};

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL, /* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,	/* IMGSENSOR_MODE enum value,
						 * record current sensor mode,such as:
						 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
						 */
	.shutter = 0x4C00,	/* current shutter */
	.gain = 0x200,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 30,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,	/* auto flicker enable:
					 * KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
					 */
	.test_pattern = KAL_FALSE,	/* test pattern mode or not.
					 * KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
					 */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x42,	/* record current sensor's i2c write id */
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ 4656, 3504,  0, 20, 4656, 3464,  2328, 1732, 12,  2, 2304, 1728, 0, 0, 2304, 1728},       // preview (2304 x 1728)

	{ 4656, 3504,  0, 22, 4656, 3460,  4656, 3460, 24,  2, 4608, 3456, 0, 0, 4608, 3456},       // capture (4608 x 3456)
	{ 4656, 3504,  0, 22, 4656, 3460,  4656, 3460, 24,  2, 4608, 3456, 0, 0, 4608, 3456},       // video (4608 x 3456)

	{ 4656, 3504,  0, 20, 4656, 3464,  2328, 1732, 12,  2, 2304, 1728, 0, 0, 2304, 1728},       // hs video (2304 x 1728)

	{ 4656, 3504,  0, 16, 4656, 3472,  1164,  868,  6,  2, 1152,  864, 0, 0, 1152,  864},      // slim video (1152 x 864)

	{ 4656, 3504,  0,166, 4656, 3172,  4656, 3172,216,  2, 4224, 3168, 0, 0, 4224, 3168},      //  custom1(4224 x 3168)
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 24,
	.i4OffsetY = 24,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 8,
	.i4SubBlkW = 16,
	.i4SubBlkH = 8,   /* need check xb.pang */
	.i4BlockNumX = 144,
	.i4BlockNumY = 108,
	.i4PosR = {{28, 25}, {44, 25}, {36, 37}, {52, 37}, {28, 41}, {44, 41}, {36, 53}, {52, 53},},
	.i4PosL = {{28, 29}, {44, 29}, {36, 33}, {52, 33}, {28, 45}, {44, 45}, {36, 49}, {52, 49},},
	.i4Crop = {{0, 0}, {24, 24}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},

	.iMirrorFlip = 0,
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
static void set_dummy(void)
{
	/* check */
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0206, imgsensor.line_length);
	write_cmos_sensor(0x020E, imgsensor.frame_length);

}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable? %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
		frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
				imgsensor.min_frame_length;
	}
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}

	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */

static int remosaic_flag = 0;
static int gain_compensat_flag = 0;



/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void write_shutter(kal_uint32 shutter)
{
	kal_uint32 realtime_fps = 0;



	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */



	LOG_INF("shutter %d line %d\n", shutter, __LINE__);
	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
	imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk * 10 /
			(imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x020E, imgsensor.frame_length);
		}
	} else {

		write_cmos_sensor(0x020E, imgsensor.frame_length);
	}



	write_cmos_sensor_8(0x020D, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x020A, shutter);
	LOG_INF("shutter =%d, framelength =%d",
		shutter, imgsensor.frame_length);
}	/*	write_shutter  */

static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	LOG_INF("set_shutter");

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */
static void set_shutter_frame_length(kal_uint32 shutter, kal_uint32 frame_length) //need change
{

	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter_frame_length! shutter =%d\n", shutter);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	/*if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;*/

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("shutter =%d, framelength =%d, min_frame_length=%d\n",
		shutter, imgsensor.frame_length, imgsensor.min_frame_length);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / (imgsensor.line_length * imgsensor.frame_length) * 10;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x020E, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
			write_cmos_sensor(0x020E, imgsensor.frame_length);
	}
	/* Update Shutter */
	write_cmos_sensor_8(0x020D, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x020A, shutter);

	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x100 */
	reg_gain = gain / 4 - 16;

	return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain_compensat_flag == 1) {
		gain = gain * 2;
	}
	if (gain < BASEGAIN)
		gain = BASEGAIN;
	else if (gain > 16 * BASEGAIN)
		gain = 16 * BASEGAIN;
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0212, reg_gain & 0x00FF);
	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function */
}				/*      night_mode      */

static void sensor_init(void)  //change
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0790, 0x0100);
	write_cmos_sensor(0x2000, 0x1001);
	write_cmos_sensor(0x2002, 0x0000);
	write_cmos_sensor(0x2006, 0x40B2);
	write_cmos_sensor(0x2008, 0xB032);
	write_cmos_sensor(0x200A, 0x8430);
	write_cmos_sensor(0x200C, 0x40B2);
	write_cmos_sensor(0x200E, 0xB07C);
	write_cmos_sensor(0x2010, 0x8476);
	write_cmos_sensor(0x2012, 0x40B2);
	write_cmos_sensor(0x2014, 0xB0EE);
	write_cmos_sensor(0x2016, 0x847A);
	write_cmos_sensor(0x2018, 0x40B2);
	write_cmos_sensor(0x201A, 0xB122);
	write_cmos_sensor(0x201C, 0x8434);
	write_cmos_sensor(0x201E, 0x40B2);
	write_cmos_sensor(0x2020, 0xB168);
	write_cmos_sensor(0x2022, 0x8488);
	write_cmos_sensor(0x2024, 0x40B2);
	write_cmos_sensor(0x2026, 0xB39A);
	write_cmos_sensor(0x2028, 0x871E);
	write_cmos_sensor(0x202A, 0x40B2);
	write_cmos_sensor(0x202C, 0xB314);
	write_cmos_sensor(0x202E, 0x86C8);
	write_cmos_sensor(0x2030, 0x4130);
	write_cmos_sensor(0x2032, 0x120B);
	write_cmos_sensor(0x2034, 0x120A);
	write_cmos_sensor(0x2036, 0x403B);
	write_cmos_sensor(0x2038, 0x0261);
	write_cmos_sensor(0x203A, 0x4B6A);
	write_cmos_sensor(0x203C, 0xC3EB);
	write_cmos_sensor(0x203E, 0x0000);
	write_cmos_sensor(0x2040, 0x1292);
	write_cmos_sensor(0x2042, 0xD000);
	write_cmos_sensor(0x2044, 0x4ACB);
	write_cmos_sensor(0x2046, 0x0000);
	write_cmos_sensor(0x2048, 0xB3EB);
	write_cmos_sensor(0x204A, 0x0000);
	write_cmos_sensor(0x204C, 0x2411);
	write_cmos_sensor(0x204E, 0x421F);
	write_cmos_sensor(0x2050, 0x85DA);
	write_cmos_sensor(0x2052, 0xD21F);
	write_cmos_sensor(0x2054, 0x85D8);
	write_cmos_sensor(0x2056, 0x930F);
	write_cmos_sensor(0x2058, 0x2404);
	write_cmos_sensor(0x205A, 0x40F2);
	write_cmos_sensor(0x205C, 0xFF80);
	write_cmos_sensor(0x205E, 0x0619);
	write_cmos_sensor(0x2060, 0x3C07);
	write_cmos_sensor(0x2062, 0x90F2);
	write_cmos_sensor(0x2064, 0x0011);
	write_cmos_sensor(0x2066, 0x0619);
	write_cmos_sensor(0x2068, 0x2803);
	write_cmos_sensor(0x206A, 0x50F2);
	write_cmos_sensor(0x206C, 0xFFF0);
	write_cmos_sensor(0x206E, 0x0619);
	write_cmos_sensor(0x2070, 0x40B2);
	write_cmos_sensor(0x2072, 0xB31A);
	write_cmos_sensor(0x2074, 0x86D0);
	write_cmos_sensor(0x2076, 0x413A);
	write_cmos_sensor(0x2078, 0x413B);
	write_cmos_sensor(0x207A, 0x4130);
	write_cmos_sensor(0x207C, 0x120B);
	write_cmos_sensor(0x207E, 0x120A);
	write_cmos_sensor(0x2080, 0x8231);
	write_cmos_sensor(0x2082, 0x430A);
	write_cmos_sensor(0x2084, 0x93C2);
	write_cmos_sensor(0x2086, 0x0C0A);
	write_cmos_sensor(0x2088, 0x2404);
	write_cmos_sensor(0x208A, 0xB3D2);
	write_cmos_sensor(0x208C, 0x0B05);
	write_cmos_sensor(0x208E, 0x2401);
	write_cmos_sensor(0x2090, 0x431A);
	write_cmos_sensor(0x2092, 0x403B);
	write_cmos_sensor(0x2094, 0x8438);
	write_cmos_sensor(0x2096, 0x422D);
	write_cmos_sensor(0x2098, 0x403E);
	write_cmos_sensor(0x209A, 0x192A);
	write_cmos_sensor(0x209C, 0x403F);
	write_cmos_sensor(0x209E, 0x86EC);
	write_cmos_sensor(0x20A0, 0x12AB);
	write_cmos_sensor(0x20A2, 0x422D);
	write_cmos_sensor(0x20A4, 0x403E);
	write_cmos_sensor(0x20A6, 0x86EC);
	write_cmos_sensor(0x20A8, 0x410F);
	write_cmos_sensor(0x20AA, 0x12AB);
	write_cmos_sensor(0x20AC, 0x930A);
	write_cmos_sensor(0x20AE, 0x2003);
	write_cmos_sensor(0x20B0, 0xD3D2);
	write_cmos_sensor(0x20B2, 0x1921);
	write_cmos_sensor(0x20B4, 0x3C09);
	write_cmos_sensor(0x20B6, 0x403D);
	write_cmos_sensor(0x20B8, 0x0200);
	write_cmos_sensor(0x20BA, 0x422E);
	write_cmos_sensor(0x20BC, 0x403F);
	write_cmos_sensor(0x20BE, 0x86EC);
	write_cmos_sensor(0x20C0, 0x1292);
	write_cmos_sensor(0x20C2, 0x8448);
	write_cmos_sensor(0x20C4, 0xC3D2);
	write_cmos_sensor(0x20C6, 0x1921);
	write_cmos_sensor(0x20C8, 0x1292);
	write_cmos_sensor(0x20CA, 0xD046);
	write_cmos_sensor(0x20CC, 0x403B);
	write_cmos_sensor(0x20CE, 0x8438);
	write_cmos_sensor(0x20D0, 0x422D);
	write_cmos_sensor(0x20D2, 0x410E);
	write_cmos_sensor(0x20D4, 0x403F);
	write_cmos_sensor(0x20D6, 0x86EC);
	write_cmos_sensor(0x20D8, 0x12AB);
	write_cmos_sensor(0x20DA, 0x422D);
	write_cmos_sensor(0x20DC, 0x403E);
	write_cmos_sensor(0x20DE, 0x86EC);
	write_cmos_sensor(0x20E0, 0x403F);
	write_cmos_sensor(0x20E2, 0x192A);
	write_cmos_sensor(0x20E4, 0x12AB);
	write_cmos_sensor(0x20E6, 0x5231);
	write_cmos_sensor(0x20E8, 0x413A);
	write_cmos_sensor(0x20EA, 0x413B);
	write_cmos_sensor(0x20EC, 0x4130);
	write_cmos_sensor(0x20EE, 0x4382);
	write_cmos_sensor(0x20F0, 0x052C);
	write_cmos_sensor(0x20F2, 0x4F0D);
	write_cmos_sensor(0x20F4, 0x930D);
	write_cmos_sensor(0x20F6, 0x3402);
	write_cmos_sensor(0x20F8, 0xE33D);
	write_cmos_sensor(0x20FA, 0x531D);
	write_cmos_sensor(0x20FC, 0xF03D);
	write_cmos_sensor(0x20FE, 0x07F0);
	write_cmos_sensor(0x2100, 0x4D0E);
	write_cmos_sensor(0x2102, 0xC312);
	write_cmos_sensor(0x2104, 0x100E);
	write_cmos_sensor(0x2106, 0x110E);
	write_cmos_sensor(0x2108, 0x110E);
	write_cmos_sensor(0x210A, 0x110E);
	write_cmos_sensor(0x210C, 0x930F);
	write_cmos_sensor(0x210E, 0x3803);
	write_cmos_sensor(0x2110, 0x4EC2);
	write_cmos_sensor(0x2112, 0x052C);
	write_cmos_sensor(0x2114, 0x3C04);
	write_cmos_sensor(0x2116, 0x4EC2);
	write_cmos_sensor(0x2118, 0x052D);
	write_cmos_sensor(0x211A, 0xE33D);
	write_cmos_sensor(0x211C, 0x531D);
	write_cmos_sensor(0x211E, 0x4D0F);
	write_cmos_sensor(0x2120, 0x4130);
	write_cmos_sensor(0x2122, 0x120B);
	write_cmos_sensor(0x2124, 0x425F);
	write_cmos_sensor(0x2126, 0x0205);
	write_cmos_sensor(0x2128, 0xC312);
	write_cmos_sensor(0x212A, 0x104F);
	write_cmos_sensor(0x212C, 0x114F);
	write_cmos_sensor(0x212E, 0x114F);
	write_cmos_sensor(0x2130, 0x114F);
	write_cmos_sensor(0x2132, 0x114F);
	write_cmos_sensor(0x2134, 0x114F);
	write_cmos_sensor(0x2136, 0x4F0B);
	write_cmos_sensor(0x2138, 0xF31B);
	write_cmos_sensor(0x213A, 0x5B0B);
	write_cmos_sensor(0x213C, 0x5B0B);
	write_cmos_sensor(0x213E, 0x5B0B);
	write_cmos_sensor(0x2140, 0x503B);
	write_cmos_sensor(0x2142, 0xD1CC);
	write_cmos_sensor(0x2144, 0x1292);
	write_cmos_sensor(0x2146, 0xD004);
	write_cmos_sensor(0x2148, 0x93C2);
	write_cmos_sensor(0x214A, 0x86BF);
	write_cmos_sensor(0x214C, 0x240B);
	write_cmos_sensor(0x214E, 0xB2E2);
	write_cmos_sensor(0x2150, 0x0400);
	write_cmos_sensor(0x2152, 0x2008);
	write_cmos_sensor(0x2154, 0x425F);
	write_cmos_sensor(0x2156, 0x86BB);
	write_cmos_sensor(0x2158, 0xD36F);
	write_cmos_sensor(0x215A, 0xF37F);
	write_cmos_sensor(0x215C, 0x5F0F);
	write_cmos_sensor(0x215E, 0x5F0B);
	write_cmos_sensor(0x2160, 0x4BA2);
	write_cmos_sensor(0x2162, 0x0402);
	write_cmos_sensor(0x2164, 0x413B);
	write_cmos_sensor(0x2166, 0x4130);
	write_cmos_sensor(0x2168, 0x8231);
	write_cmos_sensor(0x216A, 0xD3D2);
	write_cmos_sensor(0x216C, 0x7A12);
	write_cmos_sensor(0x216E, 0xC3D2);
	write_cmos_sensor(0x2170, 0x0F00);
	write_cmos_sensor(0x2172, 0x422D);
	write_cmos_sensor(0x2174, 0x403E);
	write_cmos_sensor(0x2176, 0x06D6);
	write_cmos_sensor(0x2178, 0x410F);
	write_cmos_sensor(0x217A, 0x1292);
	write_cmos_sensor(0x217C, 0x8438);
	write_cmos_sensor(0x217E, 0x93C2);
	write_cmos_sensor(0x2180, 0x86C1);
	write_cmos_sensor(0x2182, 0x2430);
	write_cmos_sensor(0x2184, 0x0B00);
	write_cmos_sensor(0x2186, 0x7304);
	write_cmos_sensor(0x2188, 0x0000);
	write_cmos_sensor(0x218A, 0x0800);
	write_cmos_sensor(0x218C, 0x7A10);
	write_cmos_sensor(0x218E, 0x421F);
	write_cmos_sensor(0x2190, 0x7100);
	write_cmos_sensor(0x2192, 0xF03F);
	write_cmos_sensor(0x2194, 0x0003);
	write_cmos_sensor(0x2196, 0x931F);
	write_cmos_sensor(0x2198, 0x241E);
	write_cmos_sensor(0x219A, 0x931F);
	write_cmos_sensor(0x219C, 0x2815);
	write_cmos_sensor(0x219E, 0x932F);
	write_cmos_sensor(0x21A0, 0x240D);
	write_cmos_sensor(0x21A2, 0x903F);
	write_cmos_sensor(0x21A4, 0x0003);
	write_cmos_sensor(0x21A6, 0x2404);
	write_cmos_sensor(0x21A8, 0x9382);
	write_cmos_sensor(0x21AA, 0x7112);
	write_cmos_sensor(0x21AC, 0x27EB);
	write_cmos_sensor(0x21AE, 0x3C1C);
	write_cmos_sensor(0x21B0, 0x41A2);
	write_cmos_sensor(0x21B2, 0x06D6);
	write_cmos_sensor(0x21B4, 0x4192);
	write_cmos_sensor(0x21B6, 0x0002);
	write_cmos_sensor(0x21B8, 0x06D8);
	write_cmos_sensor(0x21BA, 0x3FF6);
	write_cmos_sensor(0x21BC, 0x4192);
	write_cmos_sensor(0x21BE, 0x0002);
	write_cmos_sensor(0x21C0, 0x06DA);
	write_cmos_sensor(0x21C2, 0x41A2);
	write_cmos_sensor(0x21C4, 0x06DC);
	write_cmos_sensor(0x21C6, 0x3FF0);
	write_cmos_sensor(0x21C8, 0x4192);
	write_cmos_sensor(0x21CA, 0x0004);
	write_cmos_sensor(0x21CC, 0x06DA);
	write_cmos_sensor(0x21CE, 0x4192);
	write_cmos_sensor(0x21D0, 0x0006);
	write_cmos_sensor(0x21D2, 0x06DC);
	write_cmos_sensor(0x21D4, 0x3FE9);
	write_cmos_sensor(0x21D6, 0x4192);
	write_cmos_sensor(0x21D8, 0x0006);
	write_cmos_sensor(0x21DA, 0x06D6);
	write_cmos_sensor(0x21DC, 0x4192);
	write_cmos_sensor(0x21DE, 0x0004);
	write_cmos_sensor(0x21E0, 0x06D8);
	write_cmos_sensor(0x21E2, 0x3FE2);
	write_cmos_sensor(0x21E4, 0x1292);
	write_cmos_sensor(0x21E6, 0xD058);
	write_cmos_sensor(0x21E8, 0x5231);
	write_cmos_sensor(0x21EA, 0x4130);
	write_cmos_sensor(0x21EC, 0x7400);
	write_cmos_sensor(0x21EE, 0x8058);
	write_cmos_sensor(0x21F0, 0x1807);
	write_cmos_sensor(0x21F2, 0x00E0);
	write_cmos_sensor(0x21F4, 0x7002);
	write_cmos_sensor(0x21F6, 0x17C7);
	write_cmos_sensor(0x21F8, 0x7000);
	write_cmos_sensor(0x21FA, 0x1305);
	write_cmos_sensor(0x21FC, 0x0006);
	write_cmos_sensor(0x21FE, 0x001F);
	write_cmos_sensor(0x2200, 0x0055);
	write_cmos_sensor(0x2202, 0x00DB);
	write_cmos_sensor(0x2204, 0x0012);
	write_cmos_sensor(0x2206, 0x1754);
	write_cmos_sensor(0x2208, 0x206F);
	write_cmos_sensor(0x220A, 0x009E);
	write_cmos_sensor(0x220C, 0x00DD);
	write_cmos_sensor(0x220E, 0x5023);
	write_cmos_sensor(0x2210, 0x00DE);
	write_cmos_sensor(0x2212, 0x005B);
	write_cmos_sensor(0x2214, 0x0119);
	write_cmos_sensor(0x2216, 0x0390);
	write_cmos_sensor(0x2218, 0x00D1);
	write_cmos_sensor(0x221A, 0x0055);
	write_cmos_sensor(0x221C, 0x0040);
	write_cmos_sensor(0x221E, 0x0553);
	write_cmos_sensor(0x2220, 0x0456);
	write_cmos_sensor(0x2222, 0x5041);
	write_cmos_sensor(0x2224, 0x700D);
	write_cmos_sensor(0x2226, 0x2F99);
	write_cmos_sensor(0x2228, 0x2318);
	write_cmos_sensor(0x222A, 0x005C);
	write_cmos_sensor(0x222C, 0x7000);
	write_cmos_sensor(0x222E, 0x1586);
	write_cmos_sensor(0x2230, 0x0001);
	write_cmos_sensor(0x2232, 0x2032);
	write_cmos_sensor(0x2234, 0x0012);
	write_cmos_sensor(0x2236, 0x0008);
	write_cmos_sensor(0x2238, 0x0343);
	write_cmos_sensor(0x223A, 0x0148);
	write_cmos_sensor(0x223C, 0x2123);
	write_cmos_sensor(0x223E, 0x0046);
	write_cmos_sensor(0x2240, 0x05DD);
	write_cmos_sensor(0x2242, 0x00DE);
	write_cmos_sensor(0x2244, 0x00DD);
	write_cmos_sensor(0x2246, 0x00DC);
	write_cmos_sensor(0x2248, 0x00DE);
	write_cmos_sensor(0x224A, 0x07D6);
	write_cmos_sensor(0x224C, 0x5061);
	write_cmos_sensor(0x224E, 0x704F);
	write_cmos_sensor(0x2250, 0x2F99);
	write_cmos_sensor(0x2252, 0x005C);
	write_cmos_sensor(0x2254, 0x5080);
	write_cmos_sensor(0x2256, 0x4D90);
	write_cmos_sensor(0x2258, 0x50A1);
	write_cmos_sensor(0x225A, 0x2122);
	write_cmos_sensor(0x225C, 0x7800);
	write_cmos_sensor(0x225E, 0xC08C);
	write_cmos_sensor(0x2260, 0x0001);
	write_cmos_sensor(0x2262, 0x9038);
	write_cmos_sensor(0x2264, 0x59F7);
	write_cmos_sensor(0x2266, 0x903B);
	write_cmos_sensor(0x2268, 0x121C);
	write_cmos_sensor(0x226A, 0x9034);
	write_cmos_sensor(0x226C, 0x1218);
	write_cmos_sensor(0x226E, 0x8C34);
	write_cmos_sensor(0x2270, 0x0180);
	write_cmos_sensor(0x2272, 0x8DC0);
	write_cmos_sensor(0x2274, 0x01C0);
	write_cmos_sensor(0x2276, 0x7400);
	write_cmos_sensor(0x2278, 0x8058);
	write_cmos_sensor(0x227A, 0x1807);
	write_cmos_sensor(0x227C, 0x00E0);
	write_cmos_sensor(0x227E, 0x00DF);
	write_cmos_sensor(0x2280, 0x0047);
	write_cmos_sensor(0x2282, 0x7000);
	write_cmos_sensor(0x2284, 0x17C5);
	write_cmos_sensor(0x2286, 0x0046);
	write_cmos_sensor(0x2288, 0x0095);
	write_cmos_sensor(0x228A, 0x7000);
	write_cmos_sensor(0x228C, 0x148C);
	write_cmos_sensor(0x228E, 0x005B);
	write_cmos_sensor(0x2290, 0x0014);
	write_cmos_sensor(0x2292, 0x001D);
	write_cmos_sensor(0x2294, 0x216F);
	write_cmos_sensor(0x2296, 0x005E);
	write_cmos_sensor(0x2298, 0x00DD);
	write_cmos_sensor(0x229A, 0x2244);
	write_cmos_sensor(0x229C, 0x001C);
	write_cmos_sensor(0x229E, 0x00DE);
	write_cmos_sensor(0x22A0, 0x005B);
	write_cmos_sensor(0x22A2, 0x0519);
	write_cmos_sensor(0x22A4, 0x0150);
	write_cmos_sensor(0x22A6, 0x0091);
	write_cmos_sensor(0x22A8, 0x00D5);
	write_cmos_sensor(0x22AA, 0x0040);
	write_cmos_sensor(0x22AC, 0x0393);
	write_cmos_sensor(0x22AE, 0x0356);
	write_cmos_sensor(0x22B0, 0x5021);
	write_cmos_sensor(0x22B2, 0x700D);
	write_cmos_sensor(0x22B4, 0x2F99);
	write_cmos_sensor(0x22B6, 0x2318);
	write_cmos_sensor(0x22B8, 0x005C);
	write_cmos_sensor(0x22BA, 0x0006);
	write_cmos_sensor(0x22BC, 0x0016);
	write_cmos_sensor(0x22BE, 0x425A);
	write_cmos_sensor(0x22C0, 0x0012);
	write_cmos_sensor(0x22C2, 0x0008);
	write_cmos_sensor(0x22C4, 0x0403);
	write_cmos_sensor(0x22C6, 0x01C8);
	write_cmos_sensor(0x22C8, 0x2123);
	write_cmos_sensor(0x22CA, 0x0046);
	write_cmos_sensor(0x22CC, 0x095D);
	write_cmos_sensor(0x22CE, 0x00DE);
	write_cmos_sensor(0x22D0, 0x00DD);
	write_cmos_sensor(0x22D2, 0x00DC);
	write_cmos_sensor(0x22D4, 0x00DE);
	write_cmos_sensor(0x22D6, 0x04D6);
	write_cmos_sensor(0x22D8, 0x5041);
	write_cmos_sensor(0x22DA, 0x704F);
	write_cmos_sensor(0x22DC, 0x2F99);
	write_cmos_sensor(0x22DE, 0x7000);
	write_cmos_sensor(0x22E0, 0x1702);
	write_cmos_sensor(0x22E2, 0x202C);
	write_cmos_sensor(0x22E4, 0x0016);
	write_cmos_sensor(0x22E6, 0x5060);
	write_cmos_sensor(0x22E8, 0x2122);
	write_cmos_sensor(0x22EA, 0x7800);
	write_cmos_sensor(0x22EC, 0xC08C);
	write_cmos_sensor(0x22EE, 0x0001);
	write_cmos_sensor(0x22F0, 0x903B);
	write_cmos_sensor(0x22F2, 0x121C);
	write_cmos_sensor(0x22F4, 0x9034);
	write_cmos_sensor(0x22F6, 0x1218);
	write_cmos_sensor(0x22F8, 0x8DC0);
	write_cmos_sensor(0x22FA, 0x01C0);
	write_cmos_sensor(0x22FC, 0x0000);
	write_cmos_sensor(0x22FE, 0xB1EC);
	write_cmos_sensor(0x2300, 0x0000);
	write_cmos_sensor(0x2302, 0xB1EC);
	write_cmos_sensor(0x2304, 0xB25E);
	write_cmos_sensor(0x2306, 0x0002);
	write_cmos_sensor(0x2308, 0x0000);
	write_cmos_sensor(0x230A, 0xB276);
	write_cmos_sensor(0x230C, 0x0000);
	write_cmos_sensor(0x230E, 0xB276);
	write_cmos_sensor(0x2310, 0xB2EC);
	write_cmos_sensor(0x2312, 0x0002);
	write_cmos_sensor(0x2314, 0xB2FC);
	write_cmos_sensor(0x2316, 0xB308);
	write_cmos_sensor(0x2318, 0xFCE0);
	write_cmos_sensor(0x231A, 0x0040);
	write_cmos_sensor(0x231C, 0x0040);
	write_cmos_sensor(0x231E, 0x0040);
	write_cmos_sensor(0x2320, 0x0040);
	write_cmos_sensor(0x2322, 0x0040);
	write_cmos_sensor(0x2324, 0x0042);
	write_cmos_sensor(0x2326, 0x005A);
	write_cmos_sensor(0x2328, 0x005B);
	write_cmos_sensor(0x232A, 0x005C);
	write_cmos_sensor(0x232C, 0x005E);
	write_cmos_sensor(0x232E, 0x0060);
	write_cmos_sensor(0x2330, 0x0064);
	write_cmos_sensor(0x2332, 0x0066);
	write_cmos_sensor(0x2334, 0x006B);
	write_cmos_sensor(0x2336, 0x006F);
	write_cmos_sensor(0x2338, 0x0073);
	write_cmos_sensor(0x233A, 0x0077);
	write_cmos_sensor(0x233C, 0x007B);
	write_cmos_sensor(0x233E, 0x0081);
	write_cmos_sensor(0x2340, 0x0085);
	write_cmos_sensor(0x2342, 0x0089);
	write_cmos_sensor(0x2344, 0x008D);
	write_cmos_sensor(0x2346, 0x0091);
	write_cmos_sensor(0x2348, 0x0095);
	write_cmos_sensor(0x234A, 0x0099);
	write_cmos_sensor(0x234C, 0x009D);
	write_cmos_sensor(0x234E, 0x00A7);
	write_cmos_sensor(0x2350, 0x00AA);
	write_cmos_sensor(0x2352, 0x00B2);
	write_cmos_sensor(0x2354, 0x00B6);
	write_cmos_sensor(0x2356, 0x00B9);
	write_cmos_sensor(0x2358, 0x00B9);
	write_cmos_sensor(0x235A, 0x0040);
	write_cmos_sensor(0x235C, 0x0040);
	write_cmos_sensor(0x235E, 0x0040);
	write_cmos_sensor(0x2360, 0x0040);
	write_cmos_sensor(0x2362, 0x0040);
	write_cmos_sensor(0x2364, 0x0040);
	write_cmos_sensor(0x2366, 0x0054);
	write_cmos_sensor(0x2368, 0x0056);
	write_cmos_sensor(0x236A, 0x005A);
	write_cmos_sensor(0x236C, 0x005D);
	write_cmos_sensor(0x236E, 0x0062);
	write_cmos_sensor(0x2370, 0x0066);
	write_cmos_sensor(0x2372, 0x006B);
	write_cmos_sensor(0x2374, 0x0074);
	write_cmos_sensor(0x2376, 0x007B);
	write_cmos_sensor(0x2378, 0x007E);
	write_cmos_sensor(0x237A, 0x0085);
	write_cmos_sensor(0x237C, 0x008C);
	write_cmos_sensor(0x237E, 0x0095);
	write_cmos_sensor(0x2380, 0x009B);
	write_cmos_sensor(0x2382, 0x00A2);
	write_cmos_sensor(0x2384, 0x00A8);
	write_cmos_sensor(0x2386, 0x00B0);
	write_cmos_sensor(0x2388, 0x00B4);
	write_cmos_sensor(0x238A, 0x00BE);
	write_cmos_sensor(0x238C, 0x00C9);
	write_cmos_sensor(0x238E, 0x00D0);
	write_cmos_sensor(0x2390, 0x00D8);
	write_cmos_sensor(0x2392, 0x00DD);
	write_cmos_sensor(0x2394, 0x00EA);
	write_cmos_sensor(0x2396, 0x00ED);
	write_cmos_sensor(0x2398, 0x00F2);
	write_cmos_sensor(0x239A, 0x03FE);
	write_cmos_sensor(0x239C, 0x0000);
	write_cmos_sensor(0x239E, 0x0000);
	write_cmos_sensor(0x23A0, 0x0000);
	write_cmos_sensor(0x23A2, 0x0000);
	write_cmos_sensor(0x23A4, 0x03FE);
	write_cmos_sensor(0x23A6, 0x0000);
	write_cmos_sensor(0x23A8, 0x0000);
	write_cmos_sensor(0x23AA, 0x0000);
	write_cmos_sensor(0x23AC, 0x0000);
	write_cmos_sensor(0x23AE, 0x03FE);
	write_cmos_sensor(0x23B0, 0x0000);
	write_cmos_sensor(0x23B2, 0x0000);
	write_cmos_sensor(0x23B4, 0x0000);
	write_cmos_sensor(0x23B6, 0x0000);
	write_cmos_sensor(0x23B8, 0x03FE);
	write_cmos_sensor(0x23BA, 0x0000);
	write_cmos_sensor(0x23BC, 0x0000);
	write_cmos_sensor(0x23BE, 0x0000);
	write_cmos_sensor(0x23C0, 0x0000);
	write_cmos_sensor(0x0262, 0x0600);
	write_cmos_sensor(0x026A, 0xFFFF);
	write_cmos_sensor(0x026C, 0x00FF);
	write_cmos_sensor(0x026E, 0x0000);
	write_cmos_sensor(0x0360, 0x0E8E);
	write_cmos_sensor(0x0400, 0x0A10);
	write_cmos_sensor(0x040C, 0x01EB);
	write_cmos_sensor(0x0600, 0x1112);
	write_cmos_sensor(0x0602, 0x3112);
	write_cmos_sensor(0x0604, 0x8008);
	write_cmos_sensor(0x0644, 0x07FE);
	write_cmos_sensor(0x0676, 0x07FF);
	write_cmos_sensor(0x0678, 0x0002);
	write_cmos_sensor(0x06A8, 0x0240);
	write_cmos_sensor(0x06AA, 0x00CA);
	write_cmos_sensor(0x06AC, 0x0041);
	write_cmos_sensor(0x06AE, 0x03FC);
	write_cmos_sensor(0x06B4, 0x3FFF);
	write_cmos_sensor(0x06E2, 0xFF00);
	write_cmos_sensor(0x052A, 0x0000);
	write_cmos_sensor(0x052C, 0x0000);
	write_cmos_sensor(0x0F06, 0x0002);
	write_cmos_sensor(0x1102, 0x0008);
	write_cmos_sensor(0x1106, 0x0124);
	write_cmos_sensor(0x11C2, 0x0400);
	write_cmos_sensor(0x0902, 0x0003);
	write_cmos_sensor(0x0904, 0x0003);
	write_cmos_sensor(0x0912, 0x0303);
	write_cmos_sensor(0x0914, 0x0300);
	write_cmos_sensor(0x0A04, 0xB4C5);
	write_cmos_sensor(0x0A06, 0xC400);
	write_cmos_sensor(0x0A08, 0xA881);
	write_cmos_sensor(0x0A0E, 0xFEC0);
	write_cmos_sensor(0x0A12, 0x0000);
	write_cmos_sensor(0x0A18, 0x0010);
	write_cmos_sensor(0x0A1E, 0x0013);
	write_cmos_sensor(0x0A20, 0x0015);
	write_cmos_sensor(0x0C00, 0x0021);
	write_cmos_sensor(0x0C16, 0x0002);
	write_cmos_sensor(0x0708, 0x6F80);
	write_cmos_sensor(0x070C, 0x0000);
	write_cmos_sensor(0x0780, 0x010E);
	write_cmos_sensor(0x1202, 0x1E00);
	write_cmos_sensor(0x1204, 0xD700);
	write_cmos_sensor(0x1210, 0x8028);
	write_cmos_sensor(0x1216, 0xA0A0);
	write_cmos_sensor(0x1218, 0x00A0);
	write_cmos_sensor(0x121A, 0x0000);
	write_cmos_sensor(0x121C, 0x4128);
	write_cmos_sensor(0x121E, 0x0000);
	write_cmos_sensor(0x1220, 0x0000);
	write_cmos_sensor(0x1222, 0x28FA);
	write_cmos_sensor(0x100C, 0xB000);
	write_cmos_sensor(0x105C, 0x0F0B);
	write_cmos_sensor(0x1960, 0x03FE);
	write_cmos_sensor(0x196A, 0x03FE);
	write_cmos_sensor(0x1974, 0x03FE);
	write_cmos_sensor(0x197E, 0x03FE);
	write_cmos_sensor(0x1986, 0x0000);
	write_cmos_sensor(0x19C6, 0x0000);
	write_cmos_sensor(0x1A06, 0x0000);
	write_cmos_sensor(0x1A46, 0x0000);
	write_cmos_sensor(0x1958, 0x0041);
	write_cmos_sensor(0x195A, 0x008F);
	write_cmos_sensor(0x195C, 0x00C4);
	write_cmos_sensor(0x195E, 0x0288);
	write_cmos_sensor(0x1962, 0x0041);
	write_cmos_sensor(0x1964, 0x008F);
	write_cmos_sensor(0x1966, 0x00C4);
	write_cmos_sensor(0x1968, 0x0288);
	write_cmos_sensor(0x196C, 0x0041);
	write_cmos_sensor(0x196E, 0x008F);
	write_cmos_sensor(0x1970, 0x00C4);
	write_cmos_sensor(0x1972, 0x0288);
	write_cmos_sensor(0x1976, 0x0041);
	write_cmos_sensor(0x1978, 0x008F);
	write_cmos_sensor(0x197A, 0x00C4);
	write_cmos_sensor(0x197C, 0x0288);
	write_cmos_sensor(0x1980, 0x0045);
	write_cmos_sensor(0x1982, 0x002B);
	write_cmos_sensor(0x1984, 0x2015);
	write_cmos_sensor(0x1988, 0x0076);
	write_cmos_sensor(0x198A, 0x0000);
	write_cmos_sensor(0x198C, 0x082C);
	write_cmos_sensor(0x198E, 0x0000);
	write_cmos_sensor(0x1990, 0x2A17);
	write_cmos_sensor(0x1992, 0x0000);
	write_cmos_sensor(0x1994, 0x0000);
	write_cmos_sensor(0x1996, 0x0000);
	write_cmos_sensor(0x19C0, 0x0045);
	write_cmos_sensor(0x19C2, 0x002B);
	write_cmos_sensor(0x19C4, 0x2015);
	write_cmos_sensor(0x19C8, 0x0076);
	write_cmos_sensor(0x19CA, 0x0000);
	write_cmos_sensor(0x19CC, 0x082C);
	write_cmos_sensor(0x19CE, 0x0000);
	write_cmos_sensor(0x19D0, 0x2A17);
	write_cmos_sensor(0x19D2, 0x0000);
	write_cmos_sensor(0x19D4, 0x0000);
	write_cmos_sensor(0x19D6, 0x0000);
	write_cmos_sensor(0x1A00, 0x0045);
	write_cmos_sensor(0x1A02, 0x002B);
	write_cmos_sensor(0x1A04, 0x2015);
	write_cmos_sensor(0x1A08, 0x0076);
	write_cmos_sensor(0x1A0A, 0x0000);
	write_cmos_sensor(0x1A0C, 0x082C);
	write_cmos_sensor(0x1A0E, 0x0000);
	write_cmos_sensor(0x1A10, 0x2A17);
	write_cmos_sensor(0x1A12, 0x0000);
	write_cmos_sensor(0x1A14, 0x0000);
	write_cmos_sensor(0x1A16, 0x0000);
	write_cmos_sensor(0x1A40, 0x0045);
	write_cmos_sensor(0x1A42, 0x002B);
	write_cmos_sensor(0x1A44, 0x2015);
	write_cmos_sensor(0x1A48, 0x0076);
	write_cmos_sensor(0x1A4A, 0x0000);
	write_cmos_sensor(0x1A4C, 0x082C);
	write_cmos_sensor(0x1A4E, 0x0000);
	write_cmos_sensor(0x1A50, 0x2A17);
	write_cmos_sensor(0x1A52, 0x0000);
	write_cmos_sensor(0x1A54, 0x0000);
	write_cmos_sensor(0x1A56, 0x0000);
	write_cmos_sensor(0x19BC, 0x2000);
	write_cmos_sensor(0x19FC, 0x2000);
	write_cmos_sensor(0x1A3C, 0x2000);
	write_cmos_sensor(0x1A7C, 0x2000);
	write_cmos_sensor(0x361C, 0x0000);
	write_cmos_sensor(0x027E, 0x0100);

}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0B00, 0X0100);
	else
		write_cmos_sensor(0x0B00, 0x0000);
	return ERROR_NONE;
}

static void preview_setting(kal_uint16 currefps)  //change
{
	remosaic_flag = 0;
	LOG_INF("E\n");
	write_cmos_sensor(0x0B00, 0x0000);
	write_cmos_sensor(0x0204, 0x0200);
	write_cmos_sensor(0x0206, 0x02C6);
	write_cmos_sensor(0x020A, 0x0EA7);
	write_cmos_sensor(0x020E, 0x0EAB);
	write_cmos_sensor(0x0224, 0x0044);
	write_cmos_sensor(0x022A, 0x0015);
	write_cmos_sensor(0x022C, 0x0E25);
	write_cmos_sensor(0x022E, 0x0DC9);
	write_cmos_sensor(0x0234, 0x3311);
	write_cmos_sensor(0x0236, 0x3311);
	write_cmos_sensor(0x0238, 0x3311);
	write_cmos_sensor(0x023A, 0x2222);
	write_cmos_sensor(0x0268, 0x0108);
	write_cmos_sensor(0x0404, 0x0008);
	write_cmos_sensor(0x0406, 0x1244);
	write_cmos_sensor(0x0440, 0x011D);
	write_cmos_sensor(0x0D28, 0x0008);
	write_cmos_sensor(0x0D2A, 0x1247);
	write_cmos_sensor(0x0524, 0x5858);
	write_cmos_sensor(0x0526, 0x5858);
	write_cmos_sensor(0x0F00, 0x0400);
	write_cmos_sensor(0x0F04, 0x0010);
	write_cmos_sensor(0x0B04, 0x00FC);
	write_cmos_sensor(0x0B12, 0x0900);
	write_cmos_sensor(0x0B14, 0x06C0);
	write_cmos_sensor(0x0B20, 0x0200);
	write_cmos_sensor(0x1100, 0x1100);
	write_cmos_sensor(0x1108, 0x0002);
	write_cmos_sensor(0x1116, 0x0000);
	write_cmos_sensor(0x1118, 0x0014);
	write_cmos_sensor(0x0A0A, 0x8388);
	write_cmos_sensor(0x0A10, 0xB440);
	write_cmos_sensor(0x0C14, 0x0020);
	write_cmos_sensor(0x0C18, 0x1200);
	write_cmos_sensor(0x0C1A, 0x0700);
	write_cmos_sensor(0x0736, 0x0050);
	write_cmos_sensor(0x0738, 0x0002);
	write_cmos_sensor(0x073C, 0x0700);
	write_cmos_sensor(0x0746, 0x00D4);
	write_cmos_sensor(0x0748, 0x0002);
	write_cmos_sensor(0x074A, 0x0900);
	write_cmos_sensor(0x074C, 0x0100);
	write_cmos_sensor(0x074E, 0x0100);
	write_cmos_sensor(0x1200, 0x0946);
	write_cmos_sensor(0x1000, 0x0300);
	write_cmos_sensor(0x1002, 0xC311);
	write_cmos_sensor(0x1004, 0x2BB0);
	write_cmos_sensor(0x1010, 0x06CB);
	write_cmos_sensor(0x1012, 0x0097);
	write_cmos_sensor(0x1014, 0x0020);
	write_cmos_sensor(0x1016, 0x0020);
	write_cmos_sensor(0x101A, 0x0020);
	write_cmos_sensor(0x1020, 0xC107);
	write_cmos_sensor(0x1022, 0x071D);
	write_cmos_sensor(0x1024, 0x0307);
	write_cmos_sensor(0x1026, 0x080B);
	write_cmos_sensor(0x1028, 0x1209);
	write_cmos_sensor(0x102A, 0x0C0A);
	write_cmos_sensor(0x102C, 0x1500);
	write_cmos_sensor(0x1038, 0x0000);
	write_cmos_sensor(0x103E, 0x0101);
	write_cmos_sensor(0x1042, 0x0008);
	write_cmos_sensor(0x1044, 0x0120);
	write_cmos_sensor(0x1046, 0x01B0);
	write_cmos_sensor(0x1048, 0x0090);
	write_cmos_sensor(0x1066, 0x06EE);
	write_cmos_sensor(0x1600, 0x0400);
	write_cmos_sensor(0x1608, 0x0020);
	write_cmos_sensor(0x160A, 0x1200);
	write_cmos_sensor(0x160C, 0x001A);
	write_cmos_sensor(0x160E, 0x0D80);

}

static void capture_setting(kal_uint16 currefps)  //change
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0B00, 0x0000);
	write_cmos_sensor(0x0204, 0x0000);
	write_cmos_sensor(0x0206, 0x02C6);
	write_cmos_sensor(0x020A, 0x0EA7);
	write_cmos_sensor(0x020E, 0x0EAB);
	write_cmos_sensor(0x0224, 0x0046);
	write_cmos_sensor(0x022A, 0x0017);
	write_cmos_sensor(0x022C, 0x0E1B);
	write_cmos_sensor(0x022E, 0x0DC9);
	write_cmos_sensor(0x0234, 0x1111);
	write_cmos_sensor(0x0236, 0x1111);
	write_cmos_sensor(0x0238, 0x1111);
	write_cmos_sensor(0x023A, 0x1111);
	write_cmos_sensor(0x0268, 0x0108);
	write_cmos_sensor(0x0404, 0x0008);
	write_cmos_sensor(0x0406, 0x1244);
	write_cmos_sensor(0x0440, 0x011D);
	write_cmos_sensor(0x0D28, 0x0008);
	write_cmos_sensor(0x0D2A, 0x1247);
	write_cmos_sensor(0x0524, 0x5858);
	write_cmos_sensor(0x0526, 0x5858);
	write_cmos_sensor(0x0F00, 0x0000);
	write_cmos_sensor(0x0F04, 0x0020);
	write_cmos_sensor(0x0B04, 0x00DC);
	write_cmos_sensor(0x0B12, 0x1200);
	write_cmos_sensor(0x0B14, 0x0D80);
	write_cmos_sensor(0x0B20, 0x0100);
	write_cmos_sensor(0x1100, 0x1100);
	write_cmos_sensor(0x1108, 0x0002);
	write_cmos_sensor(0x1116, 0x0000);
	write_cmos_sensor(0x1118, 0x0016);
	write_cmos_sensor(0x0A0A, 0x8388);
	write_cmos_sensor(0x0A10, 0xB040);
	write_cmos_sensor(0x0C14, 0x0020);
	write_cmos_sensor(0x0C18, 0x1200);
	write_cmos_sensor(0x0C1A, 0x0D80);
	write_cmos_sensor(0x0736, 0x0050);
	write_cmos_sensor(0x0738, 0x0002);
	write_cmos_sensor(0x073C, 0x0700);
	write_cmos_sensor(0x0746, 0x00D4);
	write_cmos_sensor(0x0748, 0x0002);
	write_cmos_sensor(0x074A, 0x0900);
	write_cmos_sensor(0x074C, 0x0000);
	write_cmos_sensor(0x074E, 0x0100);
	write_cmos_sensor(0x1200, 0x0B46);
	write_cmos_sensor(0x1000, 0x0300);
	write_cmos_sensor(0x1002, 0xC311);
	write_cmos_sensor(0x1004, 0x2BB0);
	write_cmos_sensor(0x1010, 0x0DBB);
	write_cmos_sensor(0x1012, 0x0148);
	write_cmos_sensor(0x1014, 0x0020);
	write_cmos_sensor(0x1016, 0x0020);
	write_cmos_sensor(0x101A, 0x0020);
	write_cmos_sensor(0x1020, 0xC10D);
	write_cmos_sensor(0x1022, 0x0D38);
	write_cmos_sensor(0x1024, 0x050D);
	write_cmos_sensor(0x1026, 0x1012);
	write_cmos_sensor(0x1028, 0x1C10);
	write_cmos_sensor(0x102A, 0x170A);
	write_cmos_sensor(0x102C, 0x2800);
	write_cmos_sensor(0x1038, 0x0000);
	write_cmos_sensor(0x103E, 0x0001);
	write_cmos_sensor(0x1042, 0x0008);
	write_cmos_sensor(0x1044, 0x0120);
	write_cmos_sensor(0x1046, 0x01B0);
	write_cmos_sensor(0x1048, 0x0090);
	write_cmos_sensor(0x1066, 0x0E01);
	write_cmos_sensor(0x1600, 0x0000);
	write_cmos_sensor(0x1608, 0x0020);
	write_cmos_sensor(0x160A, 0x1200);
	write_cmos_sensor(0x160C, 0x001A);
	write_cmos_sensor(0x160E, 0x0D80);

	remosaic_flag = 1;
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	LOG_INF("E! video just has 30fps preview size setting ,NOT HAS 24FPS SETTING!\n");
	capture_setting(currefps);
}

static void custom1_setting(kal_uint16 currefps)  //change    4224*3168
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0B00, 0x0000);
	write_cmos_sensor(0x0204, 0x0000);
	write_cmos_sensor(0x0206, 0x02C6);
	write_cmos_sensor(0x020A, 0x0EA7);
	write_cmos_sensor(0x020E, 0x0EAB);
	write_cmos_sensor(0x0224, 0x00D6);
	write_cmos_sensor(0x022A, 0x0017);
	write_cmos_sensor(0x022C, 0x0E1B);
	write_cmos_sensor(0x022E, 0x0D39);
	write_cmos_sensor(0x0234, 0x1111);
	write_cmos_sensor(0x0236, 0x1111);
	write_cmos_sensor(0x0238, 0x1111);
	write_cmos_sensor(0x023A, 0x1111);
	write_cmos_sensor(0x0268, 0x0108);
	write_cmos_sensor(0x0404, 0x0008);
	write_cmos_sensor(0x0406, 0x1244);
	write_cmos_sensor(0x0440, 0x011D);
	write_cmos_sensor(0x0D28, 0x0008);
	write_cmos_sensor(0x0D2A, 0x1247);
	write_cmos_sensor(0x0524, 0x5858);
	write_cmos_sensor(0x0526, 0x5858);
	write_cmos_sensor(0x0F00, 0x0000);
	write_cmos_sensor(0x0F04, 0x00E0);
	write_cmos_sensor(0x0B04, 0x00DC);
	write_cmos_sensor(0x0B12, 0x1080);
	write_cmos_sensor(0x0B14, 0x0C60);
	write_cmos_sensor(0x0B20, 0x0100);
	write_cmos_sensor(0x1100, 0x1100);
	write_cmos_sensor(0x1108, 0x0002);
	write_cmos_sensor(0x1116, 0x0000);
	write_cmos_sensor(0x1118, 0x00A6);
	write_cmos_sensor(0x0A0A, 0x8388);
	write_cmos_sensor(0x0A10, 0xB040);
	write_cmos_sensor(0x0C14, 0x00E0);
	write_cmos_sensor(0x0C18, 0x1080);
	write_cmos_sensor(0x0C1A, 0x0C80);
	write_cmos_sensor(0x0736, 0x0050);
	write_cmos_sensor(0x0738, 0x0002);
	write_cmos_sensor(0x073C, 0x0700);
	write_cmos_sensor(0x0746, 0x00D4);
	write_cmos_sensor(0x0748, 0x0002);
	write_cmos_sensor(0x074A, 0x0900);
	write_cmos_sensor(0x074C, 0x0000);
	write_cmos_sensor(0x074E, 0x0100);
	write_cmos_sensor(0x1200, 0x0B46);
	write_cmos_sensor(0x1000, 0x0300);
	write_cmos_sensor(0x1002, 0xC311);
	write_cmos_sensor(0x1004, 0x2BB0);
	write_cmos_sensor(0x1010, 0x0DBB);
	write_cmos_sensor(0x1012, 0x01C0);
	write_cmos_sensor(0x1014, 0x0020);
	write_cmos_sensor(0x1016, 0x0020);
	write_cmos_sensor(0x101A, 0x0020);
	write_cmos_sensor(0x1020, 0xC10D);
	write_cmos_sensor(0x1022, 0x0D38);
	write_cmos_sensor(0x1024, 0x050D);
	write_cmos_sensor(0x1026, 0x1012);
	write_cmos_sensor(0x1028, 0x1C10);
	write_cmos_sensor(0x102A, 0x170A);
	write_cmos_sensor(0x102C, 0x2800);
	write_cmos_sensor(0x1038, 0x0000);
	write_cmos_sensor(0x103E, 0x0001);
	write_cmos_sensor(0x1042, 0x0008);
	write_cmos_sensor(0x1044, 0x0120);
	write_cmos_sensor(0x1046, 0x01B0);
	write_cmos_sensor(0x1048, 0x0090);
	write_cmos_sensor(0x1066, 0x0E01);
	write_cmos_sensor(0x1600, 0x0000);
	write_cmos_sensor(0x1608, 0x0020);
	write_cmos_sensor(0x160A, 0x1200);
	write_cmos_sensor(0x160C, 0x001A);
	write_cmos_sensor(0x160E, 0x0D80);

}
static void hs_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
	preview_setting(currefps);
}
static void slim_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
//	preview_setting(currefps);
	write_cmos_sensor(0x0B00, 0x0000);
	write_cmos_sensor(0x0204, 0x0300);
	write_cmos_sensor(0x0206, 0x02C6);
	write_cmos_sensor(0x020A, 0x0EA7);
	write_cmos_sensor(0x020E, 0x0EAB);
	write_cmos_sensor(0x0224, 0x0040);
	write_cmos_sensor(0x022A, 0x0011);
	write_cmos_sensor(0x022C, 0x0E39);
	write_cmos_sensor(0x022E, 0x0DC9);
	write_cmos_sensor(0x0234, 0x7711);
	write_cmos_sensor(0x0236, 0x7711);
	write_cmos_sensor(0x0238, 0x7711);
	write_cmos_sensor(0x023A, 0x4444);
	write_cmos_sensor(0x0268, 0x0108);
	write_cmos_sensor(0x0404, 0x0008);
	write_cmos_sensor(0x0406, 0x1244);
	write_cmos_sensor(0x0440, 0x011D);
	write_cmos_sensor(0x0D28, 0x0008);
	write_cmos_sensor(0x0D2A, 0x1247);
	write_cmos_sensor(0x0524, 0x5858);
	write_cmos_sensor(0x0526, 0x5858);
	write_cmos_sensor(0x0F00, 0x0C00);
	write_cmos_sensor(0x0F04, 0x0008);
	write_cmos_sensor(0x0B04, 0x00FC);
	write_cmos_sensor(0x0B12, 0x0480);
	write_cmos_sensor(0x0B14, 0x0360);
	write_cmos_sensor(0x0B20, 0x0400);
	write_cmos_sensor(0x1100, 0x1100);
	write_cmos_sensor(0x1108, 0x0002);
	write_cmos_sensor(0x1116, 0x0000);
	write_cmos_sensor(0x1118, 0x0010);
	write_cmos_sensor(0x0A0A, 0x8388);
	write_cmos_sensor(0x0A10, 0xB460);
	write_cmos_sensor(0x0C14, 0x0020);
	write_cmos_sensor(0x0C18, 0x1200);
	write_cmos_sensor(0x0C1A, 0x0380);
	write_cmos_sensor(0x0736, 0x0050);
	write_cmos_sensor(0x0738, 0x0002);
	write_cmos_sensor(0x073C, 0x0700);
	write_cmos_sensor(0x0746, 0x00D4);
	write_cmos_sensor(0x0748, 0x0002);
	write_cmos_sensor(0x074A, 0x0900);
	write_cmos_sensor(0x074C, 0x0300);
	write_cmos_sensor(0x074E, 0x0100);
	write_cmos_sensor(0x1200, 0x0946);
	write_cmos_sensor(0x1000, 0x0300);
	write_cmos_sensor(0x1002, 0xC311);
	write_cmos_sensor(0x1004, 0x2BB0);
	write_cmos_sensor(0x1010, 0x0357);
	write_cmos_sensor(0x1012, 0x003F);
	write_cmos_sensor(0x1014, 0x0020);
	write_cmos_sensor(0x1016, 0x0020);
	write_cmos_sensor(0x101A, 0x0020);
	write_cmos_sensor(0x1020, 0xC104);
	write_cmos_sensor(0x1022, 0x030F);
	write_cmos_sensor(0x1024, 0x0304);
	write_cmos_sensor(0x1026, 0x0407);
	write_cmos_sensor(0x1028, 0x0B06);
	write_cmos_sensor(0x102A, 0x0605);
	write_cmos_sensor(0x102C, 0x0C00);
	write_cmos_sensor(0x1038, 0x0000);
	write_cmos_sensor(0x103E, 0x0301);
	write_cmos_sensor(0x1042, 0x0008);
	write_cmos_sensor(0x1044, 0x0120);
	write_cmos_sensor(0x1046, 0x01B0);
	write_cmos_sensor(0x1048, 0x0090);
	write_cmos_sensor(0x1066, 0x0369);
	write_cmos_sensor(0x1600, 0x0000);
	write_cmos_sensor(0x1608, 0x0020);
	write_cmos_sensor(0x160A, 0x1200);
	write_cmos_sensor(0x160C, 0x001A);
	write_cmos_sensor(0x160E, 0x0D80);
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0716) << 8) | read_cmos_sensor(0x0717));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {

				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,
				*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	LOG_INF("park Read sensor sensor_id fail, id: 0x%x\n", *sensor_id);
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	LOG_INF("PLATFORM:MIPI 4LANE hi1634z6250cc142 OPEN+++++ ++++\n");

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x,sensor_id =0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting(imgsensor.current_fps);
/* mdelay(10); */
	/*set_mirror_flip(imgsensor.mirror);*/
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 16M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		/* imgsensor.autoflicker_en = KAL_FALSE; */

	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF
			    ("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
			     imgsensor_info.cap1.max_framerate / 10,
			     imgsensor_info.cap.max_framerate);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		/* imgsensor.autoflicker_en = KAL_FALSE; */
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	/* imgsensor.autoflicker_en = KAL_FALSE; */
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(30);

	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/*      normal_video   */
static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting(imgsensor.current_fps);
	
	/*mdelay(10); */
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting(imgsensor.current_fps);

	/*mdelay(10); */
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}				/*      slim_video       */
static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 16M */
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting(imgsensor.current_fps);

	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d %d\n", scenario_id, sensor_info->SensorOutputDataFormat);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting
										 * shutter default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_RAW;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)	/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MUINT32 framerate)
{
	/* kal_int16 dummyLine; */
	kal_uint32 frameHeight;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		LOG_INF("frameHeight = %d\n", frameHeight);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
		    ? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frameHeight =
		    imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.normal_video.framelength)
			? (frameHeight - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frameHeight =
		    imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line = (frameHeight > imgsensor_info.cap.framelength)
		    ? (frameHeight - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frameHeight =
		    imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.hs_video.framelength)
		    ? (frameHeight - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frameHeight =
		    imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.slim_video.framelength)
			? (frameHeight - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frameHeight =
		    imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frameHeight >
		     imgsensor_info.custom1.framelength) ? (frameHeight -
							imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
			? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
						enum MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor(0x5081, 0x01);
		write_cmos_sensor(0x5000, (read_cmos_sensor(0x5000)&0xa1)|0x00);

	} else {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor(0x5081, 0x00);
		write_cmos_sensor(0x5000, (read_cmos_sensor(0x5000)&0xa1)|0x040);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
/*
#define EEPROM_READ_ID  0xA0
static void read_eeprom(int offset, char *data, kal_uint32 size)
{
	int i = 0, addr = offset;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	for (i = 0; i < size; i++) {
		pu_send_cmd[0] = (char)(addr >> 8);
		pu_send_cmd[1] = (char)(addr & 0xFF);
		iReadRegI2C(pu_send_cmd, 2, &data[i], 1, EEPROM_READ_ID);

		addr++;
	}
}*/

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* LOG_INF("feature_id = %d\n", feature_id); */
	switch (feature_id) {
/*
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(feature_data + 2) = 2;
			break;
		default:
			*(feature_data + 2) = 1;
			break;
		}
		break;
*/
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16,
			*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *feature_data,
			*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT8) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
					sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				rate = imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("mtk_test SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("mtk_test SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/* PDAF capacity enable or not, S5K3L6 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.cap.pclk /
				(imgsensor_info.cap.linelength - 80))*
				imgsensor_info.cap.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.normal_video.pclk /
				(imgsensor_info.normal_video.linelength - 80))*
				imgsensor_info.normal_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.hs_video.pclk /
				(imgsensor_info.hs_video.linelength - 80))*
				imgsensor_info.hs_video.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				(imgsensor_info.slim_video.linelength - 80))*
				imgsensor_info.slim_video.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom1.pclk /
				(imgsensor_info.custom1.linelength - 80))*
				imgsensor_info.custom1.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.pre.pclk /
				(imgsensor_info.pre.linelength - 80))*
				imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_info("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT32) (*feature_data), (UINT32) (*(feature_data + 1)));
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI1634Z6250CC142_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    HI1634Z6250CC142_MIPI_RAW_SensorInit      */
