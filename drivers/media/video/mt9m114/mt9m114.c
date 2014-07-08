/*
 * Driver for OV5642 CMOS Image Sensor from OmniVision
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
 /* CAMERA_FRONT_SENSOR_SETTING:f20130909_03 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/hardirq.h>
#include <linux/miscdevice.h>
#include <asm/io.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <plat/rk_camera.h>
#include <mach/iomux.h>

#include "mt9m114.h"

#define ASUS_CAMERA_SUPPORT 1
static int debug;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING fmt , ## arg); } while (0)


#if SENSOR_NEED_SOFTRESET
struct seq_info soft_reset_seq_ops[] =
{
		// ops         reg       len      val    tmo
	//	{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100},	// Polling R0x0080[1] till "0". delay=10ms timeout = 100
	//	{SEQ_REG,      0x301A,  SEQ_WORD, 0x0234, 0},	// RESET_REGISTER
	//	{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0},	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0x001A,  SEQ_WORD, 0x0001, 0},
		{SEQ_REG,      0x001A,  SEQ_WORD, 0x0000, 45},	//delay 45ms after reg set
		//Mask bad frame
		{SEQ_BIT_SET,  0x301A,  SEQ_WORD, 0x0200, 0},	//BITFIELD= 0x301A, 0x0200, 1
	//	{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0},	// LOGICAL_ADDRESS_ACCESS
		{SEQ_END,           0,         0,      0, 0}
};
#endif

// Preview: 1280x960  YCbCr 30fps max
struct seq_info init_seq_ops[] =
{
		// ops         reg       len      val    tmo
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100},	// Polling R0x0080[1] till "0". delay=10ms timeout = 100
		{SEQ_REG,      0x301A,  SEQ_WORD, 0x0234, 0},	// RESET_REGISTER
		// [Step2-PLL_Timing]
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0},	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC97E,  SEQ_BYTE,   0x01, 0},	// CAM_SYSCTL_PLL_ENABLE
		{SEQ_REG,      0xC980,  SEQ_WORD, 0x0120, 0},	// CAM_SYSCTL_PLL_DIVIDER_M_N
		{SEQ_REG,      0xC982,  SEQ_WORD, 0x0700, 0},	// CAM_SYSCTL_PLL_DIVIDER_P
		{SEQ_REG,      0xC800,  SEQ_WORD, 0x0004, 0},	// CAM_SENSOR_CFG_X_ADDR_START
		{SEQ_REG,      0xC802,  SEQ_WORD, 0x0004, 0},	// CAM_SENSOR_CFG_X_ADDR_START
		{SEQ_REG,      0xC804,  SEQ_WORD, 0x03CB, 0},	// CAM_SENSOR_CFG_Y_ADDR_END
		{SEQ_REG,      0xC806,  SEQ_WORD, 0x050B, 0},	// CAM_SENSOR_CFG_X_ADDR_END
		{SEQ_REG,      0xC808, SEQ_DWORD, 0x02DC6C00, 0},	// CAM_SENSOR_CFG_PIXCLK
		{SEQ_REG,      0xC80C,  SEQ_WORD, 0x0001, 0},	// CAM_SENSOR_CFG_ROW_SPEED
		{SEQ_REG,      0xC80E,  SEQ_WORD, 0x00DB, 0},	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
		{SEQ_REG,      0xC810,  SEQ_WORD, 0x05B3, 0},	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
		{SEQ_REG,      0xC812,  SEQ_WORD, 0x03EE, 0},	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
		{SEQ_REG,      0xC814,  SEQ_WORD, 0x0636, 0},	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
		{SEQ_REG,      0xC816,  SEQ_WORD, 0x0060, 0},	// CAM_SENSOR_CFG_FINE_CORRECTION
		{SEQ_REG,      0xC818,  SEQ_WORD, 0x03C3, 0},	// CAM_SENSOR_CFG_CPIPE_LAST_ROW
		{SEQ_REG,      0xC834,  SEQ_WORD, 0x0000, 0},	// CAM_SENSOR_CONTROL_READ_MODE
		{SEQ_REG,      0xC854,  SEQ_WORD, 0x0000, 0},	// CAM_CROP_WINDOW_XOFFSET
		{SEQ_REG,      0xC856,  SEQ_WORD, 0x0000, 0},	// CAM_CROP_WINDOW_YOFFSET
		{SEQ_REG,      0xC858,  SEQ_WORD, 0x0500, 0},	// CAM_CROP_WINDOW_WIDTH
		{SEQ_REG,      0xC85A,  SEQ_WORD, 0x03C0, 0},	// CAM_CROP_WINDOW_HEIGHT
		{SEQ_REG,      0xC85C,  SEQ_BYTE,   0x03, 0},	// CAM_CROP_CROPMODE
		{SEQ_REG,      0xC868,  SEQ_WORD, 0x0500, 0},	// CAM_OUTPUT_WIDTH
		{SEQ_REG,      0xC86A,  SEQ_WORD, 0x03C0, 0},	// CAM_OUTPUT_HEIGHT
		{SEQ_REG,      0xC88C,  SEQ_WORD, 0x1E02, 0},	// CAM_AET_MAX_FRAME_RATE
		{SEQ_REG,      0xC88E,  SEQ_WORD, 0x0F00, 0},	// CAM_AET_MIN_FRAME_RATE
		{SEQ_REG,      0xC914,  SEQ_WORD, 0x0000, 0},	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
		{SEQ_REG,      0xC916,  SEQ_WORD, 0x0000, 0},	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
		{SEQ_REG,      0xC918,  SEQ_WORD, 0x04FF, 0},	// CAM_STAT_AWB_CLIP_WINDOW_XEND
		{SEQ_REG,      0xC91A,  SEQ_WORD, 0x03BF, 0},	// CAM_STAT_AWB_CLIP_WINDOW_YEND
		{SEQ_REG,      0xC91C,  SEQ_WORD, 0x0000, 0},	// CAM_STAT_AE_INITIAL_WINDOW_XSTART
		{SEQ_REG,      0xC91E,  SEQ_WORD, 0x0000, 0},	// CAM_STAT_AE_INITIAL_WINDOW_YSTART
		{SEQ_REG,      0xC920,  SEQ_WORD, 0x00FF, 0},	// CAM_STAT_AE_INITIAL_WINDOW_XEND
		{SEQ_REG,      0xC922,  SEQ_WORD, 0x00BF, 0},	// CAM_STAT_AE_INITIAL_WINDOW_YEND
		//
		{SEQ_REG,      0xE801,  SEQ_BYTE,   0x00, 0},	// AUTO_BINNING_MODE
		//
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xDC00, 0},	// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0},	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0},	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100},	// Polling R0x0080[1] till "0". delay=10ms timeout = 100
		// [Step3-Recommended]
		// [Sensor optimization]
		{SEQ_REG,      0x316A,  SEQ_WORD, 0x8270, 0}, 	// DAC_TXLO_ROW
		{SEQ_REG,      0x316C,  SEQ_WORD, 0x8270, 0}, 	// DAC_TXLO
		{SEQ_REG,      0x3ED0,  SEQ_WORD, 0x2305, 0},	// DAC_LD_4_5
		{SEQ_REG,      0x3ED2,  SEQ_WORD, 0x77CF, 0}, 	// DAC_LD_6_7
		{SEQ_REG,      0x316E,  SEQ_WORD, 0x8202, 0},	// DAC_ECL
		{SEQ_REG,      0x3180,  SEQ_WORD, 0x87FF, 0},	// DELTA_DK_CONTROL
		{SEQ_REG,      0x30D4,  SEQ_WORD, 0x6080, 0},	// COLUMN_CORRECTION
		{SEQ_REG,      0xA802,  SEQ_WORD, 0x0008, 0},	// AE_TRACK_MODE
		// LOAD=Errata item 1
		{SEQ_REG,      0x3E14,  SEQ_WORD, 0xFF39, 0}, 	// SAMP_COL_PUP2
		// added for color dots correction
		{SEQ_REG,      0x31E0,  SEQ_WORD, 0x0001, 0},
		//[Load Patch 1004]
		{SEQ_REG,      0x0982,  SEQ_WORD, 0x0001, 0}, 	// ACCESS_CTL_STAT
		{SEQ_REG,      0x098A,  SEQ_WORD, 0x5C10, 0},	// PHYSICAL_ADDRESS_ACCESS

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDC10, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xC0F1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0cda, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x76cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2184, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x9624, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x218c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8fc3, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x75cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf686, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1550, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe001, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d50, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1552, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1100, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x6038, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d52, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDC40, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x081b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00d1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8512, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00c0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7822, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2089, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0fc1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2008, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0f81, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff80, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8512, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1801, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0052, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa512, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1544, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb861, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x262f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf007, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d44, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDC70, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x20ca, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0021, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x20cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x04e1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0850, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x04a1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x21ca, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0021, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1542, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1140, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d2c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x6038, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d42, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1542, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1140, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb601, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x046d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb893, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x002d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x04a0, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDCA0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd900, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x72cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2240, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0340, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa212, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x208a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0fff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a42, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd830, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a44, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a50, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a52, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1242, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0140, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8a2c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x6038, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDCD0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a42, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1242, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0141, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2184, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb021, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb893, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x07e5, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0460, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd901, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0bfa, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x216f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0043, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc1a4, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x220a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1f80, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDCD0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a42, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1242, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0141, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2184, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb021, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb893, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x07e5, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0460, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd901, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0bfa, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x216f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0043, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc1a4, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x220a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1f80, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDD00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2240, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x134f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a48, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x13c0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7fff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe230, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc240, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xda00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf00c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1003, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1301, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x04cb, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7261, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2108, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2009, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a48, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x10c0, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDD30, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x100b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc300, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0be7, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x90c4, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2102, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0003, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x238c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8fc3, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf6c7, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xdaff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a05, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1082, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc241, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf005, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7a6f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc241, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a05, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x10c2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8040, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xda00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x20c0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0064, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDD60, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x781c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc042, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1c0e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x3082, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a48, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x13c0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7548, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7348, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7148, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7648, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7608, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1400, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x300b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x084d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x02c5, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe101, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1001, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x04cb, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a48, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDD90, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7361, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1408, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x300b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2302, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x02c0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x780d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2607, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x903e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x07d6, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffe3, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x792f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x09cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8152, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1248, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x100e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2400, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x334b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe501, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7ee2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0dbf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x90f2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1b0c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1382, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDDC0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc123, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x140e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x3080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7822, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a07, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x124c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x120b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1207, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1083, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2142, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x004b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x781b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0b21, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x02e2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a4c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe101, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0915, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00c2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc101, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1204, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDDF0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1083, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x090d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00c2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe001, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a4c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a06, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x234a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7169, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf008, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2053, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0003, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x6179, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x781c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2340, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x104b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1203, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1083, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0bf1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x90c2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1202, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDE20, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x091d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0004, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc644, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x881b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe0b2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd83c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x20ca, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0ca2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a01, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1a4c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x02b9, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0a4, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff95, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1fe0, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDE50, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x19d0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x001c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x19d1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x001c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x901f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb861, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x19d2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x001c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0d1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7ee0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0a7a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc5d4, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x9041, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x9023, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x75cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDE80, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7942, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb967, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7f30, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb53f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc84c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x91d3, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x108b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2615, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1380, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x090f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0c91, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0a8e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd906, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7e10, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2615, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1380, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0a82, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd960, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDEB0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x790f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x090d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0133, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad0c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd904, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad2c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x79ec, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2941, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7402, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x2184, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb142, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1906, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0e44, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffde, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70c9, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0a5a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d2c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad0b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad01, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0219, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDEE0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x05a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa513, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc644, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa91b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd902, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc84c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x093e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x03a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa826, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffdc, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf1b5, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x09ea, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x75cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1080, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDF10, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x08a7, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0010, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0813, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x009e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe181, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x20ca, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00a1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf24b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x090f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0050, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0927, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0091, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1550, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xde00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad2a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d50, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDF40, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1382, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1552, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1101, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d52, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1384, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb524, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x082d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x015f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff55, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd803, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf033, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1540, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0967, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x00d1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1550, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1081, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xde00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad2a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d50, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1382, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1552, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1101, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d52, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDF70, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1384, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb524, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0811, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x019e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb8a0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xad00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff47, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d40, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1382, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf01f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff5a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d01, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d40, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe812, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc644, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x893b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7030, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x22d1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8062, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf20a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0a0f, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x009e, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDFA0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc84c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x893b, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe902, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffcf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8d00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb8e7, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x26ca, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1022, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf5e2, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff3c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd801, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1d40, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x1002, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0141, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc5e1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff34, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xdd00, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xDFD0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe090, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa8a8, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb893, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0c8a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0460, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd901, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xdc10, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd813, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0b96, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0460, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x72a9, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0119, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0580, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x5bae, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7940, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xff9d, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf135, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xE000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x5cba, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7840, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0815, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x001e, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc84c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x881a, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe080, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0ee0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffc1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf121, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x78e0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc0f1, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd900, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf009, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xE030, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe0ac, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x7835, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8041, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe102, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xa040, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x09f3, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x8114, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x71cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe058, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x70cf, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xc594, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xb030, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffdd, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xd800, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xf109, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0300, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0204, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0700, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xE060, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},

		{SEQ_BURST,    0x0019,	SEQ_WORD, 0xE090, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0x0000, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xcb68, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xdff0, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xcb6c, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xffff, 0},
		{SEQ_REG,      0x0000,  SEQ_WORD, 0xe000, 0},

		{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0},	// LOGICAL_ADDRESS_ACCESS

		//[Apply Patch 1004]
		{SEQ_REG,      0xE000,  SEQ_WORD, 0x1028, 0}, 	// PATCHLDR_LOADER_ADDRESS
		{SEQ_REG,      0xE002,  SEQ_WORD, 0x1004, 0}, 	// PATCHLDR_PATCH_ID

		{SEQ_REG,      0xE004, SEQ_DWORD, 0x41030202, 0}, 	// PATCHLDR_FIRMWARE_ID
		{SEQ_REG,      0x0080,  SEQ_WORD, 0xFFF0, 0}, 	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0001, 100},	// Polling R0x0080[0] till "0".
		{SEQ_REG,      0x0080,  SEQ_WORD, 0xFFF1, 0}, 	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0001, 100},	// Polling R0x0080[0] till "0".

		// PGA parameter and APGA
		//[Step4-APGA]
		// [APGA Settings 95% 2013/07/19 03:15:07]
		{SEQ_REG,      0x3640,  SEQ_WORD, 0x02B0, 0}, 	//  P_G1_P0Q0
		{SEQ_REG,      0x3642,  SEQ_WORD, 0x168B, 0}, 	//  P_G1_P0Q1
		{SEQ_REG,      0x3644,  SEQ_WORD, 0x10B1, 0}, 	//  P_G1_P0Q2
		{SEQ_REG,      0x3646,  SEQ_WORD, 0x1E88, 0}, 	//  P_G1_P0Q3
		{SEQ_REG,      0x3648,  SEQ_WORD, 0xC6CD, 0}, 	//  P_G1_P0Q4
		{SEQ_REG,      0x364A,  SEQ_WORD, 0x00D0, 0}, 	//  P_R_P0Q0
		{SEQ_REG,      0x364C,  SEQ_WORD, 0x3DAA, 0}, 	//  P_R_P0Q1
		{SEQ_REG,      0x364E,  SEQ_WORD, 0x32D1, 0}, 	//  P_R_P0Q2
		{SEQ_REG,      0x3650,  SEQ_WORD, 0x066D, 0}, 	//  P_R_P0Q3
		{SEQ_REG,      0x3652,  SEQ_WORD, 0xC66E, 0}, 	//  P_R_P0Q4
		{SEQ_REG,      0x3654,  SEQ_WORD, 0x0150, 0}, 	//  P_B_P0Q0
		{SEQ_REG,      0x3656,  SEQ_WORD, 0x07AD, 0}, 	//  P_B_P0Q1
		{SEQ_REG,      0x3658,  SEQ_WORD, 0x5D30, 0}, 	//  P_B_P0Q2
		{SEQ_REG,      0x365A,  SEQ_WORD, 0x5D4C, 0}, 	//  P_B_P0Q3
		{SEQ_REG,      0x365C,  SEQ_WORD, 0xDD4D, 0}, 	//  P_B_P0Q4
		{SEQ_REG,      0x365E,  SEQ_WORD, 0x01B0, 0}, 	//  P_G2_P0Q0
		{SEQ_REG,      0x3660,  SEQ_WORD, 0xFA66, 0}, 	//  P_G2_P0Q1
		{SEQ_REG,      0x3662,  SEQ_WORD, 0x1671, 0}, 	//  P_G2_P0Q2
		{SEQ_REG,      0x3664,  SEQ_WORD, 0xD6AB, 0}, 	//  P_G2_P0Q3
		{SEQ_REG,      0x3666,  SEQ_WORD, 0xB24E, 0}, 	//  P_G2_P0Q4
		{SEQ_REG,      0x3680,  SEQ_WORD, 0x86AC, 0}, 	//  P_G1_P1Q0
		{SEQ_REG,      0x3682,  SEQ_WORD, 0xCFE8, 0}, 	//  P_G1_P1Q1
		{SEQ_REG,      0x3684,  SEQ_WORD, 0x02EF, 0}, 	//  P_G1_P1Q2
		{SEQ_REG,      0x3686,  SEQ_WORD, 0x36EE, 0}, 	//  P_G1_P1Q3
		{SEQ_REG,      0x3688,  SEQ_WORD, 0x9D6F, 0}, 	//  P_G1_P1Q4
		{SEQ_REG,      0x368A,  SEQ_WORD, 0xAE8C, 0}, 	//  P_R_P1Q0
		{SEQ_REG,      0x368C,  SEQ_WORD, 0xBECA, 0}, 	//  P_R_P1Q1
		{SEQ_REG,      0x368E,  SEQ_WORD, 0x50EE, 0}, 	//  P_R_P1Q2
		{SEQ_REG,      0x3690,  SEQ_WORD, 0x386E, 0}, 	//  P_R_P1Q3
		{SEQ_REG,      0x3692,  SEQ_WORD, 0xF2AE, 0}, 	//  P_R_P1Q4
		{SEQ_REG,      0x3694,  SEQ_WORD, 0xB46A, 0}, 	//  P_B_P1Q0
		{SEQ_REG,      0x3696,  SEQ_WORD, 0xD3AC, 0}, 	//  P_B_P1Q1
		{SEQ_REG,      0x3698,  SEQ_WORD, 0xB6EA, 0}, 	//  P_B_P1Q2
		{SEQ_REG,      0x369A,  SEQ_WORD, 0x75CD, 0}, 	//  P_B_P1Q3
		{SEQ_REG,      0x369C,  SEQ_WORD, 0x924D, 0}, 	//  P_B_P1Q4
		{SEQ_REG,      0x369E,  SEQ_WORD, 0xD52A, 0}, 	//  P_G2_P1Q0
		{SEQ_REG,      0x36A0,  SEQ_WORD, 0x54CA, 0}, 	//  P_G2_P1Q1
		{SEQ_REG,      0x36A2,  SEQ_WORD, 0xDDED, 0}, 	//  P_G2_P1Q2
		{SEQ_REG,      0x36A4,  SEQ_WORD, 0x5CAD, 0}, 	//  P_G2_P1Q3
		{SEQ_REG,      0x36A6,  SEQ_WORD, 0xE70C, 0}, 	//  P_G2_P1Q4
		{SEQ_REG,      0x36C0,  SEQ_WORD, 0x2031, 0}, 	//  P_G1_P2Q0
		{SEQ_REG,      0x36C2,  SEQ_WORD, 0x84EC, 0}, 	//  P_G1_P2Q1
		{SEQ_REG,      0x36C4,  SEQ_WORD, 0xE211, 0}, 	//  P_G1_P2Q2
		{SEQ_REG,      0x36C6,  SEQ_WORD, 0x75EF, 0}, 	//  P_G1_P2Q3
		{SEQ_REG,      0x36C8,  SEQ_WORD, 0x6713, 0}, 	//  P_G1_P2Q4
		{SEQ_REG,      0x36CA,  SEQ_WORD, 0x4F51, 0}, 	//  P_R_P2Q0
		{SEQ_REG,      0x36CC,  SEQ_WORD, 0x55CC, 0}, 	//  P_R_P2Q1
		{SEQ_REG,      0x36CE,  SEQ_WORD, 0xA291, 0}, 	//  P_R_P2Q2
		{SEQ_REG,      0x36D0,  SEQ_WORD, 0x0A10, 0}, 	//  P_R_P2Q3
		{SEQ_REG,      0x36D2,  SEQ_WORD, 0x2DD3, 0}, 	//  P_R_P2Q4
		{SEQ_REG,      0x36D4,  SEQ_WORD, 0x1331, 0}, 	//  P_B_P2Q0
		{SEQ_REG,      0x36D6,  SEQ_WORD, 0x020E, 0}, 	//  P_B_P2Q1
		{SEQ_REG,      0x36D8,  SEQ_WORD, 0xA512, 0}, 	//  P_B_P2Q2
		{SEQ_REG,      0x36DA,  SEQ_WORD, 0x2DEF, 0}, 	//  P_B_P2Q3
		{SEQ_REG,      0x36DC,  SEQ_WORD, 0x08B4, 0}, 	//  P_B_P2Q4
		{SEQ_REG,      0x36DE,  SEQ_WORD, 0x1C91, 0}, 	//  P_G2_P2Q0
		{SEQ_REG,      0x36E0,  SEQ_WORD, 0xDF2C, 0}, 	//  P_G2_P2Q1
		{SEQ_REG,      0x36E2,  SEQ_WORD, 0xEF51, 0}, 	//  P_G2_P2Q2
		{SEQ_REG,      0x36E4,  SEQ_WORD, 0x56AF, 0}, 	//  P_G2_P2Q3
		{SEQ_REG,      0x36E6,  SEQ_WORD, 0x6373, 0}, 	//  P_G2_P2Q4
		{SEQ_REG,      0x3700,  SEQ_WORD, 0x852E, 0}, 	//  P_G1_P3Q0
		{SEQ_REG,      0x3702,  SEQ_WORD, 0x010C, 0}, 	//  P_G1_P3Q1
		{SEQ_REG,      0x3704,  SEQ_WORD, 0x9DD0, 0}, 	//  P_G1_P3Q2
		{SEQ_REG,      0x3706,  SEQ_WORD, 0xB470, 0}, 	//  P_G1_P3Q3
		{SEQ_REG,      0x3708,  SEQ_WORD, 0x0251, 0}, 	//  P_G1_P3Q4
		{SEQ_REG,      0x370A,  SEQ_WORD, 0xD3CE, 0}, 	//  P_R_P3Q0
		{SEQ_REG,      0x370C,  SEQ_WORD, 0x0F6A, 0}, 	//  P_R_P3Q1
		{SEQ_REG,      0x370E,  SEQ_WORD, 0xAE10, 0}, 	//  P_R_P3Q2
		{SEQ_REG,      0x3710,  SEQ_WORD, 0x85D0, 0}, 	//  P_R_P3Q3
		{SEQ_REG,      0x3712,  SEQ_WORD, 0x5B11, 0}, 	//  P_R_P3Q4
		{SEQ_REG,      0x3714,  SEQ_WORD, 0x9168, 0}, 	//  P_B_P3Q0
		{SEQ_REG,      0x3716,  SEQ_WORD, 0x5C2E, 0}, 	//  P_B_P3Q1
		{SEQ_REG,      0x3718,  SEQ_WORD, 0x9250, 0}, 	//  P_B_P3Q2
		{SEQ_REG,      0x371A,  SEQ_WORD, 0x9DB0, 0}, 	//  P_B_P3Q3
		{SEQ_REG,      0x371C,  SEQ_WORD, 0x1891, 0}, 	//  P_B_P3Q4
		{SEQ_REG,      0x371E,  SEQ_WORD, 0xB52C, 0}, 	//  P_G2_P3Q0
		{SEQ_REG,      0x3720,  SEQ_WORD, 0x51EB, 0}, 	//  P_G2_P3Q1
		{SEQ_REG,      0x3722,  SEQ_WORD, 0xE72F, 0}, 	//  P_G2_P3Q2
		{SEQ_REG,      0x3724,  SEQ_WORD, 0x8D90, 0}, 	//  P_G2_P3Q3
		{SEQ_REG,      0x3726,  SEQ_WORD, 0x2E71, 0}, 	//  P_G2_P3Q4
		{SEQ_REG,      0x3740,  SEQ_WORD, 0x9E2E, 0}, 	//  P_G1_P4Q0
		{SEQ_REG,      0x3742,  SEQ_WORD, 0x75D0, 0}, 	//  P_G1_P4Q1
		{SEQ_REG,      0x3744,  SEQ_WORD, 0x63B3, 0}, 	//  P_G1_P4Q2
		{SEQ_REG,      0x3746,  SEQ_WORD, 0x9672, 0}, 	//  P_G1_P4Q3
		{SEQ_REG,      0x3748,  SEQ_WORD, 0x81D5, 0}, 	//  P_G1_P4Q4
		{SEQ_REG,      0x374A,  SEQ_WORD, 0xEF70, 0}, 	//  P_R_P4Q0
		{SEQ_REG,      0x374C,  SEQ_WORD, 0x21CF, 0}, 	//  P_R_P4Q1
		{SEQ_REG,      0x374E,  SEQ_WORD, 0x55F3, 0}, 	//  P_R_P4Q2
		{SEQ_REG,      0x3750,  SEQ_WORD, 0xA312, 0}, 	//  P_R_P4Q3
		{SEQ_REG,      0x3752,  SEQ_WORD, 0x8935, 0}, 	//  P_R_P4Q4
		{SEQ_REG,      0x3754,  SEQ_WORD, 0xBB11, 0}, 	//  P_B_P4Q0
		{SEQ_REG,      0x3756,  SEQ_WORD, 0xBA90, 0}, 	//  P_B_P4Q1
		{SEQ_REG,      0x3758,  SEQ_WORD, 0x47F4, 0}, 	//  P_B_P4Q2
		{SEQ_REG,      0x375A,  SEQ_WORD, 0xCF0D, 0}, 	//  P_B_P4Q3
		{SEQ_REG,      0x375C,  SEQ_WORD, 0xC495, 0}, 	//  P_B_P4Q4
		{SEQ_REG,      0x375E,  SEQ_WORD, 0xC56D, 0}, 	//  P_G2_P4Q0
		{SEQ_REG,      0x3760,  SEQ_WORD, 0x08B1, 0}, 	//  P_G2_P4Q1
		{SEQ_REG,      0x3762,  SEQ_WORD, 0x6BB3, 0}, 	//  P_G2_P4Q2
		{SEQ_REG,      0x3764,  SEQ_WORD, 0x8592, 0}, 	//  P_G2_P4Q3
		{SEQ_REG,      0x3766,  SEQ_WORD, 0x8095, 0}, 	//  P_G2_P4Q4
		{SEQ_REG,      0x3784,  SEQ_WORD, 0x0280, 0}, 	//  CENTER_COLUMN
		{SEQ_REG,      0x3782,  SEQ_WORD, 0x01E0, 0}, 	//  CENTER_ROW
		{SEQ_REG,      0x37C0,  SEQ_WORD, 0x8EAA, 0}, 	//  P_GR_Q5
		{SEQ_REG,      0x37C2,  SEQ_WORD, 0xF0E8, 0}, 	//  P_RD_Q5
		{SEQ_REG,      0x37C4,  SEQ_WORD, 0x8209, 0}, 	//  P_BL_Q5
		{SEQ_REG,      0x37C6,  SEQ_WORD, 0xFB69, 0}, 	//  P_GB_Q5
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0}, 	//  LOGICAL addressing
		{SEQ_REG,      0xC960,  SEQ_WORD, 0x0AF0, 0}, 	// CAM_PGA_L_CONFIG_COLOUR_TEMP
		{SEQ_REG,      0xC962,  SEQ_WORD, 0x7640, 0}, 	// CAM_PGA_L_CONFIG_GREEN_RED_Q14
		{SEQ_REG,      0xC964,  SEQ_WORD, 0x6478, 0}, 	// CAM_PGA_L_CONFIG_RED_Q14
		{SEQ_REG,      0xC966,  SEQ_WORD, 0x753C, 0}, 	// CAM_PGA_L_CONFIG_GREEN_BLUE_Q14
		{SEQ_REG,      0xC968,  SEQ_WORD, 0x7668, 0}, 	// CAM_PGA_L_CONFIG_BLUE_Q14
		{SEQ_REG,      0xC96A,  SEQ_WORD, 0x0FA0, 0}, 	// CAM_PGA_M_CONFIG_COLOUR_TEMP
		{SEQ_REG,      0xC96C,  SEQ_WORD, 0x7EE3, 0}, 	// CAM_PGA_M_CONFIG_GREEN_RED_Q14
		{SEQ_REG,      0xC96E,  SEQ_WORD, 0x7F88, 0}, 	// CAM_PGA_M_CONFIG_RED_Q14
		{SEQ_REG,      0xC970,  SEQ_WORD, 0x7F05, 0}, 	// CAM_PGA_M_CONFIG_GREEN_BLUE_Q14
		{SEQ_REG,      0xC972,  SEQ_WORD, 0x7F7E, 0}, 	// CAM_PGA_M_CONFIG_BLUE_Q14
		{SEQ_REG,      0xC974,  SEQ_WORD, 0x1964, 0}, 	// CAM_PGA_R_CONFIG_COLOUR_TEMP
		{SEQ_REG,      0xC976,  SEQ_WORD, 0x7BE2, 0}, 	// CAM_PGA_R_CONFIG_GREEN_RED_Q14
		{SEQ_REG,      0xC978,  SEQ_WORD, 0x7220, 0}, 	// CAM_PGA_R_CONFIG_RED_Q14
		{SEQ_REG,      0xC97A,  SEQ_WORD, 0x7D42, 0}, 	// CAM_PGA_R_CONFIG_GREEN_BLUE_Q14
		{SEQ_REG,      0xC97C,  SEQ_WORD, 0x76F4, 0}, 	// CAM_PGA_R_CONFIG_BLUE_Q14
		{SEQ_REG,      0xC95E,  SEQ_WORD, 0x0003, 0}, 	//  CAM_PGA_PGA_CONTROL

		{SEQ_REG,      0x3786,  SEQ_WORD, 0x0004, 0}, 	// PGA_Y_ADDR_START
		{SEQ_REG,      0x3788,  SEQ_WORD, 0x03CB, 0}, 	// PGA_Y_ADDR_END
		{SEQ_REG,      0x378A,  SEQ_WORD, 0x0004, 0}, 	// PGA_X_ADDR_START
		{SEQ_REG,      0x378C,  SEQ_WORD, 0x050B, 0}, 	// PGA_X_ADDR_END

		//[Step5-AWB_CCM]1: LOAD=CCM
		{SEQ_REG,      0xC892,  SEQ_WORD, 0x0267, 0}, 	// CAM_AWB_CCM_L_0
		{SEQ_REG,      0xC894,  SEQ_WORD, 0xFF1A, 0}, 	// CAM_AWB_CCM_L_1
		{SEQ_REG,      0xC896,  SEQ_WORD, 0xFFB3, 0}, 	// CAM_AWB_CCM_L_2
		{SEQ_REG,      0xC898,  SEQ_WORD, 0xFF80, 0}, 	// CAM_AWB_CCM_L_3
		{SEQ_REG,      0xC89A,  SEQ_WORD, 0x0166, 0}, 	// CAM_AWB_CCM_L_4
		{SEQ_REG,      0xC89C,  SEQ_WORD, 0x0003, 0}, 	// CAM_AWB_CCM_L_5
		{SEQ_REG,      0xC89E,  SEQ_WORD, 0xFF9A, 0}, 	// CAM_AWB_CCM_L_6
		{SEQ_REG,      0xC8A0,  SEQ_WORD, 0xFEB4, 0}, 	// CAM_AWB_CCM_L_7
		{SEQ_REG,      0xC8A2,  SEQ_WORD, 0x024D, 0}, 	// CAM_AWB_CCM_L_8
		{SEQ_REG,      0xC8A4,  SEQ_WORD, 0x01BF, 0}, 	// CAM_AWB_CCM_M_0
		{SEQ_REG,      0xC8A6,  SEQ_WORD, 0xFF01, 0}, 	// CAM_AWB_CCM_M_1
		{SEQ_REG,      0xC8A8,  SEQ_WORD, 0xFFF3, 0}, 	// CAM_AWB_CCM_M_2
		{SEQ_REG,      0xC8AA,  SEQ_WORD, 0xFF75, 0}, 	// CAM_AWB_CCM_M_3
		{SEQ_REG,      0xC8AC,  SEQ_WORD, 0x0198, 0}, 	// CAM_AWB_CCM_M_4
		{SEQ_REG,      0xC8AE,  SEQ_WORD, 0xFFFD, 0}, 	// CAM_AWB_CCM_M_5
		{SEQ_REG,      0xC8B0,  SEQ_WORD, 0xFF9A, 0}, 	// CAM_AWB_CCM_M_6
		{SEQ_REG,      0xC8B2,  SEQ_WORD, 0xFEE7, 0}, 	// CAM_AWB_CCM_M_7
		{SEQ_REG,      0xC8B4,  SEQ_WORD, 0x02A8, 0}, 	// CAM_AWB_CCM_M_8

		{SEQ_REG,      0xC8B6,  SEQ_WORD, 0x0100, 0}, 	// CAM_AWB_CCM_R_0
		{SEQ_REG,      0xC8B8,  SEQ_WORD, 0xFF84, 0}, 	// CAM_AWB_CCM_R_1
		{SEQ_REG,      0xC8BA,  SEQ_WORD, 0xFFEB, 0}, 	// CAM_AWB_CCM_R_2
		{SEQ_REG,      0xC8BC,  SEQ_WORD, 0xFFBD, 0}, 	// CAM_AWB_CCM_R_3
		{SEQ_REG,      0xC8BE,  SEQ_WORD, 0x0105, 0}, 	// CAM_AWB_CCM_R_4
		{SEQ_REG,      0xC8C0,  SEQ_WORD, 0xFFDB, 0}, 	// CAM_AWB_CCM_R_5
		{SEQ_REG,      0xC8C2,  SEQ_WORD, 0xFFCC, 0}, 	// CAM_AWB_CCM_R_6
		{SEQ_REG,      0xC8C4,  SEQ_WORD, 0xFE2A, 0}, 	// CAM_AWB_CCM_R_7
		{SEQ_REG,      0xC8C6,  SEQ_WORD, 0x03AB, 0}, 	// CAM_AWB_CCM_R_8

		{SEQ_REG,      0xC8DA,  SEQ_WORD, 0x004D, 0}, 	// CAM_AWB_LL_CCM_0
		{SEQ_REG,      0xC8DC,  SEQ_WORD, 0x0096, 0}, 	// CAM_AWB_LL_CCM_1
		{SEQ_REG,      0xC8DE,  SEQ_WORD, 0x001D, 0}, 	// CAM_AWB_LL_CCM_2
		{SEQ_REG,      0xC8E0,  SEQ_WORD, 0x004D, 0}, 	// CAM_AWB_LL_CCM_3
		{SEQ_REG,      0xC8E2,  SEQ_WORD, 0x0096, 0}, 	// CAM_AWB_LL_CCM_4
		{SEQ_REG,      0xC8E4,  SEQ_WORD, 0x001D, 0}, 	// CAM_AWB_LL_CCM_5
		{SEQ_REG,      0xC8E6,  SEQ_WORD, 0x004D, 0}, 	// CAM_AWB_LL_CCM_6
		{SEQ_REG,      0xC8E8,  SEQ_WORD, 0x0096, 0}, 	// CAM_AWB_LL_CCM_7
		{SEQ_REG,      0xC8EA,  SEQ_WORD, 0x001D, 0}, 	// CAM_AWB_LL_CCM_8


		//AWB gain setting
		{SEQ_REG,      0xC8C8,  SEQ_WORD, 0x0075, 0}, 	// CAM_AWB_CCM_L_RG_GAIN
		{SEQ_REG,      0xC8CA,  SEQ_WORD, 0x011C, 0}, 	// CAM_AWB_CCM_L_BG_GAIN
		{SEQ_REG,      0xC8CC,  SEQ_WORD, 0x00A0, 0}, 	// CAM_AWB_CCM_M_RG_GAIN
		{SEQ_REG,      0xC8CE,  SEQ_WORD, 0x00F3, 0}, 	// CAM_AWB_CCM_M_BG_GAIN
		{SEQ_REG,      0xC8D0,  SEQ_WORD, 0x00B3, 0}, 	// CAM_AWB_CCM_R_RG_GAIN
		{SEQ_REG,      0xC8D2,  SEQ_WORD, 0x009D, 0}, 	// CAM_AWB_CCM_R_BG_GAIN
		{SEQ_REG,      0xC8D4,  SEQ_WORD, 0x0A8C, 0}, 	// CAM_AWB_CCM_L_CTEMP
		{SEQ_REG,      0xC8D6,  SEQ_WORD, 0x1004, 0}, 	// CAM_AWB_CCM_M_CTEMP
		{SEQ_REG,      0xC8D8,  SEQ_WORD, 0x1964, 0}, 	// CAM_AWB_CCM_R_CTEMP

		// LOAD=AWB
		{SEQ_REG,      0xC914,  SEQ_WORD, 0x0000, 0}, 	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
		{SEQ_REG,      0xC916,  SEQ_WORD, 0x0000, 0}, 	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
		{SEQ_REG,      0xC918,  SEQ_WORD, 0x04FF, 0}, 	// CAM_STAT_AWB_CLIP_WINDOW_XEND
		{SEQ_REG,      0xC91A,  SEQ_WORD, 0x03BF, 0},	// CAM_STAT_AWB_CLIP_WINDOW_YEND

		{SEQ_REG,      0xC8F2,  SEQ_BYTE,   0x04, 0}, 	// CAM_AWB_AWB_XSCALE
		{SEQ_REG,      0xC8F3,  SEQ_BYTE,   0x02, 0}, 	// CAM_AWB_AWB_YSCALE

		// -- 20120912, Rev3, Alias
		{SEQ_REG,      0xC904,  SEQ_WORD, 0x0031, 0}, 	// CAM_AWB_AWB_XSHIFT_PRE_ADJ
		{SEQ_REG,      0xC906,  SEQ_WORD, 0x003D, 0}, 	// CAM_AWB_AWB_YSHIFT_PRE_ADJ

		{SEQ_REG,      0xC8F4,  SEQ_WORD, 0x0000, 0}, 	// CAM_AWB_AWB_WEIGHTS_0
		{SEQ_REG,      0xC8F6,  SEQ_WORD, 0x0000, 0}, 	// CAM_AWB_AWB_WEIGHTS_1
		{SEQ_REG,      0xC8F8,  SEQ_WORD, 0x0000, 0}, 	// CAM_AWB_AWB_WEIGHTS_2
		{SEQ_REG,      0xC8FA,  SEQ_WORD, 0xE724, 0}, 	// CAM_AWB_AWB_WEIGHTS_3
		{SEQ_REG,      0xC8FC,  SEQ_WORD, 0x1583, 0}, 	// CAM_AWB_AWB_WEIGHTS_4
		{SEQ_REG,      0xC8FE,  SEQ_WORD, 0x2045, 0}, 	// CAM_AWB_AWB_WEIGHTS_5
		{SEQ_REG,      0xC900,  SEQ_WORD, 0x05DC, 0}, 	// CAM_AWB_AWB_WEIGHTS_6
		{SEQ_REG,      0xC902,  SEQ_WORD, 0x007C, 0}, 	// CAM_AWB_AWB_WEIGHTS_7

		// fully white balance at A light
		{SEQ_REG,      0xC90A,  SEQ_WORD, 0x1004, 0}, 	// CAM_AWB_TINTS_CTEMP_THRESHOLD
		{SEQ_REG,      0xC90C,  SEQ_BYTE,   0x7E, 0}, 	// CAM_AWB_K_R_L
		{SEQ_REG,      0xC90D,  SEQ_BYTE,   0x86, 0}, 	// CAM_AWB_K_G_L
		{SEQ_REG,      0xC90E,  SEQ_BYTE,   0x89, 0}, 	// CAM_AWB_K_B_L
		{SEQ_REG,      0xC90F,  SEQ_BYTE,   0x7A, 0}, 	// CAM_AWB_K_R_R
		{SEQ_REG,      0xC910,  SEQ_BYTE,   0x7B, 0}, 	// CAM_AWB_K_G_R
		{SEQ_REG,      0xC911,  SEQ_BYTE,   0x7C, 0}, 	// CAM_AWB_K_B_R

		// for LSC at DNP
		{SEQ_REG,      0xAC06,  SEQ_BYTE,   0x63, 0}, 	// AWB_R_RATIO_LOWER
		{SEQ_REG,      0xAC07,  SEQ_BYTE,   0x65, 0}, 	// AWB_R_RATIO_UPPER
		{SEQ_REG,      0xAC08,  SEQ_BYTE,   0x63, 0}, 	// AWB_B_RATIO_LOWER
		{SEQ_REG,      0xAC09,  SEQ_BYTE,   0x65, 0}, 	// AWB_B_RATIO_UPPER

		// LOAD=Step7-CPIPE_Preference
		// changed for LV6 noise level
		{SEQ_REG,      0xC926,  SEQ_WORD, 0x0020, 0}, 	// CAM_LL_START_BRIGHTNESS
		//
		{SEQ_REG,      0xC928,  SEQ_WORD, 0x009A, 0}, 	// CAM_LL_STOP_BRIGHTNESS
		{SEQ_REG,      0xC946,  SEQ_WORD, 0x0070, 0}, 	// CAM_LL_START_GAIN_METRIC
		{SEQ_REG,      0xC948,  SEQ_WORD, 0x00F3, 0}, 	// CAM_LL_STOP_GAIN_METRIC
		//
		// fix AE test at LV5~LV2
		{SEQ_REG,      0xC952,  SEQ_WORD, 0x0090, 0}, 	// CAM_LL_START_TARGET_LUMA_BM
		{SEQ_REG,      0xC954,  SEQ_WORD, 0x02C0, 0}, 	// CAM_LL_STOP_TARGET_LUMA_BM

		// Modified default saturation to 110%
		{SEQ_REG,      0xC95A,  SEQ_BYTE,   0x05, 0}, 	// CAM_SEQ_UV_COLOR_BOOST
		{SEQ_REG,      0xC92A,  SEQ_BYTE,   0x50, 0}, 	// CAM_LL_START_SATURATION
		{SEQ_REG,      0xC92B,  SEQ_BYTE,   0x3C, 0},	// CAM_LL_END_SATURATION
		{SEQ_REG,      0xC92C,  SEQ_BYTE,   0x00, 0}, 	// CAM_LL_START_DESATURATION
		{SEQ_REG,      0xC92D,  SEQ_BYTE,   0xFF, 0}, 	// CAM_LL_END_DESATURATION
		{SEQ_REG,      0xC92E,  SEQ_BYTE,   0x3C, 0}, 	// CAM_LL_START_DEMOSAIC

		// Modified Sharpness setting
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x02, 0}, 	// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,   0x06, 0}, 	// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0xC931,  SEQ_BYTE,   0x64, 0}, 	// CAM_LL_STOP_DEMOSAIC
		{SEQ_REG,      0xC932,  SEQ_BYTE,   0x01, 0}, 	// CAM_LL_STOP_AP_GAIN
		{SEQ_REG,      0xC933,  SEQ_BYTE,   0x0C, 0}, 	// CAM_LL_STOP_AP_THRESH
		{SEQ_REG,      0xC934,  SEQ_BYTE,   0x3C, 0}, 	// CAM_LL_START_NR_RED
		{SEQ_REG,      0xC935,  SEQ_BYTE,   0x3C, 0}, 	// CAM_LL_START_NR_GREEN
		{SEQ_REG,      0xC936,  SEQ_BYTE,   0x3C, 0}, 	// CAM_LL_START_NR_BLUE
		{SEQ_REG,      0xC937,  SEQ_BYTE,   0x0F, 0}, 	// CAM_LL_START_NR_THRESH
		{SEQ_REG,      0xC938,  SEQ_BYTE,   0x64, 0}, 	// CAM_LL_STOP_NR_RED
		{SEQ_REG,      0xC939,  SEQ_BYTE,   0x64, 0}, 	// CAM_LL_STOP_NR_GREEN
		{SEQ_REG,      0xC93A,  SEQ_BYTE,   0x64, 0}, 	// CAM_LL_STOP_NR_BLUE
		{SEQ_REG,      0xC93B,  SEQ_BYTE,   0x32, 0}, 	// CAM_LL_STOP_NR_THRESH
		//
		{SEQ_REG,      0xC93C,  SEQ_WORD, 0x0005, 0}, 	// CAM_LL_START_CONTRAST_BM
		{SEQ_REG,      0xC93E,  SEQ_WORD, 0x0358, 0}, 	// CAM_LL_STOP_CONTRAST_BM
		//
		{SEQ_REG,      0xC940,  SEQ_WORD, 0x00DC, 0}, 	// CAM_LL_GAMMA
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x3C, 0},	// CAM_LL_START_CONTRAST_GRADIENT
		{SEQ_REG,      0xC943,  SEQ_BYTE,   0x30, 0}, 	// CAM_LL_STOP_CONTRAST_GRADIENT
		{SEQ_REG,      0xC944,  SEQ_BYTE,   0x22, 0},	// CAM_LL_START_CONTRAST_LUMA_PERCENTAGE
		{SEQ_REG,      0xC945,  SEQ_BYTE,   0x19, 0}, 	// CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE
		//
		// changed for fixing FTB
		{SEQ_REG,      0xC94A,  SEQ_WORD, 0x00F0, 0}, 	// CAM_LL_START_FADE_TO_BLACK_LUMA
		//
		{SEQ_REG,      0xC94C,  SEQ_WORD, 0x0010, 0}, 	// CAM_LL_STOP_FADE_TO_BLACK_LUMA
		{SEQ_REG,      0xC94E,  SEQ_WORD, 0x01CD, 0}, 	// CAM_LL_CLUSTER_DC_TH_BM
		{SEQ_REG,      0xC950,  SEQ_BYTE,   0x05, 0}, 	// CAM_LL_CLUSTER_DC_GATE_PERCENTAGE
		{SEQ_REG,      0xC951,  SEQ_BYTE,   0x40, 0}, 	// CAM_LL_SUMMING_SENSITIVITY_FACTOR
		//

		// for AE to 135 and low light between 80~180 at color checker block 18.
		{SEQ_REG,      0xC87A,  SEQ_BYTE,   0x38, 0}, 	// CAM_AET_TARGET_AVERAGE_LUMA
		{SEQ_REG,      0xC87B,  SEQ_BYTE,   0x0E, 0}, 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK

		{SEQ_REG,      0xC878,  SEQ_BYTE,   0x0E, 0}, 	// CAM_AET_AEMODE  -- for DR test
		// fix noise at LV6
		{SEQ_REG,      0xC890,  SEQ_WORD, 0x0040, 0}, 	// CAM_AET_TARGET_GAIN

		// Increase AE_ vir_digit_gain for low light test on device
		{SEQ_REG,      0xC882,  SEQ_WORD, 0x0100, 0}, 	// CAM_AET_AE_MAX_VIRT_DGAIN
		//
		{SEQ_REG,      0xC886,  SEQ_WORD, 0x0100, 0}, 	// CAM_AET_AE_MAX_VIRT_AGAIN
		{SEQ_REG,      0xA404, 0x02, 0}, 	// AE_RULE_ALGO
		//
		{SEQ_REG,      0xC87C,  SEQ_WORD, 0x0016, 0},	// CAM_AET_BLACK_CLIPPING_TARGET
		//
		{SEQ_REG,      0xB42A,  SEQ_BYTE,   0x05, 0}, 	// CCM_DELTA_GAIN

		// Improve AE Tracking speed
		{SEQ_REG,      0xA80A,  SEQ_BYTE,   0x18, 0}, 	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED

		// Increase sharpness (2D-Aperture correction)
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1706, 0}, 	// APERTURE_PARAMETERS_2D

		// LOAD=Step8-Features
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x0000, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC984,  SEQ_WORD, 0x8000, 0}, 	// CAM_PORT_OUTPUT_CONTROL
		{SEQ_REG,      0x001E,  SEQ_WORD, 0x0777, 0}, 	// PAD_SLEW	//<ASUS-Vincent_Liu-20130425111101>


		// LOAD=Dual Lane Mipi Receiver Init
		// LOAD=Enter Suspend
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x40, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0}, 	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100}, // Polling R0x0080[1] till "0".

		//;REG= 0xDC00, 0x34 	// SYSMGR_NEXT_STATE
		//;REG= 0x0080, 0x8002 	// COMMAND_REGISTER
		//;POLL_FIELD=COMMAND_REGISTER, HOST_COMMAND_1, !=0, DELAY=10, TIMEOUT=100		// Polling R0x0080[1] till "0".


		// following two lines of setting are for dual lane MIPI Rx board used only,
		// Please enable your MIPI Rx here or make sure it is ready for receiving data
		// [Dual Lane Mipi Receiver Init]3:
		//;DELAY=500
		//SERIAL_REG=0xCA, 0x00, 0x8000, 8:16
		//SERIAL_REG=0xCA, 0x00, 0x0006, 8:16
		//;DELAY=500

		//LOAD=Leave Suspend
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x34, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0}, 	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100},	// Polling R0x0080[1] till "0".

		{SEQ_REG,      0x098E,  SEQ_WORD, 0x2802, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC908,  SEQ_BYTE,   0x01, 0}, 	// CAM_AWB_SKIP_FRAMES
		{SEQ_REG,      0xC879,  SEQ_BYTE,   0x01, 0}, 	// CAM_AET_SKIP_FRAMES
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x02, 0}, 	// CAM_AWB_AWBMODE

		// Improve AE Dampening speed
		{SEQ_REG,      0xA80B,  SEQ_BYTE,   0x18, 0},	// AE_TRACK_AE_DAMPENING_SPEED
		//

		{SEQ_REG,      0xAC16,  SEQ_BYTE,   0x18, 0}, 	// AWB_PRE_AWB_RATIOS_TRACKING_SPEED
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0}, 	// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,  SEQ_WORD, 0x0002, 100},	// Polling R0x0080[1] till "0".
		
		//State= Detect Master Clock, 1		// Detect system clock
		//image= 1280, 960, YCBCR
		//;delay=50

		{SEQ_MS,      0,  SEQ_WORD, 0, 50},
		//end of initialization
		{SEQ_END,           0,         0,      0, 0}
};

#if CONFIG_SENSOR_AntiBanding
static	struct seq_info sensor_antibandingAuto[] = 
{
	//	[Auto FD]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0xC000, 0},	// LOGICAL_ADDRESS_ACCESS [FLICKER_DETECT_MODE]
	{SEQ_REG,	   0xC000,	SEQ_BYTE, 0x03, 0},	// FLICKER_DETECT_MODE
	{SEQ_END,           0,         0,      0, 0}
};

static	struct seq_info sensor_antibanding50hz[] = 
{
	//	[Disable Auto FD and Set flicker frequency-50Hz]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0xC000, 0},	// LOGICAL_ADDRESS_ACCESS [CAM_AET_FLICKER_FREQ_HZ]
	{SEQ_REG,	   0xC000,	SEQ_BYTE, 0x00, 0},	// FLICKER_DETECT_MODE
	{SEQ_REG,	   0xC88B,	SEQ_BYTE, 0x32, 0}, // CAM_AET_FLICKER_FREQ_HZ
	{SEQ_REG,	   0xDC00,	SEQ_BYTE, 0x28, 0}, // SYSMGR_NEXT_STATE
	{SEQ_REG,	   0x0080,	SEQ_WORD, 0x8002, 10}, // COMMAND_REGISTER
	{SEQ_END,           0,         0,      0, 0}
};

static	struct seq_info sensor_antibanding60hz[] = 
{
	//	[Disable Auto FD and Set flicker frequency-60Hz]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0xC000, 0},	// LOGICAL_ADDRESS_ACCESS [CAM_AET_FLICKER_FREQ_HZ]
	{SEQ_REG,	   0xC000,	SEQ_BYTE, 0x00, 0},	// FLICKER_DETECT_MODE
	{SEQ_REG,	   0xC88B,	SEQ_BYTE, 0x3C, 0}, // CAM_AET_FLICKER_FREQ_HZ
	{SEQ_REG,	   0xDC00,	SEQ_BYTE, 0x28, 0}, // SYSMGR_NEXT_STATE
	{SEQ_REG,	   0x0080,	SEQ_WORD, 0x8002, 10}, // COMMAND_REGISTER
	{SEQ_END,           0,         0,      0, 0}
};
static struct seq_info *sensor_antibanding[] = {sensor_antibanding50hz, sensor_antibanding60hz,NULL,};
#endif
#if	CONFIG_SENSOR_ExposureLock
static	struct seq_info sensor_ExposureLock[] = 
{
	//	[Disable AE]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0x2804, 0},	// LOGICAL_ADDRESS_ACCESS [AE_TRACK_ALGO]
	{SEQ_REG,	   0xA804,	SEQ_WORD, 0x0000, 0},	//AE_TRACK_ALGO
	{SEQ_END,           0,         0,      0, 0}
};

static	struct seq_info sensor_ExposureUnLock[] = 
{
	//	[Enable AE]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0x2804, 0},	// LOGICAL_ADDRESS_ACCESS [AE_TRACK_ALGO]
	{SEQ_REG,	   0xA804,	SEQ_WORD, 0x00FF, 0},	// AE_TRACK_ALGO
	{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info *sensor_ExposureLk[] = {sensor_ExposureUnLock, sensor_ExposureLock, NULL,};
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
static	struct seq_info sensor_WhiteBalanceLock[] = 
{
	//	[Disable AWB]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0xC909, 0},	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{SEQ_REG,	   0xC909,	SEQ_BYTE, 0x00, 0}, // CAM_AWB_AWBMODE
	{SEQ_END,           0,         0,      0, 0}
};

static	struct seq_info sensor_WhiteBalanceUnLock[] = 
{
	//	[Enable AWB]
	// ops		   reg		 len	  val	 tmo
	{SEQ_REG,	   0x098E,	SEQ_WORD, 0xC909, 0},	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_AWBMODE]
	{SEQ_REG,	   0xC909,	SEQ_BYTE, 0x03, 0},	// CAM_AWB_AWBMODE
	{SEQ_END,           0,         0,      0, 0}
};
static struct seq_info *sensor_WhiteBalanceLk[] = {sensor_WhiteBalanceUnLock, sensor_WhiteBalanceLock, NULL,};
#endif
#if CONFIG_SENSOR_Effect
//[2. Effects APIs]
//[2.1 Normal -- default]
static struct seq_info sensor_effects_default[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0},	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x00, 0},	// CAM_SFX_CONTROL
		{SEQ_BIT_SET,  0xDC00,  SEQ_BYTE,   0x28, 0},	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0},	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

//[2.2 Mono color effect]
static struct seq_info sensor_effects_mono[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x01, 0}, 	// CAM_SFX_CONTROL
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0}, 	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

//[2.3 Sepia effect]
static struct seq_info sensor_effects_sepia[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x02, 0}, 	// CAM_SFX_CONTROL
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0}, 	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

//[2.4 Negative effect]
static struct seq_info sensor_effects_negative[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x03, 0}, 	// CAM_SFX_CONTROL
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0}, 	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

//[2.5 Solarize 1]
static struct seq_info sensor_effects_solarize1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x04, 0}, 	// CAM_SFX_CONTROL
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0}, 	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

//[2.6 Solarize 2]
static struct seq_info sensor_effects_solarize2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC874, 0}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SFX_CONTROL]
		{SEQ_REG,      0xC874,  SEQ_BYTE,   0x05, 0}, 	// CAM_SFX_CONTROL
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x28, 0}, 	// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8004, 0}, 	// COMMAND_REGISTER
		{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info* sensor_effects_seqs[]=
{
		sensor_effects_default,
		sensor_effects_mono,
		sensor_effects_negative,
		sensor_effects_sepia,
		sensor_effects_solarize1,
		sensor_effects_solarize2,
		NULL
};
#endif	// CONFIG_SENSOR_Effect


#if CONFIG_SENSOR_Exposure
//[3. EV adjustment APIs]
#if ASUS_CAMERA_SUPPORT
//[3.1 EV-6: 75]
static struct seq_info sensor_exposure_n6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     31, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.2 EV-5: 85]
static struct seq_info sensor_exposure_n5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     34, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.3 EV-4: 95]
static struct seq_info sensor_exposure_n4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     37, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};
#endif
//[3.4 EV-3: 105]
static struct seq_info sensor_exposure_n3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     42, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.5 EV-2: 115]
static struct seq_info sensor_exposure_n2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     46, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.6 EV-1: 125]
static struct seq_info sensor_exposure_n1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     50, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.7 EV0: 135 -- default]
static struct seq_info sensor_exposure_0[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     55, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.8 EV+1: 145]
static struct seq_info sensor_exposure_p1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     63, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.9 EV+2: 155]
static struct seq_info sensor_exposure_p2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     71, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.10 EV+3: 165]
static struct seq_info sensor_exposure_p3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     78, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};
#if ASUS_CAMERA_SUPPORT
//[3.11 EV+4: 175]
static struct seq_info sensor_exposure_p4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     85, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.12 EV+5: 185]
static struct seq_info sensor_exposure_p5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,     95, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};

//[3.13 EV+6: 195]
static struct seq_info sensor_exposure_p6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C0A, 0}, 	// LOGICAL_ADDRESS_ACCESS [UVC_BRIGHTNESS_CONTROL]
		{SEQ_REG,      0xCC0A,  SEQ_WORD,    110, 0}, 	// UVC_BRIGHTNESS_CONTROL
		{SEQ_END,           0,         0,      0, 0}
};
#endif
static struct seq_info *sensor_exposure_seqs[] =
{
#if ASUS_CAMERA_SUPPORT
		sensor_exposure_n6,
		sensor_exposure_n5,
		sensor_exposure_n4,
#endif
		sensor_exposure_n3,
		sensor_exposure_n2,
		sensor_exposure_n1,
		sensor_exposure_0,
		sensor_exposure_p1,
		sensor_exposure_p2,
		sensor_exposure_p3,
#if ASUS_CAMERA_SUPPORT
		sensor_exposure_p4,
		sensor_exposure_p5,
		sensor_exposure_p6,
#endif
		NULL
};
#endif	// CONFIG_SENSOR_Exposure

#if CONFIG_SENSOR_WhiteBalance
//[4. AWB/MWB APIs]
//[4.1 AWB -- default]
static struct seq_info sensor_wb_default[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC909, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x03, 0}, 	// CAM_AWB_AWBMODE
		{SEQ_REG,      0xAC04,  SEQ_WORD, 0x0288, 0},	// AWB_ALGO
		{SEQ_END,           0,         0,      0, 0}
};

//[4.2 MWB: Incandescent- 2800K]
static struct seq_info sensor_wb_incandescent[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC909, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x00, 0},	// CAM_AWB_AWBMODE
		{SEQ_REG,      0xAC04,  SEQ_WORD, 0x0288, 0}, 	// AWB_ALGO
		{SEQ_REG,      0xC8F0,  SEQ_WORD, 0x09C4, 0}, 	// CAM_AWB_COLOR_TEMPERATURE
		{SEQ_END,           0,         0,     0, 0}
};

//[4.3 MWB: Fluorescent- 4005K]
static struct seq_info sensor_wb_fluorescent[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC909, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x00, 0},	// CAM_AWB_AWBMODE
		{SEQ_REG,      0xAC04,  SEQ_WORD, 0x0288, 0}, 	// AWB_ALGO
		{SEQ_REG,      0xC8F0,  SEQ_WORD, 0x0D67, 0}, 	// CAM_AWB_COLOR_TEMPERATURE
		{SEQ_END,           0,         0,     0, 0}
};

//[4.4 MWB: Day Light- 4811K]
static struct seq_info sensor_wb_daylight[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC909, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x00, 0},	// CAM_AWB_AWBMODE
		{SEQ_REG,      0xAC04,  SEQ_WORD, 0x0288, 0}, 	// AWB_ALGO
		{SEQ_REG,      0xC8F0,  SEQ_WORD, 0x1964, 0}, 	// CAM_AWB_COLOR_TEMPERATURE
		{SEQ_END,           0,         0,      0, 0}
};

//[4.5 MWB: Cloudy- 6500K]
static struct seq_info sensor_wb_cloudy[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC909, 0}, 	// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xC909,  SEQ_BYTE,   0x00, 0},	// CAM_AWB_AWBMODE
		{SEQ_REG,      0xAC04,  SEQ_WORD, 0x0208, 0}, 	// AWB_ALGO
		{SEQ_REG,      0xC8F0,  SEQ_WORD, 0x1964, 0}, 	// CAM_AWB_COLOR_TEMPERATURE
		{SEQ_REG,      0xAC12,  SEQ_WORD, 0x00B4, 0}, 	// AWB_R_GAIN
		{SEQ_REG,      0xAC14,  SEQ_WORD, 0x0080, 0}, 	// AWB_B_GAIN
		{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info *sensor_wb_seqs[] =
{
		sensor_wb_default,
		sensor_wb_incandescent,
		sensor_wb_fluorescent,
		sensor_wb_daylight,
		sensor_wb_cloudy,
		NULL,
};
#endif	// CONFIG_SENSOR_WhiteBalance

#if CONFIG_SENSOR_Saturation
//[5. Saturation adjustment APIs]
//[5.1 Saturation +6]
static struct seq_info sensor_saturation_p6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		/// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0078, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x07, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.2 Saturation +5]
static struct seq_info sensor_saturation_p5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x008E, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x05, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.3 Saturation +4]
static struct seq_info sensor_saturation_p4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x007E, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x05, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.4 Saturation +3]
static struct seq_info sensor_saturation_p3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x008A, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x04, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.5 Saturation +2]
static struct seq_info sensor_saturation_p2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x007C, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x04, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.6 Saturation +1]
static struct seq_info sensor_saturation_p1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x008E, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.7 Saturation 0: default]
static struct seq_info sensor_saturation_0[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0080, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.8 Saturation -1]
static struct seq_info sensor_saturation_n1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0073, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.9 Saturation -2]
static struct seq_info sensor_saturation_n2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0064, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.10 Saturation -3]
static struct seq_info sensor_saturation_n3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0055, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.11 Saturation -4]
static struct seq_info sensor_saturation_n4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0046, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.12 Saturation -5]
static struct seq_info sensor_saturation_n5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x0038, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

//[5.13 Saturation -6]
static struct seq_info sensor_saturation_n6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0x4C12, 0},		// LOGICAL_ADDRESS_ACCESS
		{SEQ_REG,      0xCC12,  SEQ_WORD, 0x002D, 0},		// UVC_SATURATION_CONTROL
		{SEQ_REG,      0xC95A,  SEQ_BYTE,	0x03, 0},		// CAM_SEQ_UV_COLOR_BOOST
 		{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info *sensor_saturation_seqs[] =
{
		sensor_saturation_n6,
		sensor_saturation_n5,
		sensor_saturation_n4,
		sensor_saturation_n3,
		sensor_saturation_n2,
		sensor_saturation_n1,
		sensor_saturation_0,
		sensor_saturation_p1,
		sensor_saturation_p2,
		sensor_saturation_p3,
		sensor_saturation_p4,
		sensor_saturation_p5,
		sensor_saturation_p6,
		NULL,
};
#endif

#if CONFIG_SENSOR_Sharpness
//[6. Sharpness adjustment APIs]
//[6.1 Sharpness +6]
static struct seq_info sensor_sharpness_p6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x04, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x18, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x3418, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.2 Sharpness +5]
static struct seq_info sensor_sharpness_p5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x03, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x18, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x3318, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.3 Sharpness +4]
static struct seq_info sensor_sharpness_p4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x05, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x10, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x2510, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.4 Sharpness +3]
static struct seq_info sensor_sharpness_p3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x07, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x10, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1F10, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.5 Sharpness +2]
static struct seq_info sensor_sharpness_p2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x05, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x10, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1D10, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.6 Sharpness +1]
static struct seq_info sensor_sharpness_p1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x06, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x0C, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x160C, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.7 Sharpness 0: default]
static struct seq_info sensor_sharpness_0[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x04, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x0C, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x140C, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.8 Sharpness -1]
static struct seq_info sensor_sharpness_n1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x03, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1306, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.9 Sharpness -2]
static struct seq_info sensor_sharpness_n2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x02, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1206, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.10 Sharpness -3]
static struct seq_info sensor_sharpness_n3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x01, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1106, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.11 Sharpness -4]
static struct seq_info sensor_sharpness_n4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x00, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1006, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.12 Sharpness -5]
static struct seq_info sensor_sharpness_n5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x00, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1006, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

//[6.13 Sharpness -6]
static struct seq_info sensor_sharpness_n6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC92F, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_AP_GAIN]
		{SEQ_REG,      0xC92F,  SEQ_BYTE,   0x00, 0},		// CAM_LL_START_AP_GAIN
		{SEQ_REG,      0xC930,  SEQ_BYTE,	0x06, 0},		// CAM_LL_START_AP_THRESH
		{SEQ_REG,      0x326C,  SEQ_WORD, 0x1006, 0},		// APERTURE_PARAMETERS_2D
		{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info *sensor_sharpness_seqs[] =
{
	sensor_sharpness_n6,
	sensor_sharpness_n5,
	sensor_sharpness_n4,
	sensor_sharpness_n3,
	sensor_sharpness_n2,
	sensor_sharpness_n1,
	sensor_sharpness_0,
	sensor_sharpness_p1,
	sensor_sharpness_p2,
	sensor_sharpness_p3,
	sensor_sharpness_p4,
	sensor_sharpness_p5,
	sensor_sharpness_p6,
	NULL,
};
#endif	// CONFIG_SENSOR_Sharpness

#if CONFIG_SENSOR_Contrast
//[7. Contrast adjustment APIs]
//[7.1 Contrast +6]
static struct seq_info sensor_contrast_p6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x60, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 96
		{SEQ_END,           0,         0,      0, 0}
};

//[7.2 Contrast +5]
static struct seq_info sensor_contrast_p5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x5A, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 90
		{SEQ_END,           0,         0,      0, 0}
};

//[7.3 Contrast +4]
static struct seq_info sensor_contrast_p4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x54, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 84
		{SEQ_END,           0,         0,      0, 0}
};

//[7.4 Contrast +3]
static struct seq_info sensor_contrast_p3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x4E, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 78
		{SEQ_END,           0,         0,      0, 0}
};

//[7.5 Contrast +2]
static struct seq_info sensor_contrast_p2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x48, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 72
		{SEQ_END,           0,         0,      0, 0}
};

//[7.6 Contrast +1]
static struct seq_info sensor_contrast_p1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x42, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 66
		{SEQ_END,           0,         0,      0, 0}
};

//[7.7 Contrast 0: default]
static struct seq_info sensor_contrast_0[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x3C, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 60
		{SEQ_END,           0,         0,      0, 0}
};

//[7.8 Contrast -1]
static struct seq_info sensor_contrast_n1[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x36, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 54
		{SEQ_END,           0,         0,      0, 0}
};

//[7.9 Contrast -2]
static struct seq_info sensor_contrast_n2[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x30, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 48
		{SEQ_END,           0,         0,      0, 0}
};

//[7.10 Contrast -3]
static struct seq_info sensor_contrast_n3[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x2A, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 42
		{SEQ_END,           0,         0,      0, 0}
};

//[7.11 Contrast -4]
static struct seq_info sensor_contrast_n4[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x24, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 36
		{SEQ_END,           0,         0,      0, 0}
};

//[7.12 Contrast -5]
static struct seq_info sensor_contrast_n5[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x1E, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 30
		{SEQ_END,           0,         0,      0, 0}
};

//[7.13 Contrast -6]
static struct seq_info sensor_contrast_n6[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xC942, 0},		// LOGICAL_ADDRESS_ACCESS [CAM_LL_START_CONTRAST_GRADIENT]
		{SEQ_REG,      0xC942,  SEQ_BYTE,   0x18, 0},		// CAM_LL_START_CONTRAST_GRADIENT -- 24
		{SEQ_END,           0,         0,      0, 0}
};

static struct seq_info *sensor_contrast_seqs[] =
{
	sensor_contrast_n6,
	sensor_contrast_n5,
	sensor_contrast_n4,
	sensor_contrast_n3,
	sensor_contrast_n2,
	sensor_contrast_n1,
	sensor_contrast_0,
	sensor_contrast_p1,
	sensor_contrast_p2,
	sensor_contrast_p3,
	sensor_contrast_p4,
	sensor_contrast_p5,
	sensor_contrast_p6,
	NULL,
};
#endif

#if CONFIG_SENSOR_Mirror

#endif

#if CONFIG_SENSOR_Scene

#endif

#if CONFIG_SENSOR_DigitalZoom
#endif

#if 0
//[Enter Standby]
static struct seq_info sensor_enter_standby[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xDC00, 0},		// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x50, 0},		// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0},		// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,	SEQ_WORD, 0x0002, 100},		// Polling R0x0080[1] till "0".
		{SEQ_POLL_CLR, 0xDC01,	SEQ_BYTE,   0x52, 50},		// POLL_REG= 0xDC01,0xFF, !=0x52,DELAY=50,TIMEOUT=10
 		{SEQ_END,           0,         0,      0, 0}
};

//[Exit Standby]
static struct seq_info sensor_exit_standby[] =
{
		// ops         reg       len      val    tmo
		{SEQ_REG,      0x098E,  SEQ_WORD, 0xDC00, 0},		// LOGICAL_ADDRESS_ACCESS [SYSMGR_NEXT_STATE]
		{SEQ_REG,      0xDC00,  SEQ_BYTE,   0x54, 0},		// SYSMGR_NEXT_STATE
		{SEQ_REG,      0x0080,  SEQ_WORD, 0x8002, 0},		// COMMAND_REGISTER
		{SEQ_POLL_CLR, 0x0080,	SEQ_WORD, 0x0002, 100},		// Polling R0x0080[1] till "0".
		{SEQ_END,           0,         0,      0, 0}
};
#endif

static const struct v4l2_querymenu sensor_menus[] =
{
#if CONFIG_SENSOR_WhiteBalance
	{.id = V4L2_CID_DO_WHITE_BALANCE, .index = 0, .name = "auto",			.reserved = 0, },
	{.id = V4L2_CID_DO_WHITE_BALANCE, .index = 1, .name = "incandescent",	.reserved = 0,},
	{.id = V4L2_CID_DO_WHITE_BALANCE, .index = 2, .name = "fluorescent",	.reserved = 0,},
	{.id = V4L2_CID_DO_WHITE_BALANCE, .index = 3, .name = "daylight",		.reserved = 0,},
	{.id = V4L2_CID_DO_WHITE_BALANCE, .index = 4, .name = "cloudy-daylight",	.reserved = 0,},
#endif

#if CONFIG_SENSOR_Effect
	{.id = V4L2_CID_EFFECT,	.index = 0,	.name = "none",		.reserved = 0,},
	{.id = V4L2_CID_EFFECT,	.index = 1,	.name = "mono",		.reserved = 0,},
	{.id = V4L2_CID_EFFECT,	.index = 2,	.name = "negative",	.reserved = 0,},
	{.id = V4L2_CID_EFFECT,	.index = 3,	.name = "sepia",	.reserved = 0,},
	{.id = V4L2_CID_EFFECT,	.index = 4,	.name = "solarize",	.reserved = 0,},
#endif

	#if CONFIG_SENSOR_Scene
    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_AntiBanding
    { .id = V4L2_CID_ANTIBANDING,  .index = 0, .name = "50hz", .reserved = 0,} ,{ .id = V4L2_CID_ANTIBANDING,  .index = 1, .name = "60hz", .reserved = 0,},
	#endif

	#if CONFIG_SENSOR_ISO
	{ .id = V4L2_CID_ISO,	.index = 0, .name = "auto", .reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 1, .name = "100",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 2, .name = "200",	.reserved = 0, },
	#endif

	#if CONFIG_SENSOR_Flash
    { .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
    { .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {  .id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
    #endif
};

static  struct v4l2_queryctrl sensor_controls[] =
{
#if CONFIG_SENSOR_WhiteBalance
	{
		.id		= V4L2_CID_DO_WHITE_BALANCE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "White Balance Control",
		.minimum	= 0,
		.maximum	= 4,
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_Brightness
	{
		.id		= V4L2_CID_BRIGHTNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Brightness Control",
		.minimum	= -3,
		.maximum	= 2,
		.step		= 1,
		.default_value = 0,
	},
#endif

#if CONFIG_SENSOR_Effect
	{
		.id		= V4L2_CID_EFFECT,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Effect Control",
		.minimum	= 0,
		.maximum	= 4,
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_Exposure
	{
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure Control",
		#if ASUS_CAMERA_SUPPORT
		.minimum	= -6,
		.maximum	= 6,
		#else
		.minimum	= -3,
		.maximum	= 3,
		#endif
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_Saturation
	{
		.id		= V4L2_CID_SATURATION,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Saturation Control",
		.minimum	= -6,
		.maximum	= 6,
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_Contrast
	{
		.id		= V4L2_CID_CONTRAST,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Contrast Control",
		.minimum	= -6,
		.maximum	= 6,
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_Mirror
	{
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Mirror Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 1,
	},
#endif

#if CONFIG_SENSOR_Flip
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	},
#endif

#if CONFIG_SENSOR_Scene
	{
		.id		= V4L2_CID_SCENE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Scene Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
#endif
#if CONFIG_SENSOR_AntiBanding
	{
		.id 	= V4L2_CID_ANTIBANDING,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Antibanding Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
#endif
#if	CONFIG_SENSOR_WhiteBalanceLock
	{
		.id 	= V4L2_CID_WHITEBALANCE_LOCK,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "WhiteBalanceLock Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
#endif
#if CONFIG_SENSOR_ExposureLock
	{
		.id 	= V4L2_CID_EXPOSURE_LOCK,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "ExposureLock Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
#endif

#if CONFIG_SENSOR_ISO
	{
		.id 	= V4L2_CID_ISO,
		.type		= V4L2_CTRL_TYPE_MENU,
		.minimum	= 0,
		.maximum	= 2,
		.step		= 1,
		.default_value	= 0,
	},
#endif

#if CONFIG_SENSOR_JPEG_EXIF
	{
		.id 	= V4L2_CID_JPEG_EXIF,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "ExposureTime Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
#endif

#if CONFIG_SENSOR_DigitalZoom
	{
	.id		= V4L2_CID_ZOOM_RELATIVE,
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.name		= "DigitalZoom Control",
	.minimum	= -1,
	.maximum	= 1,
	.step		= 1,
	.default_value = 0,
	}, {
	.id		= V4L2_CID_ZOOM_ABSOLUTE,
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.name		= "DigitalZoom Control",
	.minimum	= 0,
	.maximum	= 3,
	.step		= 1,
	.default_value = 0,
	},
#endif

#if CONFIG_SENSOR_Focus
	{
	.id		= V4L2_CID_FOCUS_RELATIVE,
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.name		= "Focus Control",
	.minimum	= -1,
	.maximum	= 1,
	.step		= 1,
	.default_value = 0,
	}, {
	.id		= V4L2_CID_FOCUS_ABSOLUTE,
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.name		= "Focus Control",
	.minimum	= 0,
	.maximum	= 255,
	.step		= 1,
	.default_value = 125,
	},
	{
	.id		= V4L2_CID_FOCUS_AUTO,
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.name		= "Focus Control",
	.minimum	= 0,
	.maximum	= 1,
	.step		= 1,
	.default_value = 0,
	},{
	.id		= V4L2_CID_FOCUS_CONTINUOUS,
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.name		= "Focus Control",
	.minimum	= 0,
	.maximum	= 1,
	.step		= 1,
	.default_value = 0,
	},
#endif

#if CONFIG_SENSOR_Flash
	{
		.id		= V4L2_CID_FLASH,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Flash Control",
		.minimum	= 0,
		.maximum	= 2,
		//.maximum	= 3
		.step		= 1,
		.default_value = 0,
	},
#endif

#if	CONFIG_SENSOR_Sharpness
	{
		.id		= V4L2_CID_SHARPNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Sharpness Control",
		.minimum	= -6,
		.maximum	= 6,
		.step		= 1,
		.default_value	= 0,
	},
#endif
};

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *did);
static int sensor_video_probe(struct soc_camera_device *icd, struct i2c_client *client);
static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_g_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_s_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg);
static int sensor_resume(struct soc_camera_device *icd);
static int sensor_set_bus_param(struct soc_camera_device *icd,unsigned long flags);
static unsigned long sensor_query_bus_param(struct soc_camera_device *icd);
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
static int sensor_deactivate(struct i2c_client *client);

static struct soc_camera_ops sensor_ops =
{
	.suspend		= sensor_suspend,
	.resume			= sensor_resume,
	.set_bus_param		= sensor_set_bus_param,
	.query_bus_param	= sensor_query_bus_param,
	.controls		= sensor_controls,
	.menus			= sensor_menus,
	.num_controls		= ARRAY_SIZE(sensor_controls),
	.num_menus		= ARRAY_SIZE(sensor_menus),
};

/* Find a data format by a pixel code in an array */
static const struct sensor_datafmt *sensor_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct sensor_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct sensor_datafmt sensor_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
// TODO
//	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}
};

static struct sensor* to_sensor(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct sensor, subdev);
}

#define SWAP16(d16) (u16)((((d16) & 0xFF) << 8) | ((d16) >> 8 & 0xFF))
#define SWAP32(d32) (u32)((((d32) & 0xFF) << 24) | ((d32) >> 24 & 0xFF) | (((d32) & 0xff00) << 8) | ((d32) & 0xff0000) >> 8)

static int i2c_send_trans(struct i2c_client *client, u8 *buf, u16 len)
{
	struct i2c_msg msg;
	int err = -EAGAIN;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len ;
	msg.buf = buf;
	msg.scl_rate = CONFIG_SENSOR_I2C_SPEED;         /* ddl@rock-chips.com : 100kHz */
	err = i2c_transfer(client->adapter, &msg, 1);
	return (err == 1) ? len : err;
};

static int sensor_read8(struct i2c_client *client, const u16 regs, u8 *buf)
{
	struct i2c_msg msgs[2];
	u16 r = SWAP16(regs);
	int err = -EAGAIN;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = sizeof(u16);
	msgs[0].buf = (char*)&r;
	msgs[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[0].udelay = client->udelay;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = sizeof(u8);
	msgs[1].buf = (char *)buf;
	msgs[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[1].udelay = client->udelay;

	err = i2c_transfer(client->adapter, msgs, 2);
	return err;
};

static int sensor_read16(struct i2c_client *client, const u16 regs, u16 *buf)
{
	struct i2c_msg msgs[2];
	u16 r = SWAP16(regs);
	int err = -EAGAIN;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = sizeof(u16);
	msgs[0].buf = (char*)&r;
	msgs[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[0].udelay = client->udelay;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = sizeof(u16);
	msgs[1].buf = (char *)buf;
	msgs[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[1].udelay = client->udelay;

	err = i2c_transfer(client->adapter, msgs, 2);
	if(err >=0){
		*buf = SWAP16(*buf);
	}
	return err;
};

static int sensor_read32(struct i2c_client *client, const u16 regs, u32 *buf)
{
	struct i2c_msg msgs[2];
	u16 r = SWAP16(regs);
	int err = -EAGAIN;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = sizeof(u16);
	msgs[0].buf = (char*)&r;
	msgs[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[0].udelay = client->udelay;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = sizeof(u32);
	msgs[1].buf = (char *)buf;
	msgs[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;
	msgs[1].udelay = client->udelay;

	err = i2c_transfer(client->adapter, msgs, 2);
	if(err >=0){
		*buf = SWAP32(*buf);
	}
	return err;
};

static int sensor_write8(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[sizeof(u16) + sizeof(u8)];
	char *tx_buf = buf;
	*((short*)tx_buf) = SWAP16(reg);
	memcpy(tx_buf + sizeof(u16), &val, sizeof(u8));
	return i2c_send_trans(client, tx_buf, sizeof(u16) + sizeof(u8));
}

static int sensor_write16(struct i2c_client *client, u16 reg, u16 val)
{
	u16 v = SWAP16(val);
	u8 buf[sizeof(u16) + sizeof(u16)];
	char *tx_buf = buf;
	*((short*)tx_buf) = SWAP16(reg);
	memcpy(tx_buf + sizeof(u16), &v, sizeof(u16));

	return i2c_send_trans(client, tx_buf, sizeof(u16) + sizeof(u16));
}

static int sensor_write32(struct i2c_client *client, u16 reg, u32 val)
{
	u32 v = SWAP32(val);
	u8 buf[sizeof(u16) + sizeof(u32)];
	char *tx_buf = buf;
	*((short*)tx_buf) = SWAP16(reg);
	memcpy(tx_buf + sizeof(u16), &v, sizeof(u32));

	return i2c_send_trans(client, tx_buf, sizeof(u16) + sizeof(u32));
}

static int sensor_burst_write(struct i2c_client *client, struct seq_info *si)
{
	struct seq_info* pSeqInf = si;
	u16 v = 0;
	int count = 0, i, ret = 0;
	char *tx_buf, *buf;
	if(pSeqInf->ops == SEQ_BURST){
		buf = kmalloc(sizeof(u16) * pSeqInf->reg, GFP_KERNEL);
		if(!buf) return -ENOMEM;
		tx_buf = buf;
		count = pSeqInf->reg;
		for(i = 0; i < count; i++, pSeqInf++){
			v = pSeqInf->val;
			v = SWAP16(v);
			memcpy(tx_buf + (i * sizeof(u16)), &v, sizeof(u16));
		}
		ret = i2c_send_trans(client, tx_buf, sizeof(u16) * count);
		kfree(buf);
	}
	return ret >= 0 ? count: ret;
}

static int sensor_seq_setbit(struct i2c_client *client, struct seq_info *si, u8 set)
{
	int ret = 0;
	u8 rv8 = 0, m8 = 0;
	u16 rv16 = 0, m16 = 0;
	u32 rv32 = 0, m32 = 0;
	// get original value
	switch(si->len){
		case SEQ_BYTE:
		m8 = (u8)(si->val & 0xFF);
		ret = sensor_read8(client, si->reg, &rv8);
		break;
	case SEQ_WORD:
		m16 = (u16)(si->val & 0xFFFF);
		ret = sensor_read16(client, si->reg, &rv16);
		break;
	case SEQ_DWORD:
		m32 = si->val;
		ret = sensor_read32(client, si->reg, &rv32);
		break;
	default:
		break;
	}
	if(ret < 0) return ret;

	// set new value with bits set or cleared
	switch(si->len){
	case SEQ_BYTE:
		if(!set) rv8 &= ~m8;		// means clear bits
		else rv8 |= m8;			// means set bits
		ret = sensor_write8(client, si->reg, rv8);
		break;
	case SEQ_WORD:
		if(!set) rv16 &= ~m16;	// means clear bits
		else rv16 |= m16;		// means set bits
		ret = sensor_write16(client, si->reg, rv16);
		break;
	case SEQ_DWORD:
		if(!set) rv32 &= ~m32;	// means clear bits
		else rv32 |= m32;		// means set bits
		ret = sensor_write32(client, si->reg, rv32);
		break;
	default:
		break;
	}

	if(ret >=0 && si->tmo){
		mdelay(si->tmo);
	}
	return ret;
};

static int sensor_seq_pollbit(struct i2c_client *client, struct seq_info *si, u8 set)
{
	int ret = 0;
	u8 rv8 = 0, m8 = 0, polling = 0, pollingtimeout = 0;
	u16 rv16 = 0, m16 = 0;
	u32 rv32 = 0, m32 = 0;
	pollingtimeout = si->tmo;

	switch(si->len){
		case SEQ_BYTE:
		m8 = (u8)(si->val & 0xFF);
		break;
	case SEQ_WORD:
		m16 = (u16)(si->val & 0xFFFF);
		break;
	case SEQ_DWORD:
		m32 = si->val;
		break;
	default:
		break;
	}

	polling = 1;
	while(polling){
		// polling get value
		switch(si->len){
		case SEQ_BYTE:
			ret = sensor_read8(client, si->reg, &rv8);
			break;
		case SEQ_WORD:
			ret = sensor_read16(client, si->reg, &rv16);
			break;
		case SEQ_DWORD:
			ret = sensor_read32(client, si->reg, &rv32);
			break;
		default:
			break;
		}
		if(ret < 0) return ret;

		// check if bits set or clr
		switch(si->len){
		case SEQ_BYTE:
			if(!set){
				if(!(rv8 & m8)) polling = 0; // bits cleared, done
			}
			else{
				if(rv8 & m8) polling = 0;	// bits set, done
			}
			break;
		case SEQ_WORD:
			if(!set){
				if(!(rv16 & m16)) polling = 0; // bits cleared, done
			}
			else{
				if(rv16 & m16) polling = 0;	// bits set, done
			}
			break;
		case SEQ_DWORD:
			if(!set){
				if(!(rv32 & m32)) polling = 0; // bits cleared, done
			}
			else{
				if(rv32 & m32) polling = 0;	// bits set, done
			}
			break;
		default:
			break;
		}
		if(!pollingtimeout) polling = 0;
		if(polling){
			if(in_atomic())
				mdelay(10);
			else
				msleep(10);
			pollingtimeout--;
		}
	}
	DEBUG_TRACE("%s leave\n",__FUNCTION__);
	return ret;
};

static int exec_sensor_seq(struct i2c_client *client, struct seq_info *pseq_info)
{
	int ret = 0, count = 0;
	struct seq_info *pseq = pseq_info;
	while(pseq->ops != SEQ_END){
		switch(pseq->ops) {
		case SEQ_REG:
			{
				switch(pseq->len){
				case SEQ_BYTE:
					ret = sensor_write8(client, pseq->reg, (u8)(pseq->val & 0xFF));
					break;
				case SEQ_WORD:
					ret = sensor_write16(client, pseq->reg, (u16)(pseq->val & 0xFFFF));
					break;
				case SEQ_DWORD:
					ret = sensor_write32(client, pseq->reg, pseq->val);
					break;
				default:
					break;
				}
				if(ret < 0) return ret;

				if(pseq->tmo){
					mdelay(pseq->tmo);
				}
			}
			break;
		case SEQ_BIT_SET:
			ret = sensor_seq_setbit(client, pseq, 1);
			break;
		case SEQ_BIT_CLR:
			ret = sensor_seq_setbit(client, pseq, 0);
			break;
		case SEQ_POLL_SET:
			ret = sensor_seq_pollbit(client, pseq, 1);
			break;
		case SEQ_POLL_CLR:
			ret = sensor_seq_pollbit(client, pseq, 0);
			break;
		case SEQ_BURST:
			ret = sensor_burst_write(client, pseq);
			if(ret < 0) return ret;
			break;
		case SEQ_MS:
			mdelay(pseq->tmo);
			break;
		default:
			break;
		}
		if(pseq->ops != SEQ_BURST){
			count++;
			pseq++;
		}
		else{
			count += ret;
			pseq += ret;
		}
	}
	return ret;
};

static int sensor_ioctrl(struct soc_camera_device *icd,enum rk29sensor_power_cmd cmd, int on)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct rk29camera_platform_data* pdata = (struct rk29camera_platform_data*)(to_soc_camera_host(icd->dev.parent)->v4l2_dev.dev->platform_data);
	int ret = 0;

	DEBUG_TRACE("%s %s  cmd(%d) on(%d)\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd,on);
	switch (cmd)
	{
		case Sensor_PowerDown:
		{
			if (icl->powerdown) {
				ret = icl->powerdown(icd->pdev, on);
				if (ret == RK29_CAM_IO_SUCCESS) {
					if (on == 0) {
						mdelay(2);
						if (icl->reset)
							icl->reset(icd->pdev);
					}
				} else if (ret == RK29_CAM_EIO_REQUESTFAIL) {
					ret = -ENODEV;
					goto sensor_power_end;
				}
			}
			break;
			/*nelson_yang@asus: remove
			if(on == 1){
				// power down

				if(pdata && pdata->rk_host_mclk_ctrl)
					pdata->rk_host_mclk_ctrl(icd,0);
				iomux_set(GPIO3_B3);
				gpio_set_value(RK30_PIN3_PB3,0);
				
				if(icl->power){
					icl->power(icd->pdev,0);
				}
				else if(icl->powerdown){
					icl->powerdown(icd->pdev,1);
				}
			}
			else{
				// power on
				if(pdata && pdata->rk_host_mclk_ctrl)
					pdata->rk_host_mclk_ctrl(icd,1);

				iomux_set(CIF0_CLKOUT);
				if(icl->power){
					icl->power(icd->pdev,1);
				}
				else if(icl->powerdown){
					icl->powerdown(icd->pdev,0);
				}
				
			}
			break;
			*/
			
		}
		default:
		{
			DEBUG_TRACE("%s %s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_power_end:

	return ret;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct sensor *sensor = to_sensor(client);
#if (ADJUST_OPTIMIZE_TIME_FALG == 0)
	const struct v4l2_queryctrl *qctrl;
#endif
	const struct sensor_datafmt *fmt;
	int ret;
	u16 pid = 0;

	DEBUG_TRACE("\n%s..%s.. \n",SENSOR_NAME_STRING(),__FUNCTION__);

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}
    
	DEBUG_TRACE("\n soft reset..%s.\n",SENSOR_NAME_STRING());

	sensor->info_priv.outputSize =OUTPUT_QUADVGA;
	sensor->info_priv.supportedSizeNum = 1;
	sensor->info_priv.supportedSize[0] = OUTPUT_QUADVGA;
	
	/* soft reset */

#if SENSOR_NEED_SOFTRESET
	ret = exec_sensor_seq(client, soft_reset_seq_ops);
	if (ret < 0) {
		DEBUG_TRACE("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}
#endif

	DEBUG_TRACE("\n init_seq_ops..%s.\n",SENSOR_NAME_STRING());

	//ret =sensor_write_init_data(client, init_seq_ops);
	ret = exec_sensor_seq(client, init_seq_ops);
	if (ret < 0) {
		DEBUG_TRACE("error: %s initial failed\n",SENSOR_NAME_STRING());
		goto sensor_INIT_ERR;
	}

	/* check if it is an sensor sensor */
#if SENSOR_ID_NEED_PROBE
	ret = sensor_read16(client, 0xC814, &pid);
	if (ret >= 0) {
		DEBUG_TRACE("\n %s  cfg y addr end = 0x%x \n", SENSOR_NAME_STRING(), pid);
	}

	ret = sensor_read16(client, SENSOR_ID_REG, &pid);
	if (ret < 0) {
		DEBUG_TRACE("read chip id failed\n");
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}
	DEBUG_TRACE("\n %s  pid = 0x%x \n", SENSOR_NAME_STRING(), pid);
#else
	pid = SENSOR_ID;
#endif

	if (pid == SENSOR_ID) {
		sensor->model = SENSOR_V4L2_IDENT;
	} else {
		DEBUG_TRACE("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}

	sensor->info_priv.preview_w = SENSOR_INIT_WIDTH;
	sensor->info_priv.preview_h = SENSOR_INIT_HEIGHT;
	sensor->info_priv.capture_w = SENSOR_MAX_WIDTH;
	sensor->info_priv.capture_h = SENSOR_MAX_HEIGHT;
	fmt = sensor_find_datafmt(SENSOR_INIT_PIXFMT,sensor_colour_fmts, ARRAY_SIZE(sensor_colour_fmts));
	if (!fmt) {
		DEBUG_TRACE("error: %s initial array colour fmts is not support!!",SENSOR_NAME_STRING());
		ret = -EINVAL;
		goto sensor_INIT_ERR;
	}
	sensor->info_priv.fmt = *fmt;

	/* sensor sensor information for initialization  */
#if ADJUST_OPTIMIZE_TIME_FALG
	DEBUG_TRACE("\n optimize code..%s.\n",SENSOR_NAME_STRING());
	#if CONFIG_SENSOR_WhiteBalance
	sensor->info_priv.whiteBalance = 0;
	#endif
	#if CONFIG_SENSOR_Brightness
	sensor->info_priv.brightness = 0;
	#endif
	#if CONFIG_SENSOR_Effect
	sensor->info_priv.effect = 0;
	#endif
	#if CONFIG_SENSOR_Exposure
	sensor->info_priv.exposure = 0;
	#endif
	#if CONFIG_SENSOR_Saturation
	sensor->info_priv.saturation = 0;
	#endif
	#if CONFIG_SENSOR_Contrast
	sensor->info_priv.contrast = 0;
	#endif
	#if CONFIG_SENSOR_Sharpness
		sensor->info_priv.sharpness = 0;
	#endif

	#if CONFIG_SENSOR_Mirror
		sensor->info_priv.mirror = 1;
	#endif
	#if CONFIG_SENSOR_Flip
		sensor->info_priv.flip = 1;
		index++;
	#endif
	#if CONFIG_SENSOR_Scene
		sensor->info_priv.scene = 0;
		index++;
	#endif
	#if CONFIG_SENSOR_DigitalZoom
		sensor->info_priv.digitalzoom = 0;
	#endif
	#if CONFIG_SENSOR_Focus
	sensor->info_priv.focus = 125  ;
	if (sensor_af_init(client) < 0) {
		sensor->info_priv.funmodule_state &= ~SENSOR_AF_IS_OK;
		DEBUG_TRACE("%s auto focus module init is fail!\n",SENSOR_NAME_STRING());
	} else {
		sensor->info_priv.funmodule_state |= SENSOR_AF_IS_OK;
		DEBUG_TRACE("%s auto focus module init is success!\n",SENSOR_NAME_STRING());
	}
	#endif
	#if CONFIG_SENSOR_Flash
		sensor->info_priv.flash = 0 ;
	#endif
    
#else //ADJUST_OPTIMIZE_TIME_FALG
    DEBUG_TRACE("\n origin code..%s.\n",SENSOR_NAME_STRING());
#if CONFIG_SENSOR_WhiteBalance
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
	if (qctrl)
    	sensor->info_priv.whiteBalance = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Brightness
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_BRIGHTNESS);
	if (qctrl)
    	sensor->info_priv.brightness = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Effect
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
	if (qctrl)
    	sensor->info_priv.effect = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Exposure
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE);
	if (qctrl)
        sensor->info_priv.exposure = qctrl->default_value;
#endif
#if CONFIG_SENSOR_AntiBanding
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ANTIBANDING);
	if (qctrl){
		sensor->info_priv.antibanding = qctrl->default_value;
		sensor_set_antibanding(icd, qctrl,sensor->info_priv.antibanding);
	}
#endif
#if	CONFIG_SENSOR_WhiteBalanceLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_WHITEBALANCE_LOCK);
	if (qctrl){
		sensor->info_priv.WhiteBalanceLock = qctrl->default_value;
		sensor_whitebalance_lock(icd, qctrl,sensor->info_priv.WhiteBalanceLock);
	}
#endif
#if CONFIG_SENSOR_ExposureLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE_LOCK);
	if (qctrl){
		sensor->info_priv.ExposureLock = qctrl->default_value;
		sensor_set_exposure_lock(icd, qctrl,sensor->info_priv.ExposureLock);
	}
#endif
#if CONFIG_SENSOR_Saturation
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SATURATION);
	if (qctrl)
        sensor->info_priv.saturation = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Contrast
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_CONTRAST);
	if (qctrl)
        sensor->info_priv.contrast = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Mirror
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_HFLIP);
	if (qctrl)
        sensor->info_priv.mirror = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Flip
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_VFLIP);
	if (qctrl)
        sensor->info_priv.flip = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Scene
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);
	if (qctrl)
        sensor->info_priv.scene = qctrl->default_value;
#endif
#if CONFIG_SENSOR_DigitalZoom
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.digitalzoom = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Focus
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.focus = qctrl->default_value;
#endif //CONFIG_SENSOR_Focus
#if CONFIG_SENSOR_Flash
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FLASH);
	if (qctrl)
		sensor->info_priv.flash = qctrl->default_value;
#endif
#endif //ADJUST_OPTIMIZE_TIME_FALG
	DEBUG_TRACE("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),((val == 0)?__FUNCTION__:"sensor_reinit"),icd->user_width,icd->user_height);
	sensor->info_priv.funmodule_state |= SENSOR_INIT_IS_OK;
	return 0;

sensor_INIT_ERR:
	sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	sensor_deactivate(client);
	return ret;
}
static int sensor_deactivate(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;
	struct sensor *sensor = to_sensor(client);

	DEBUG_TRACE("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

	/* ddl@rock-chips.com : all sensor output pin must change to input for other sensor */

	sensor_ioctrl(icd, Sensor_PowerDown, 1);
	msleep(100);

	/* ddl@rock-chips.com : sensor config init width , because next open sensor quickly(soc_camera_open -> Try to configure with default parameters) */
	icd->user_width = SENSOR_INIT_WIDTH;
	icd->user_height = SENSOR_INIT_HEIGHT;
	sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;

	return 0;
}
static struct seq_info sensor_power_down_sequence[]=
{
		{SEQ_END,           0,         0,      0, 0}
};

static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
    int ret;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if (pm_msg.event == PM_EVENT_SUSPEND) {
        DEBUG_TRACE("\n %s Enter Suspend..pm_msg.event=%d \n", SENSOR_NAME_STRING(),pm_msg.event);
        ret = exec_sensor_seq(client, sensor_power_down_sequence) ;
        if (ret < 0) {
            DEBUG_TRACE("\n %s..%s WriteReg Fail.. \n", SENSOR_NAME_STRING(),__FUNCTION__);
            return ret;
        } else {
            ret = sensor_ioctrl(icd, Sensor_PowerDown, 1);
            if (ret < 0) {
			    DEBUG_TRACE("\n %s suspend fail for turn on power!\n", SENSOR_NAME_STRING());
                return -EINVAL;
            }
        }
    } else {
        DEBUG_TRACE("\n %s cann't suppout Suspend..\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }

    return 0;
}

static int sensor_resume(struct soc_camera_device *icd)
{
	int ret;

    ret = sensor_ioctrl(icd, Sensor_PowerDown, 0);
    if (ret < 0) {
		DEBUG_TRACE("\n %s resume fail for turn on power!\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

	DEBUG_TRACE("\n %s Enter Resume.. \n", SENSOR_NAME_STRING());
	return 0;
}

static int sensor_set_bus_param(struct soc_camera_device *icd,
                                unsigned long flags)
{

    return 0;
}

static unsigned long sensor_query_bus_param(struct soc_camera_device *icd)
{
    struct soc_camera_link *icl = to_soc_camera_link(icd);
    unsigned long flags = SENSOR_BUS_PARAM;

    return soc_camera_apply_sensor_flags(icl, flags);
}

static int sensor_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct sensor *sensor = to_sensor(client);

	mf->width	= icd->user_width;
	mf->height	= icd->user_height;
	mf->code	= sensor->info_priv.fmt.code;
	mf->colorspace	= sensor->info_priv.fmt.colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

/*modify image with resolution 2592*1944;solve bug that the first 32 pixel data*/
/*in the first line have misplace with the last 32 pixel data in the last line*/
static int sensor_cb(void *arg)
{
   void __iomem *vbpmem;
   struct videobuf_buffer *buffer;
   char *imagey_addr =NULL;
   char *imageuv_addr = NULL;
   char *tempaddr = NULL;
   int  tempsize = 0;
   
   buffer = (struct videobuf_buffer*)arg; 
   if(buffer->width!=SENSOR_MAX_WIDTH||buffer->height!=SENSOR_MAX_HEIGHT||buffer==NULL)
    return -EINVAL;
 
   if (buffer->bsize< YUV420_BUFFER_MAX_SIZE)        //yuv420 format size
    return -EINVAL;

   
   vbpmem = ioremap(buffer->boff,buffer->bsize);
   if(vbpmem == NULL) {
      DEBUG_TRACE("\n%s..%s..ioremap fail\n",__FUNCTION__,SENSOR_NAME_STRING());
      return -ENXIO;
   }
     
   imagey_addr = (char*)vbpmem;         // y data  to be dealed with
   imageuv_addr = imagey_addr+buffer->width*buffer->height;
   
   tempaddr =  imageuv_addr - 32;  
   memcpy(tempaddr,imagey_addr,32);

   tempaddr = imagey_addr+32;
   memcpy(imagey_addr,tempaddr,32);

                                      //uv data to be dealed with
   tempsize  = YUV420_BUFFER_MAX_SIZE-32;                              
   tempaddr = imagey_addr+tempsize;
   memcpy(tempaddr,imageuv_addr,32);

   tempaddr = imageuv_addr+32;
   memcpy(imageuv_addr,tempaddr,32);
   return 0;
}


static int sensor_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	struct soc_camera_device *icd = client->dev.platform_data;
//	struct sensor *sensor = to_sensor(client);
	const struct sensor_datafmt *fmt;
//	struct seq_op const *winseqe_set_addr=NULL;
	int ret = 0, set_w,set_h;//,cnt;
//	u16 seq_state=0;
//	int time = 0;
//	u16 targetbrightness,realbrightness;

	DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());

	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (!fmt) {
		ret = -EINVAL;
		goto sensor_s_fmt_end;
	}

	set_w = mf->width;
	set_h = mf->height;

	//sensor_fmt_catch(set_w, set_h, &set_w, &set_h);
	if ((set_w <= 1280) && (set_h <= 960)){
		mf->width = 1280;
		mf->height = 960;
	}
	else{
		mf->width = SENSOR_INIT_WIDTH;
		mf->height = SENSOR_INIT_HEIGHT;
	}
sensor_s_fmt_end:
    return ret;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor *sensor = to_sensor(client);
	const struct sensor_datafmt *fmt;
	int ret = 0;

	DEBUG_TRACE("%s..%s w%dh%d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (fmt == NULL) {
		fmt = &sensor->info_priv.fmt;
		mf->code = fmt->code;
	} 

	if (mf->height > SENSOR_MAX_HEIGHT)
		mf->height = SENSOR_MAX_HEIGHT;
	else if (mf->height < SENSOR_MIN_HEIGHT)
		mf->height = SENSOR_MIN_HEIGHT;

	if (mf->width > SENSOR_MAX_WIDTH)
		mf->width = SENSOR_MAX_WIDTH;
	else if (mf->width < SENSOR_MIN_WIDTH)
		mf->width = SENSOR_MIN_WIDTH;

	//sensor_fmt_catch(mf->width, mf->height, &mf->width, &mf->height);
	if ((mf->width <= 1280) && (mf->height <= 960)){
			mf->width = 1280;
			mf->height = 960;
	}
	else{
			mf->width = SENSOR_INIT_WIDTH;
			mf->height = SENSOR_INIT_HEIGHT;
	}
	mf->colorspace = fmt->colorspace;

	return ret;
}
 static int sensor_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *id)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
        return -EINVAL;

    if (id->match.addr != client->addr)
        return -ENODEV;

    id->ident = SENSOR_V4L2_IDENT;      /* ddl@rock-chips.com :  Return OV2655  identifier */
    id->revision = 0;

    return 0;
}

#if CONFIG_SENSOR_Brightness
static int sensor_set_brightness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_BrightnessSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_BrightnessSeqe[value - qctrl->minimum]) != 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	int time =5;
	int ret =0 ;
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

	if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
		if (sensor_effects_seqs[value - qctrl->minimum] != NULL) {
			/*set five times to make sure the set go into effect*/
			/*solve bug for setting invalidate during changing from preview to video*/
			while(time >0) {
				time--;
				ret |= exec_sensor_seq(client, sensor_effects_seqs[value - qctrl->minimum]);
				if(ret < 0) {
					DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
					return -EINVAL;
				}
				msleep(50);
			}
			DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
			return 0;
		}
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_exposure_seqs[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_exposure_seqs[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Saturation
static int sensor_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_saturation_seqs[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_saturation_seqs[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Contrast
static int sensor_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_contrast_seqs[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_contrast_seqs[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Mirror
static int sensor_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_MirrorSeqe[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_MirrorSeqe[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Flip
static int sensor_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_FlipSeqe[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_FlipSeqe[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Scene
static int sensor_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_AntiBanding
static int sensor_set_antibanding(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_antibanding[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_antibanding[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;

}
#endif
#if	CONFIG_SENSOR_WhiteBalanceLock
static int sensor_whitebalance_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_WhiteBalanceLk[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_WhiteBalanceLk[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if	CONFIG_SENSOR_ExposureLock
static int sensor_set_exposure_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureLk[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_ExposureLk[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_JPEG_EXIF
static int sensor_get_iso(struct i2c_client *client)
{
	int ret = 0;
	u32 gain = 0;
	sensor_write16(client, 0x098E, 0x2838);
	sensor_read32(client, 0xA838, &gain);
	if( (gain >= 102) && (gain < 204 )){			//0.8 ~ 1.6 * 128
		ret = 100;
	}else if( (gain >= 204) && (gain < 409 )){	//1.6~3.2 * 128
		ret = 200;
	}else if( (gain >= 409) && (gain < 819 )){	//3.2~6.4 * 128
		ret = 400;
	}else if( (gain >= 819) && (gain < 1638 )){	//6.4~12.8 * 128
		ret = 800;
	}else if( (gain >= 1638) && (gain < 3276 )){	//12.8~25.6 * 128
		ret = 1600;
	}else if( (gain >= 3276) && (gain <= 6553 )){	//25.6~51.2 * 128
		ret = 3200;
	}else{
		ret = 0;
	}			
	return ret;
}
static int sensor_get_exposure_time(struct i2c_client *client)
{
	int ret = 0;
	u16 Line_length_pck,fine_integration_time,coarse_integration_time;
	u32 vt_pix_clk_freq_mhz;
	
	sensor_read16(client, 0x300C, &Line_length_pck);
	sensor_read16(client, 0x3014, &fine_integration_time);
	sensor_read16(client, 0x3012, &coarse_integration_time);
	sensor_read32(client, 0xC808, &vt_pix_clk_freq_mhz);
	
	printk("V4L2_CID_JPEG_EXIF Line_length_pck(%d) fine_integration_time(%d) coarse_integration_time(%d) vt_pix_clk_freq_mhz(%d)\n", 
		Line_length_pck,fine_integration_time,coarse_integration_time,vt_pix_clk_freq_mhz);
	ret = coarse_integration_time;
	ret *= Line_length_pck;
	ret += fine_integration_time;
	
	ret = vt_pix_clk_freq_mhz/ret;
	printk("V4L2_CID_JPEG_EXIF ExposureTime(%d) \n", ret);

	return ret;
}

static int sensor_set_exposure_time(struct i2c_client *client , int EvDenom)
{
	u16 Line_length_pck,fine_integration_time,coarse_integration_time;
	u32 vt_pix_clk_freq_mhz, tmp;
	
	sensor_read16(client, 0x300C, &Line_length_pck);
	sensor_read16(client, 0x3014, &fine_integration_time);
	sensor_read16(client, 0x3012, &coarse_integration_time);
	sensor_read32(client, 0xC808, &vt_pix_clk_freq_mhz);
	
	coarse_integration_time = Line_length_pck*(vt_pix_clk_freq_mhz - EvDenom*fine_integration_time)/EvDenom;

	if(coarse_integration_time < 0x03EC){
		sensor_write16(client, 0x300A, 0x03EE);
	}
	else{
		sensor_write16(client, 0x300A, coarse_integration_time + 0x02);
	}
	
	sensor_write16(client, 0x3012, coarse_integration_time);
	
	return 0;
}
#endif

#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_wb_seqs[value - qctrl->minimum] != NULL)
        {
            if (exec_sensor_seq(client, sensor_wb_seqs[value - qctrl->minimum]) < 0)
            {
                DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Sharpness
static int sensor_set_sharpness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

	if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
		if (sensor_sharpness_seqs[value - qctrl->minimum] != NULL) {
			if (exec_sensor_seq(client, sensor_sharpness_seqs[value - qctrl->minimum]) < 0) {
				DEBUG_TRACE("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
				return -EINVAL;
			}
			DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
			return 0;
		}
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -EINVAL;
}
#endif

#if CONFIG_SENSOR_DigitalZoom

#endif

#if CONFIG_SENSOR_Flash
static int sensor_set_flash(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{    
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
        if (value == 3) {       /* ddl@rock-chips.com: torch */
            sensor_ioctrl(icd, Sensor_Flash, Flash_Torch);   /* Flash On */
        } else {
            sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
        }
        DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }
    
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Focus
static int sensor_set_focus_absolute(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
	int ret = 0;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (!qctrl_info)
		return -EINVAL;

	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK) && (sensor->info_priv.affm_reinit == 0)) {
		if ((value >= qctrl_info->minimum) && (value <= qctrl_info->maximum)) {

			DEBUG_TRACE("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
		} else {
			ret = -EINVAL;
			DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
		}
	} else {
		ret = -EACCES;
		DEBUG_TRACE("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}

sensor_set_focus_absolute_end:
	return ret;
}

static int sensor_set_focus_relative(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
	int ret = 0;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_RELATIVE);
	if (!qctrl_info)
		return -EINVAL;

	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK) && (sensor->info_priv.affm_reinit == 0)) {
		if ((value >= qctrl_info->minimum) && (value <= qctrl_info->maximum)) {

			DEBUG_TRACE("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
		} else {
			ret = -EINVAL;
			DEBUG_TRACE("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
		}
	} else {
		ret = -EACCES;
		DEBUG_TRACE("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}
sensor_set_focus_relative_end:
	return ret;
}

static int sensor_set_focus_mode(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client);
	int ret = 0;

	if ((sensor->info_priv.funmodule_state & SENSOR_AF_IS_OK)  && (sensor->info_priv.affm_reinit == 0)) {
		switch (value)
		{
			case SENSOR_AF_MODE_AUTO:
			{
				ret = sensor_af_single(client);
				break;
			}

			case SENSOR_AF_MODE_MACRO:
			{
				ret = sensor_set_focus_absolute(icd, qctrl, 0xff);
				break;
			}

			case SENSOR_AF_MODE_INFINITY:
			{
				ret = sensor_set_focus_absolute(icd, qctrl, 0x00);
				break;
			}

			case SENSOR_AF_MODE_CONTINUOUS:
			{
				ret = sensor_af_const(client);
				break;
			}
			default:
				DEBUG_TRACE("\n %s..%s AF value(0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
				break;

		}

		DEBUG_TRACE("%s..%s : %d  ret:0x%x\n",SENSOR_NAME_STRING(),__FUNCTION__, value,ret);
	} else {
		ret = -EACCES;
		DEBUG_TRACE("\n %s..%s AF module state(0x%x, 0x%x) is error!\n",SENSOR_NAME_STRING(),__FUNCTION__,
			sensor->info_priv.funmodule_state,sensor->info_priv.affm_reinit);
	}

	return ret;
}
#endif

static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct v4l2_queryctrl *qctrl;
    DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());

    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

	switch (ctrl->id) {
#if CONFIG_SENSOR_Brightness
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = sensor->info_priv.brightness;
		break;
#endif
#if CONFIG_SENSOR_Saturation
	case V4L2_CID_SATURATION:
		ctrl->value = sensor->info_priv.saturation;
		break;
#endif
#if CONFIG_SENSOR_Contrast
	case V4L2_CID_CONTRAST:
		ctrl->value = sensor->info_priv.contrast;
		break;
#endif
#if CONFIG_SENSOR_WhiteBalance
	case V4L2_CID_DO_WHITE_BALANCE:
		ctrl->value = sensor->info_priv.whiteBalance;
		break;
#endif
#if CONFIG_SENSOR_Effect
	case V4L2_CID_EXPOSURE:
		ctrl->value = sensor->info_priv.exposure;
		break;
#endif
#if CONFIG_SENSOR_Scene
	case V4L2_CID_SCENE:
		ctrl->value = sensor->info_priv.scene;
		break;
#endif
#if CONFIG_SENSOR_Mirror
	case V4L2_CID_HFLIP:
		ctrl->value = sensor->info_priv.mirror;
		break;
#endif
#if CONFIG_SENSOR_Flip
	case V4L2_CID_VFLIP:
		ctrl->value = sensor->info_priv.flip;
		break;
#endif
#if CONFIG_SENSOR_Flash
	case V4L2_CID_FLASH:
		ctrl->value = sensor->info_priv.flash;
		break;
#endif
#if CONFIG_SENSOR_Sharpness
	case V4L2_CID_SHARPNESS:
		ctrl->value = sensor->info_priv.sharpness;
		break;
#endif
	default :
		break;
    }
    return 0;
}

#define offset_of(type, member)	((i32)(&((type*)0)->member))
#define offset2member(p, offset, membertype)	((membertype*)((char*)p) + offset)

static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor *sensor = to_sensor(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	const struct v4l2_queryctrl *qctrl;

	DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());


	qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

	if (!qctrl) {
		DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
		return -EINVAL;
	}

    switch (ctrl->id) {
#if CONFIG_SENSOR_Brightness
	case V4L2_CID_BRIGHTNESS:
       	if (ctrl->value != sensor->info_priv.brightness) {
       		if (sensor_set_brightness(icd, qctrl,ctrl->value) != 0) {
       			return -EINVAL;
       		}
       		sensor->info_priv.brightness = ctrl->value;
       	}
       	break;
#endif
#if CONFIG_SENSOR_Exposure
	case V4L2_CID_EXPOSURE:
		if (ctrl->value != sensor->info_priv.exposure) {
			if (sensor_set_exposure(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.exposure = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_Saturation
	case V4L2_CID_SATURATION:
		if (ctrl->value != sensor->info_priv.saturation) {
			if (sensor_set_saturation(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.saturation = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_Contrast
	case V4L2_CID_CONTRAST:
		if (ctrl->value != sensor->info_priv.contrast) {
			if (sensor_set_contrast(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.contrast = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_WhiteBalance
	case V4L2_CID_DO_WHITE_BALANCE:
		if (ctrl->value != sensor->info_priv.whiteBalance) {
			if (sensor_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.whiteBalance = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_Mirror
	case V4L2_CID_HFLIP:
		if (ctrl->value != sensor->info_priv.mirror) {
			if (sensor_set_mirror(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.mirror = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_Flip
	case V4L2_CID_VFLIP:
		if (ctrl->value != sensor->info_priv.flip) {
			if (sensor_set_flip(icd, qctrl,ctrl->value) != 0)
				return -EINVAL;
			sensor->info_priv.flip = ctrl->value;
		}
		break;
#endif
#if CONFIG_SENSOR_Sharpness
	case V4L2_CID_SHARPNESS:
        if (ctrl->value != sensor->info_priv.sharpness) {
        	if (sensor_set_sharpness(icd, qctrl,ctrl->value) != 0)
        		return -EINVAL;
        	sensor->info_priv.sharpness = ctrl->value;
        }
        break;
#endif
	default:
		break;
    }
    return 0;
}

static int sensor_g_ext_control(struct soc_camera_device *icd , struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
    
    DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl) {
        DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

    switch (ext_ctrl->id) {
#if CONFIG_SENSOR_Scene
	case V4L2_CID_SCENE:
		ext_ctrl->value = sensor->info_priv.scene;
		break;
#endif
#if CONFIG_SENSOR_AntiBanding
	case V4L2_CID_ANTIBANDING:
		{
			ext_ctrl->value = sensor->info_priv.antibanding;
			break;
		}
#endif
#if	CONFIG_SENSOR_WhiteBalanceLock
	case V4L2_CID_WHITEBALANCE_LOCK:
		{
			ext_ctrl->value = sensor->info_priv.WhiteBalanceLock;
			break;
		}
#endif
#if CONFIG_SENSOR_ExposureLock
	case V4L2_CID_EXPOSURE_LOCK:
		{
			ext_ctrl->value = sensor->info_priv.ExposureLock;
			break;
		}
#endif
#if CONFIG_SENSOR_Effect
	case V4L2_CID_EFFECT:
		ext_ctrl->value = sensor->info_priv.effect;
		break;
#endif
#if CONFIG_SENSOR_DigitalZoom
	case V4L2_CID_ZOOM_ABSOLUTE:
		ext_ctrl->value = sensor->info_priv.digitalzoom;
		break;
	case V4L2_CID_ZOOM_RELATIVE:
		return -EINVAL;
#endif
#if CONFIG_SENSOR_Focus
	case V4L2_CID_FOCUS_ABSOLUTE:
		return -EINVAL;
	case V4L2_CID_FOCUS_RELATIVE:
		return -EINVAL;
#endif
#if CONFIG_SENSOR_Flash
	case V4L2_CID_FLASH:
		ext_ctrl->value = sensor->info_priv.flash;
		break;
#endif
#if CONFIG_SENSOR_ISO
	case V4L2_CID_ISO:
		ext_ctrl->value = sensor_get_iso(client);
		break;
#endif
#if CONFIG_SENSOR_JPEG_EXIF
	case V4L2_CID_JPEG_EXIF:
		{
			RkExifInfo *pExitInfo = (RkExifInfo *)ext_ctrl->value;
			if(pExitInfo){
				pExitInfo->ExposureTime.num = 1;
				pExitInfo->ExposureTime.denom = sensor_get_exposure_time(client);
				pExitInfo->ISOSpeedRatings = sensor_get_iso(client);
				pExitInfo->ExposureBiasValue.num = sensor->info_priv.exposure;
				pExitInfo->ExposureBiasValue.denom = 3;
			}
			break;
		}
#endif
	default:
		break;
    }
    return 0;
}

static int sensor_s_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
    int val_offset;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

	val_offset = 0;
    switch (ext_ctrl->id)
    {
#if CONFIG_SENSOR_Scene
        case V4L2_CID_SCENE:
            {
                if (ext_ctrl->value != sensor->info_priv.scene)
                {
                    if (sensor_set_scene(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.scene = ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_AntiBanding
		case V4L2_CID_ANTIBANDING:
			{
				if (ext_ctrl->value != sensor->info_priv.antibanding)
				{
					if (sensor_set_antibanding(icd, qctrl,ext_ctrl->value) != 0)
						return -EINVAL;
					sensor->info_priv.antibanding = ext_ctrl->value;
				}
				break;
			}
#endif
#if	CONFIG_SENSOR_WhiteBalanceLock
		case V4L2_CID_WHITEBALANCE_LOCK:
			{
				if (ext_ctrl->value != sensor->info_priv.WhiteBalanceLock)
				{
					if (sensor_whitebalance_lock(icd, qctrl,ext_ctrl->value) != 0)
						return -EINVAL;
					sensor->info_priv.WhiteBalanceLock = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_ExposureLock
		case V4L2_CID_EXPOSURE_LOCK:
			{
				if (ext_ctrl->value != sensor->info_priv.ExposureLock)
				{
					if (sensor_set_exposure_lock(icd, qctrl,ext_ctrl->value) != 0)
						return -EINVAL;
					sensor->info_priv.ExposureLock = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Effect
        case V4L2_CID_EFFECT:
            {
                if (ext_ctrl->value != sensor->info_priv.effect)
                {
                    if (sensor_set_effect(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.effect= ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_DigitalZoom
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.digitalzoom)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.digitalzoom;

                    if (sensor_set_digitalzoom(icd, qctrl,&val_offset) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += val_offset;

                    DEBUG_TRACE("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->info_priv.digitalzoom);
                }

                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    if (sensor_set_digitalzoom(icd, qctrl,&ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += ext_ctrl->value;

                    DEBUG_TRACE("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.digitalzoom);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Focus
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

				if (sensor_set_focus_absolute(icd, qctrl,ext_ctrl->value) == 0) {
					if (ext_ctrl->value == qctrl->minimum) {
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_INFINITY;
					} else if (ext_ctrl->value == qctrl->maximum) {
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_MACRO;
					} else {
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_FIXED;
					}
				}

                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                sensor_set_focus_relative(icd, qctrl,ext_ctrl->value);
                break;
            }
		case V4L2_CID_FOCUS_AUTO:
			{
				if (ext_ctrl->value == 1) {
					if (sensor_set_focus_mode(icd, qctrl,SENSOR_AF_MODE_AUTO) != 0)
						return -EINVAL;
					sensor->info_priv.auto_focus = SENSOR_AF_MODE_AUTO;
				} else if (SENSOR_AF_MODE_AUTO == sensor->info_priv.auto_focus){
					if (ext_ctrl->value == 0)
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CLOSE;
				}
				break;
			}
		case V4L2_CID_FOCUS_CONTINUOUS:
			{
				if (SENSOR_AF_MODE_CONTINUOUS != sensor->info_priv.auto_focus) {
					if (ext_ctrl->value == 1) {
						if (sensor_set_focus_mode(icd, qctrl,SENSOR_AF_MODE_CONTINUOUS) != 0)
							return -EINVAL;
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CONTINUOUS;
					}
				} else {
					if (ext_ctrl->value == 0)
						sensor->info_priv.auto_focus = SENSOR_AF_MODE_CLOSE;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Flash
        case V4L2_CID_FLASH:
            {
                if (sensor_set_flash(icd, qctrl,ext_ctrl->value) != 0)
                    return -EINVAL;
                sensor->info_priv.flash = ext_ctrl->value;

                DEBUG_TRACE("%s flash is %x\n",SENSOR_NAME_STRING(), sensor->info_priv.flash);
                break;
            }
#endif
#if CONFIG_SENSOR_JPEG_EXIF
		case V4L2_CID_JPEG_EXIF:
			{
				RkExifInfo *pExitInfo = (RkExifInfo *)ext_ctrl->value;
				if(pExitInfo){
					if(pExitInfo->ExposureTime.num == 0){
						// Disable AE
						exec_sensor_seq(client, sensor_ExposureLk[1]);
					}
					
					if(pExitInfo->ExposureTime.denom != sensor_get_exposure_time(client)){
						sensor_set_exposure_time(client, pExitInfo->ExposureTime.denom);
					}

					if(pExitInfo->ExposureTime.num == 1){
						// Enable AE
						exec_sensor_seq(client, sensor_ExposureLk[0]);
					}
				}
				break;
			}
#endif

        default:
            break;
    }

    return 0;
}

static int sensor_g_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;
    
    DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());


    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_g_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

static int sensor_s_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;
    
    DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());

    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_s_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

//the return value is not used, Refer: soc_camera.c line 794, 826
static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor *sensor = to_sensor(client);
	int ret = 0;
	DEBUG_TRACE("%s...entry %d\n",__FUNCTION__, enable);
	if (enable == 1) {
		//ret = exec_sensor_seq(client, sensor_exit_standby);
		//if (ret >= 0)
			sensor->info_priv.enable = 1;
	} else if (enable == 0) {
		//ret = exec_sensor_seq(client, sensor_enter_standby);
		//if (ret >= 0)
			sensor->info_priv.enable = 0;
	}

	DEBUG_TRACE("%s...exit %d\n",__FUNCTION__, ret);
	return ret;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
	int ret;
	u16 pid = 0;
	struct sensor *sensor = to_sensor(client);

	/* We must have a parent by now. And it cannot be a wrong one.
	* So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}

	/* check if it is an sensor sensor */
#if SENSOR_ID_NEED_PROBE
	ret = sensor_read16(client, SENSOR_ID_REG, &pid);
	if (ret < 0) {
		DEBUG_TRACE("read chip id failed\n");
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}

	DEBUG_TRACE("\n %s  pid = 0x%x \n", SENSOR_NAME_STRING(), pid);
#else
	pid = SENSOR_ID;
#endif

	if (pid == SENSOR_ID) {
		sensor->model = SENSOR_V4L2_IDENT;
	} else {
		DEBUG_TRACE("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}

	/* soft reset */
#if	SENSOR_NEED_SOFTRESET
	ret = exec_sensor_seq(client, soft_reset_seq_ops);
	if (ret < 0) {
		DEBUG_TRACE("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}
#endif

	return 0;

sensor_video_probe_err:
	 return ret;
}
static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct sensor *sensor = to_sensor(client);
	int ret = 0, i;

	rk29_camera_sensor_cb_s *icd_cb =NULL;
    
	DEBUG_TRACE("\n%s..%s..cmd:%x \n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
	switch (cmd)
	{
		case RK29_CAM_SUBDEV_DEACTIVATE:
		{
			sensor_deactivate(client);
			break;
		}
		case RK29_CAM_SUBDEV_IOREQUEST:
		{
			sensor->sensor_io_request = (struct rk29camera_platform_data*)arg;           
			if (sensor->sensor_io_request != NULL) {
				sensor->sensor_gpio_res = NULL;
				for (i=0; i<RK29_CAM_SUPPORT_NUMS;i++) {
					if (sensor->sensor_io_request->gpio_res[i].dev_name &&
						(strcmp(sensor->sensor_io_request->gpio_res[i].dev_name, dev_name(icd->pdev)) == 0)) {
						sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[i];
					}
				}
				if (sensor->sensor_gpio_res == NULL) {
					DEBUG_TRACE("%s %s obtain gpio resource failed when RK29_CAM_SUBDEV_IOREQUEST \n",SENSOR_NAME_STRING(),__FUNCTION__);
					ret = -EINVAL;
					goto sensor_ioctl_end;
				}
			} else {
				DEBUG_TRACE("%s %s RK29_CAM_SUBDEV_IOREQUEST fail\n",SENSOR_NAME_STRING(),__FUNCTION__);
				ret = -EINVAL;
				goto sensor_ioctl_end;
			}
			/* ddl@rock-chips.com : if gpio_flash havn't been set in board-xxx.c, sensor driver must notify is not support flash control
			for this project */
			break;
		}
		case RK29_CAM_SUBDEV_CB_REGISTER:
		{
			icd_cb = (rk29_camera_sensor_cb_s*)(arg);
			icd_cb->sensor_cb = sensor_cb;
			break;
		}
		default:
		{
			DEBUG_TRACE("%s %s cmd(0x%x) is unknown !\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_ioctl_end:
	return ret;
}

static int sensor_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor *sensor = to_sensor(client);
	

	//get supported framesize num

	
	if (fsize->index >= sensor->info_priv.supportedSizeNum) {
		err = -1;
		goto end;
	}

	if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_QCIF))
	{
		
		fsize->discrete.width = 176;
		fsize->discrete.height = 144;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_QVGA))
	{
		fsize->discrete.width = 320;
		fsize->discrete.height = 240;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_CIF) )
	{
	
		fsize->discrete.width = 352;
		fsize->discrete.height = 288;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_VGA))
	{
	
		fsize->discrete.width = 640;
		fsize->discrete.height = 480;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_SVGA))
	{
	
		fsize->discrete.width = 800;
		fsize->discrete.height = 600;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_XGA))
	{
	
		fsize->discrete.width = 1024;
		fsize->discrete.height = 768;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_720P))
	{
	
		fsize->discrete.width = 1280;
		fsize->discrete.height = 720;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_QUADVGA))
	{
		fsize->discrete.width = 1280;
		fsize->discrete.height = 960;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_XGA))
	{
		fsize->discrete.width = 1280;
		fsize->discrete.height = 1024;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_UXGA) )
	{
		fsize->discrete.width = 1600;
		fsize->discrete.height = 1200;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_1080P))
	{
		fsize->discrete.width = 1920;
		fsize->discrete.height = 1080;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_QXGA))
	{
		fsize->discrete.width = 2048;
		fsize->discrete.height = 1536;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->info_priv.supportedSize[fsize->index] & OUTPUT_QSXGA))
	{
	
		fsize->discrete.width = 2592;
		fsize->discrete.height = 1944;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	} else {
		err = -1;
	}

	
	
end:
	return err;
}

static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(sensor_colour_fmts))
		return -EINVAL;

	*code = sensor_colour_fmts[index].code;
	return 0;
}
static struct v4l2_subdev_core_ops sensor_subdev_core_ops = {
	.init		= sensor_init,
	.g_ctrl		= sensor_g_control,
	.s_ctrl		= sensor_s_control,
	.g_ext_ctrls	= sensor_g_ext_controls,
	.s_ext_ctrls	= sensor_s_ext_controls,
	.g_chip_ident	= sensor_g_chip_ident,
	.ioctl		= sensor_ioctl,
};
static struct v4l2_subdev_video_ops sensor_subdev_video_ops = {
	.s_mbus_fmt	= sensor_s_fmt,
	.g_mbus_fmt	= sensor_g_fmt,
	.try_mbus_fmt	= sensor_try_fmt,
	.enum_mbus_fmt	= sensor_enum_fmt,
	.s_stream	= sensor_s_stream,
	.enum_framesizes = sensor_enum_framesizes,
};
static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
	.video	= &sensor_subdev_video_ops,
};

static int sensor_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct sensor *sensor;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	DEBUG_TRACE("\n%s..%s..%d..\n",__FUNCTION__,__FILE__,__LINE__);
	if (!icd) {
		dev_err(&client->dev, "%s: missing soc-camera data!\n",SENSOR_NAME_STRING());
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "%s driver needs platform data\n", SENSOR_NAME_STRING());
		return -EINVAL;
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			"I2C-Adapter doesn't support I2C_FUNC_I2C\n");
		return -EIO;
	}
	sensor = kzalloc(sizeof(struct sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	v4l2_i2c_subdev_init(&sensor->subdev, client, &sensor_subdev_ops);
	/* Second stage probe - when a capture adapter is there */
	icd->ops		= &sensor_ops;
	sensor->info_priv.fmt = sensor_colour_fmts[0];
	ret = sensor_video_probe(icd, client);
	if (ret < 0) {
		icd->ops = NULL;
		i2c_set_clientdata(client, NULL);
		kfree(sensor);
		sensor = NULL;
	}
	DEBUG_TRACE("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor *sensor = to_sensor(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(sensor);
	sensor = NULL;
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME_STRING(), 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME_STRING(),
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_id,
};

static int __init sensor_mod_init(void)
{
	LOG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());
    return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	LOG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());
	i2c_del_driver(&sensor_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION(SENSOR_NAME_STRING(Camera sensor driver));
MODULE_AUTHOR("ddl <kernel@rock-chips>");
MODULE_LICENSE("GPL");

