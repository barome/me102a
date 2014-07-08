/*
o* Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/hardirq.h>
#include <linux/miscdevice.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <plat/rk_camera.h>

#define ASUS_CAMERA_SUPPORT 1
static int debug;

module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING fmt , ## arg); } while (0)

#define SENSOR_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define SENSOR_DG(format, ...) dprintk(1, format, ## __VA_ARGS__)


#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define MIN(x,y)	((x<y) ? x: y)
#define MAX(x,y)	((x>y) ? x: y)

/* Sensor Driver Configuration */
#define SENSOR_NAME st25a
#define SENSOR_V4L2_IDENT V4L2_IDENT_ST25A
#define SENSOR_ID						0x1A
#define SENSOR_MIN_WIDTH		800	//176
#define SENSOR_MIN_HEIGHT		600	//144
#define SENSOR_MAX_WIDTH		1600
#define SENSOR_MAX_HEIGHT		1200
#define SENSOR_INIT_WIDTH		1600	//640			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT	1200	//480
#define SENSOR_INIT_WINSEQADR sensor_uxga
#define SENSOR_INIT_PIXFMT V4L2_MBUS_FMT_YUYV8_2X8

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness		1
#define CONFIG_SENSOR_Contrast			1
#define CONFIG_SENSOR_Saturation		1
#define CONFIG_SENSOR_Effect				1
#define CONFIG_SENSOR_Scene					1
#define CONFIG_SENSOR_DigitalZoom		0
#define CONFIG_SENSOR_Focus					0
#define CONFIG_SENSOR_Exposure			1
#define CONFIG_SENSOR_Flash					1
#define CONFIG_SENSOR_Mirror				1
#define CONFIG_SENSOR_Flip					1
#define CONFIG_SENSOR_AntiBanding			1
#define CONFIG_SENSOR_WhiteBalanceLock		1
#define CONFIG_SENSOR_ExposureLock			1
#define CONFIG_SENSOR_ISO					1
#define CONFIG_SENSOR_JPEG_EXIF				1


#define CONFIG_SENSOR_I2C_SPEED			80000       /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING |\
                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW|\
                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_24MHZ)

#define COLOR_TEMPERATURE_CLOUDY_DN			6500
#define COLOR_TEMPERATURE_CLOUDY_UP			8000
#define COLOR_TEMPERATURE_CLEARDAY_DN		5000
#define COLOR_TEMPERATURE_CLEARDAY_UP		6500
#define COLOR_TEMPERATURE_OFFICE_DN     3500
#define COLOR_TEMPERATURE_OFFICE_UP     5000
#define COLOR_TEMPERATURE_HOME_DN       2500
#define COLOR_TEMPERATURE_HOME_UP       3500

#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SENSOR_AF_IS_ERR			(0x00<<0)
#define SENSOR_AF_IS_OK				(0x01<<0)
#define SENSOR_INIT_IS_ERR		(0x00<<28)
#define SENSOR_INIT_IS_OK			(0x01<<28)


enum SENSOR_OUTPUT_RES{
	OUTPUT_QCIF 		=0x0001, // 176*144
	OUTPUT_HQVGA		=0x0002,// 240*160
	OUTPUT_QVGA 		=0x0004, // 320*240
	OUTPUT_CIF			=0x0008,  // 352*288
	OUTPUT_VGA			=0x0010,  // 640*480
	OUTPUT_SVGA 		=0x0020, // 800*600
	OUTPUT_720P 		=0x0040, // 1280*720
	OUTPUT_XGA			=0x0080,  // 1024*768
	OUTPUT_QUADVGA		=0x0100,  // 1280*960
	OUTPUT_SXGA 		=0x0200, // 1280*1024
	OUTPUT_UXGA 		=0x0400, // 1600*1200
	OUTPUT_1080P		=0x0800, //1920*1080
	OUTPUT_QXGA 		=0x1000,  // 2048*1536
	OUTPUT_QSXGA		=0x2000, // 2592*1944
};

typedef struct{
	uint32_t num;
	uint32_t denom;
}rat_t;

typedef struct{
	/*IFD0*/
	char *maker;//manufacturer of digicam, just to adjust to make inPhybusAddr to align to 64
	int makerchars;//length of maker, contain the end '\0', so equal strlen(maker)+1
	char *modelstr;//model number of digicam
	int modelchars;//length of modelstr, contain the end '\0'
	int Orientation;//usually 1
	//XResolution, YResolution; if need be not 72, TODO...
	char DateTime[20];//must be 20 chars->  yyyy:MM:dd0x20hh:mm:ss'\0'
	/*Exif SubIFD*/
	rat_t ExposureTime;//such as 1/400=0.0025s
	rat_t ApertureFNumber;//actual f-number
	int ISOSpeedRatings;//CCD sensitivity equivalent to Ag-Hr film speedrate
	rat_t CompressedBitsPerPixel;
	rat_t ShutterSpeedValue;
	rat_t ApertureValue;
	rat_t ExposureBiasValue;
	rat_t MaxApertureValue;
	int MeteringMode;
	int Flash;
	rat_t FocalLength;
	rat_t FocalPlaneXResolution;
	rat_t FocalPlaneYResolution;
	int SensingMethod;//2 means 1 chip color area sensor
	int FileSource;//3 means the image source is digital still camera
	int CustomRendered;//0
	int ExposureMode;//0
	int WhiteBalance;//0
	rat_t DigitalZoomRatio;// inputw/inputw
	//int FocalLengthIn35mmFilm;
	int SceneCaptureType;//0
	
}RkExifInfo;

struct reginfo
{
    u8 reg;
    u8 val;
};

/* CAMERA_REAR_SENSOR_SETTING:r20130830_01 */

//flash off in fixed time to prevent from too hot , zyc
struct  flash_timer{
    struct soc_camera_device *icd;
	struct hrtimer timer;
};
static enum hrtimer_restart flash_off_func(struct hrtimer *timer);

static struct  flash_timer flash_off_timer;
//for user defined if user want to customize the series , zyc
#ifdef CONFIG_ST25A_USER_DEFINED_SERIES
#include "st25a_user_series.c"
#else
/* init 1600X1200 UXGA */
static struct reginfo sensor_init_data[] =
{
	//Sensor Setting
	//PMU Block
	{0x00, 0x00},
	{0x03, 0x04},
	{0x11, 0x32},	//Recommend, PAD_CNTR VDDIO : 1.8V

	//SNR Block 
	{0x00, 0x01},
	{0x03, 0x01},
    	
	{0x04, 0x50},
	{0x08, 0x90},
	{0x0a, 0x20},	//MCLK 24M, PCLK 64M
	{0x10, 0x11},	//Recommend, COMP_BIAS
	{0x11, 0x33},	//Recommend, PXL_BIAS
	{0x12, 0x10},	//Recommend, RAMP_BIAS1
	{0x14, 0x80},	//Recommend, ABS_CNTR1
	{0x15, 0x00},
	{0x16, 0xa9},	//Recommend, PCP_CNTR 64M
	{0x17, 0x10},	//Recommend, CP_CNTR
	{0x18, 0x04},	//Recommend, NCP_CNTR

	{0x30, 0x00},
	{0x39, 0x00},
	{0x3a, 0x26},
	{0x3b, 0x00},
	{0x3c, 0x00},
	{0x3d, 0x00},
	{0x3e, 0x1c},

	{0x3f, 0x07},
	{0x40, 0x03},	//Recommend, BL_CNTR
	{0x41, 0x02},	//Recommend, RAMP_CNTR1
	{0x42, 0x20},	//Recommend, RAMP_CNTR2
	{0x44, 0x07},	//Recommend, RAMP_OFFSET1
	{0x45, 0x07},	//Recommend, RAMP_OFFSET2
	{0x50, 0x00},	//Recommend, TEST_50
	{0x51, 0x10},	//Recommend, TEST_51
	{0x52, 0x00},
	{0x53, 0x00},
	{0x54, 0x25},
	{0x55, 0x25},
	{0x56, 0x10},	//Recommend, TEST_56

	//AE Block
	{0x00, 0x02},
	{0x10, 0x98},	//[7] AE_ON [3] update sequence [2]AE_BLC
	{0x11, 0x10},	//MAX_SHUTSTEP, 7.5fps
	{0x12, 0x36},	//Y_TARGET_N
	{0x13, 0x36},	//Y_TARGET_CWF
	{0x14, 0x36},	//Y_TARGET_A
	{0x15, 0x03},	//LOCK_DIFF

	{0x1a, 0x04},	//AEBLC_MAX

	{0x1c, 0x01}, //INI_SHUTTIME
	{0x1d, 0x00}, //INI_CGAIN	
	{0x1e, 0x14}, //INI_FGAINH
	{0x1f, 0x00}, //INI_FGAINL

	{0x20, 0x30}, //[7]DRK_TEN [4]NOR_ADJ_TEN
	{0x26, 0x38}, //NOR_ADJ_TARGET
	{0x27, 0x01}, //TGT_ADJ_SHUT
	{0x28, 0x00}, //DRK_ADJ_ILLUMI
	{0x29, 0x00}, //DRK_TGT_CNT

	{0x36, 0x9c}, //STST

	{0x40, 0x0a},	//Recommend, GAIN_MIN0 1.25x
	{0x41, 0x0a},	//Recommend, GAIN_MIN1 1.25x
	{0x42, 0x00},	//TGT_SHUT0 700Lux x2.1
	{0x43, 0x00},	//REF_GAIN0
	{0x44, 0x00},	//TGT_SHUT1 200Lux x3.375
	{0x45, 0x00},	//REF_GAIN1
	{0x46, 0x00},	//TGT_SHUT2 100Lux x4.73
	{0x47, 0x00},	//REF_GAIN2

	{0x4e, 0x20},	//Maximum Gain x4

	{0x55, 0x00},	//DARK_CNT

	{0x56, 0x80},	//YWRPCTRL
	{0x57, 0xe1},	//YWRPGAIN
	{0x58, 0x1e},	//YWRPLO_REF
	{0x59, 0xe6},	//YWRPHI_REF
	{0x5a, 0xaa},	//YWRPHI_MAX

	{0x60, 0xff},	//WINA_USE
	{0x61, 0xff},	//WINB_USE
	{0x62, 0xff},	//WINC_USE
	{0x63, 0xff},	//WIND_USE
	{0x64, 0xff},	//WINE_USE
	{0x65, 0xff},	//WINF_USE
	{0x66, 0x00},	//WINA_WEIGHT
	{0x67, 0x50},	//WINB_WEIGHT
	{0x68, 0x50},	//WINC_WEIGHT
	{0x69, 0x50},	//WIND_WEIGHT
	{0x6a, 0x50},	//WINE_WEIGHT
	{0x6b, 0x00},	//WINF_WEIGHT
	{0x6c, 0x06},	//TOTAL_WEIGHT
	{0x6d, 0x80},	//AVG_WEIGHT

	{0x76, 0xc2},	//Dark condition enable
	{0x77, 0xff},	//Yavg
	{0x78, 0x10},	//Shuttet Max
	{0x7b, 0x20},	//Gain Max x4


	//AWB Block
	{0x00, 0x03},
	{0x10, 0xd3},	//AWBCTRL1 # 0xd3 # 0xc8(fixed on) 20130723
	{0x11, 0x10},	//AWBCTRL2 # Default : 89
	{0x12, 0x80},	//Ourdoor fix
	{0x13, 0x80},	//AWB taget Cr
	{0x14, 0x80},	//AWB taget Cb
	{0x15, 0xbc},	//RGNTOP 	# Default : df                            
	{0x16, 0x7e},	//RGNBOT	# 80 # Default : 70 20130723
	{0x17, 0xb0},	//BGNTOP	# 0xea # Default : df
	{0x18, 0x7c},	//BGNBOT	# Default : 78

	{0x19, 0xbc},	//ad RGNTOP@BRT
	{0x1a, 0x7c},	//ad RGNBOT@BRT
	{0x1b, 0xa0},	//88 BGNTOP@BRT
	{0x1c, 0x7c},	//88 BGNBOT@BRT
	{0x1d, 0x94},	//04 BRTSRT	# Default : 04
	{0x1e, 0x2a},	//2a GAINCONT

	{0x20, 0xd8},	//AWBLTO	# CSC F0
	{0x21, 0x70},	//AWBLBO	# CSC 70	
	{0x22, 0x90},	//AWBSNGRT	# Default : 98
	{0x23, 0x70},	//AWBSNGRB	# Default : 68
	{0x24, 0x90},	//AWBSNGBT	# Default : 8c
	{0x25, 0x70},	//AWBSNGBB	# Default : 74
	{0x26, 0x90},	//AWBSNGCT	# Default : 8c
	{0x27, 0x70},	//AWBSNGCB	# Default : 74
	{0x28, 0x20},	//WHTCNTL	# Default : 05
	{0x29, 0x05},	//WHTCNTN	# Default : 30

	{0x2a, 0xb4},	//RBLINE	# Default : b4 ?????
	{0x2b, 0xff},	//RBOVER	# Default : 20

	{0x30, 0x00},	//Default : 0
	{0x31, 0x10},	//Default : c8
	{0x32, 0x00},	//Default : 0
	{0x33, 0x10},	//Default : c8
	{0x34, 0x06},	//Default : 5
	{0x35, 0x30},	//Default : 78
	{0x36, 0x04},	//Default : 3
	{0x37, 0xa0},	//Default : e8
	{0x40, 0x01},	//Default : 1
	{0x41, 0x04},	//0x05 # Default : 2
	{0x42, 0x08},	//Default : 4
	{0x43, 0x10},	//Default : 8
	{0x44, 0x13},	//Default : 13
	{0x45, 0x6b},	//0x48 # Default : 6c
	{0x46, 0x82},	//Default : 82

	{0x50, 0xb0},
	{0x51, 0xc0},
	{0x52, 0x80},	//20130723
	//D60~D30~D20 Light Condition Detection
	{0x53, 0x89},	//94 AWB R Gain for D30 to D20  b9
	{0x54, 0xab},	//b8 AWB B Gain for D30 to D20  b5
	{0x55, 0x89},	//94 AWB R Gain for D20 to D30  b9
	{0x56, 0xab},	//b8 AWB B Gain for D20 to D30  b5
	{0x57, 0xa7},	//ba AWB R Gain for D65 to D30  db
	{0x58, 0x91},	//90 AWB B Gain for D65 to D30  8A
	{0x59, 0xa7},	//ba AWB R Gain for D30 to D65  db
	{0x5a, 0x91},	//90 AWB B Gain for D30 to D65  8A

	{0x70, 0xf0},	//DNP RGAIN BOT
	{0x71, 0xd2},	//DNP RGAIN TOP
	{0x72, 0x9c},	//DNP BGAIN BOT
	{0x73, 0x8c},	//DNP BGAIN TOP
	{0x74, 0x20},	//YDiff threshold for DNP
	{0x75, 0x30},	//DNP EXP TOP
	{0x76, 0x08},	//DNP EXP BOT
	{0x77, 0x24},	//DNPWINCNT

	{0x80, 0x00},	//AWBCONCNTR [7] Lock on/off [1] Singletone [0] Dark
	{0x81, 0xae},	//Single tone - Top
	{0x82, 0x90},	//Single tone - Bottom
	{0x83, 0x10},	//SNGTONTH : threshold for count of white area

	//IDP Block
	{0x00, 0x04},
	{0x10, 0xff},
	{0x11, 0x1d},
	{0x12, 0x1d},
	{0x13, 0xff},
	{0x14, 0x0f},
	{0x15, 0x01},

	//Shading
	{0x40, 0x08},	//08 R left
	{0x41, 0x00},	//08 R right
	{0x42, 0x00},	//08 R top
	{0x43, 0x00},	//08 R bottom
       
	{0x44, 0x0c},	//G left
	{0x45, 0x04},	//G right
	{0x46, 0x08},	//G top
	{0x47, 0x00},	//G bottom
    	   
	{0x48, 0x00},	//B left
	{0x49, 0x00},	//B right
	{0x4a, 0x00},	//B top
	{0x4b, 0x00},	//B bottom
    	   
	{0x50, 0x32},	//32 X Y High Center
	{0x51, 0x20},	//20 X Low Center     
	{0x52, 0x58},	//58 Y Low Center               
	{0x58, 0x80},	//78 R Center start gain
	{0x59, 0x80},	//80 G Center start gain                       
	{0x5a, 0x80},	//82 B Center start gain                       
    	
	{0x60, 0x22},	//32 R area A,B
	{0x61, 0x33},	//22 R area C,D
	{0x62, 0x44},	//33 R area E,F
	{0x63, 0x55},	//55 R area G,H
	{0x64, 0x66},	//65 R area I,J
    	
	{0x65, 0x11},	//21 G area A,B
	{0x66, 0x22},	//21 G area C,D
	{0x67, 0x33},	//32 G area E,F
	{0x68, 0x44},	//44 G area G,H
	{0x69, 0x55},	//54 G area I,J
    	
	{0x6a, 0x21},	//21 B area A,B
	{0x6b, 0x32},	//21 B area C,D
	{0x6c, 0x43},	//32 B area E,F
	{0x6d, 0x44},	//44 B area G,H
	{0x6e, 0x55},	//54 B area I,J  

	{0x6f, 0x00},

	//CMA
	{0x70, 0xaf}, //CMASB
	{0x71, 0x80}, //79 CMA11H  80
	{0x72, 0xae}, //ba CMA12H  ba
	{0x73, 0x12}, //0b CMA13H  05
	{0x74, 0xea}, //f0 CMA21H  F0
	{0x75, 0x7a}, //54 CMA22H  5A
	{0x76, 0xdc}, //fb CMA23H  F5
	{0x77, 0xfd}, //f6 CMA31H  F6
	{0x78, 0xa7}, //bf CMA32H  d9
	{0x79, 0x9d}, //8d CMA33H  76
	{0x7a, 0x0c}, //00 CMAL1
	{0x7b, 0x3e}, //00 CMAL2
	{0x7c, 0x26}, //00 CMAL3

	{0x80, 0x6f},	//CMBSB CWF
	{0x81, 0x79},	//79 CMB11H  71
	{0x82, 0xbb},	//ba CMB12H  b6
	{0x83, 0x0b},	//0b CMB13H  19
	{0x84, 0xf0},	//f0 CMB21H  E8
	{0x85, 0x7a},	//54 CMB22H  5a
	{0x86, 0xd6},	//fb CMB23H  Fe
	{0x87, 0xf0},	//f6 CMB31H  e8
	{0x88, 0x9a},	//bf CMB32H  ae
	{0x89, 0xb7},	//8d CMB33H  aa
	{0x8a, 0x3b},	//00 CMBL1
	{0x8b, 0x0d},	//00 CMBL2
	{0x8c, 0x34},	//00 CMBL3
          
	{0x90, 0x2f},	//CMCSB
	{0x91, 0x59},	//60 CMC11H  5C
	{0x92, 0xd8},	//d7 CMC12H  DD
	{0x93, 0x0e},	//08 CMC13H  06
	{0x94, 0xe4},	//f0 CMC21H  E0
	{0x95, 0x61},	//54 CMC22H  69
	{0x96, 0xfc},	//fb CMC23H  F6
	{0x97, 0xea},	//e3 CMC31H  E5
	{0x98, 0xa7},	//a7 CMC32H  AB
	{0x99, 0xb0},	//b7 CMC33H  AE
	{0x9a, 0x37},	//00 CMCL1
	{0x9b, 0x16},	//00 CMCL2
	{0x9c, 0x26},	//00 CMCL3

	//Gamma - R
	{0xa0, 0x00}, //00
	{0xa1, 0x0b}, //06
	{0xa2, 0x1a}, //0e
	{0xa3, 0x34}, //20
	{0xa4, 0x60}, //40
	{0xa5, 0x80}, //5c
	{0xa6, 0x96}, //74
	{0xa7, 0xa8}, //88
	{0xa8, 0xb6}, //99
	{0xa9, 0xc0}, //a8
	{0xaa, 0xc8}, //b6
	{0xab, 0xd8}, //cb
	{0xac, 0xe6}, //de
	{0xad, 0xf2}, //f0
	{0xae, 0xf8}, //f8
	{0xaf, 0xff}, //ff
	//Gamma - G
	{0xb0, 0x00}, //00
	{0xb1, 0x0b}, //06
	{0xb2, 0x1a}, //0e
	{0xb3, 0x34}, //20
	{0xb4, 0x60}, //40
	{0xb5, 0x80}, //5c           
	{0xb6, 0x96}, //74           
	{0xb7, 0xa8}, //88           
	{0xb8, 0xb6}, //99           
	{0xb9, 0xc0}, //a8           
	{0xba, 0xc8}, //b6           
	{0xbb, 0xd8}, //cb           
	{0xbc, 0xe6}, //de           
	{0xbd, 0xf2}, //f0           
	{0xbe, 0xf8}, //f8           
	{0xbf, 0xff}, //ff           
	//Gamma - B 
	{0xc0, 0x00},
	{0xc1, 0x0b},	//06 0c	  
	{0xc2, 0x1a},	//0e 14	
	{0xc3, 0x34},	//20 29	
	{0xc4, 0x60},	//40 4c	
	{0xc5, 0x80},	//5c 68 
	{0xc6, 0x96},	//74 80 
	{0xc7, 0xa8},	//88 92 
	{0xc8, 0xb6},	//99 a3 
	{0xc9, 0xc0},	//a8 b0 
	{0xca, 0xc8},	//b6 bc 
	{0xcb, 0xd8},	//cb d2 
	{0xcc, 0xe6},	//de e3 
	{0xcd, 0xf2},	//f0 f2 
	{0xce, 0xf8},	//f8 f8 
	{0xcf, 0xff},	//ff ff
         
	{0xd0, 0x00},	//GMACTRL [7] Adaptive gamma enable (R:Bright,G:Normal,B:Dark)
								//				[6] United gamma enable (Only G gamma)


	//IDP2
	{0x00, 0x05},

	{0X20, 0Xd3},

	//Y gamma
	{0x30, 0x00},	//00 YGMA 00
	{0x31, 0x02},	//02 YGMA 04
	{0x32, 0x05},	//05 YGMA 08
	{0x33, 0x0a},	//0c YGMA 16
	{0x34, 0x18},	//1b YGMA 32
	{0x35, 0x29},	//2b YGMA 48
	{0x36, 0x3b},	//3b YGMA 64
	{0x37, 0x4c},	//4c YGMA 80
	{0x38, 0x5d},	//5d YGMA 96
	{0x39, 0x6f},	//6f YGMA 112
	{0x3a, 0x81},	//81 YGMA 128
	{0x3b, 0xa2},	//a2 YGMA 160
	{0x3c, 0xc4},	//c4 YGMA 192
	{0x3d, 0xe4},	//e4 YGMA 224
	{0x3e, 0xf4},	//f4 YGMA 240
	{0x3f, 0xff},	//ff YGMA 255

	{0x40, 0x60},	//Cb point 60	
	{0x41, 0x70},	//Cb point 70
	{0x42, 0x78},	//Cb point 78
	{0x43, 0x7c},	//Cb point 7c
	{0x44, 0x84},	//Cb point 84
	{0x45, 0x88},	//Cb point 88
	{0x46, 0x90},	//Cb point 90
	{0x47, 0xa0},	//Cb point a0
	{0x48, 0x48},	//Cb gain

	{0x50, 0x60}, //Cr point 5c	
	{0x51, 0x70}, //Cr point 6c
	{0x52, 0x78}, //Cr point 78
	{0x53, 0x7c}, //Cr point 7c
	{0x54, 0x84}, //Cr point 84
	{0x55, 0x88}, //Cr point 88
	{0x56, 0x90}, //Cr point 8c
	{0x57, 0xa0}, //Cr point 9c
	{0x58, 0x5c}, //Cr gain

	{0x60, 0x18},	//Color suppress start
	{0x61, 0x04},	//Color suppress slope
	{0x62, 0x00},	//Color suppress end gain [4:0]

	//windowing
	{0x90, 0x00},
    	
	{0x94, 0x06},	//06
	{0x95, 0x40},	//40
	{0x96, 0x04},	//04
	{0x97, 0xb0},	//b0 


	//Memory
	{0xc0, 0x25},	//Recommend, M1SPDA
	{0xc1, 0x02},	//Recommend, M1SPDB
	{0xc2, 0x04},	//Recommend, M1SPDC
	{0xc3, 0x25},	//Recommend, M2SPDA
	{0xc4, 0x02},	//Recommend, M2SPDB
	{0xc5, 0x04},	//Recommend, M2SPDC

	//IDP3
	{0x00, 0x06},

	//DPCNR
	{0x10, 0xbe},	//Recommend, DPCNRCTRL
								//[7] DPC enable
								//[6] G1/G2 mixed DPC
								//[5] Gx min/max or med
								//[4] Cx min/max or med
								//[3] NR enable
								//[2] G1/G2 mixed NR
								//[1] nr_gopt X Weight 2
	{0x11, 0x10},	//Recommend, DPCTHV
	{0x12, 0x10},	//DPCTHVSLP
	{0x13, 0x10},	//DPCTHVDIFSRT
	{0x14, 0x10},	//DPCTHVDIFSLP
	{0x15, 0x10},	//BNRTHV
	{0x16, 0x20},	//BNRTHVSLPN 20130627
	{0x17, 0xff},	//BNRTHVSLPD
	{0x18, 0x0a},	//STRTNOR x1
	{0x19, 0x0a},	//Recommend, MAX_GAIN - h08, STRTDRK x5
	{0x1A, 0x10},	//BNRNEICNT [5:0] Minimum NR weight
	{0x1B, 0x00},	//Recommend, MEDDIV [3:0] 
	{0x1C, 0x3f},	//BNRTHV offset for R [7] sign bit
	{0x1D, 0x3f},	//BNRTHV offset for B [7] sign bit
	{0x1e, 0x00},	//CONERCTRL A,B
	{0x1f, 0x00},	//CONERCTRL C,D

	//Interpolation
	{0x20, 0xcd},	//INTCTRL 1f
	{0x21, 0x10},	//INTTHRMIN
	{0x22, 0xff},	//INTTHRMAX
	{0x23, 0x08},	//INTTHRSRT
	{0x24, 0x00},	//INTTHRSLP
	{0x25, 0x01},	//[0] Gedge opt
	{0x28, 0x80},	//FCSTGT08
	{0x29, 0x80},	//FCSTGT10

	//G edge
	{0x30, 0x1c},	//GEUGAIN 20130627
	{0x31, 0x1c},	//GEDGAIN 20130627
	{0x32, 0x03},	//GEUCORE
	{0x33, 0x03},	//GEDCORE
	{0x34, 0xff},	//GEUCLIP
	{0x35, 0xff},	//GEDCLIP
	{0x36, 0xff},	//GESTART
	{0x37, 0x80},	//GESLOP

	//Y Edge
	{0x40, 0x26},	//YEDGCNT		
	{0x41, 0x18},	//YEUGAIN		
	{0x42, 0x20},	//YEDGAIN		
	{0x43, 0x02},	//YEUCORE		
	{0x44, 0x18},	//YEUCORSTRT	        
	{0x45, 0xff},	//YEUCOREND        
	{0x46, 0x00},	//YEUCORSLOP	        
	{0x47, 0x02},	//YEDCORE		
	{0x48, 0x18},	//YEDCORSTRT	        
	{0x49, 0xff},	//YEDCOREND	        
	{0x4a, 0x00},	//YEDCORSLOP	        
	{0x4b, 0xff},	//YEUCLIP		
	{0x4c, 0xff},	//YEDCLIP		
	{0x4d, 0x00},	//YLEVEL		
	{0x4e, 0x00},	//YEEDNOPT
	{0x4f, 0x27},	//YEEDNCNT

	{0x50, 0xe0},	//UVFILT			
	{0x51, 0x89},	//CBCORTHD	                
	{0x52, 0x89},	//CRCORTHD	                
    	
	{0x53, 0x44},	//EDGOPT       	   	
	{0x54, 0x73},	//EDGSTRGTH    	   	
	{0x55, 0x28},	//LUMISTUSD    	   	
	{0x56, 0x08},	//LUMISTUSO    	   	
	{0x57, 0xea},	//EDGSMAR      	
    	
	{0x58, 0x18},	//EESTHRMIN Y sigma filter
	{0x59, 0xff},	//EESTHRMAX
	{0x5a, 0x20},	//EESTHRSRT
	{0x5b, 0x80},	//EESTHRSLP

	{0x5c, 0x18},	//EECTHRMIN Cx sigma filter
	{0x5d, 0xff},	//EECTHRMAX
	{0x5e, 0x20},	//EECTHRSRT
	{0x5f, 0x80},	//EECTHRSLP	
    	
	{0x60, 0xff}, //YESTART      	
	{0x61, 0x80}, //YESLOP
	{0x62, 0x00}, //YEMIN [4:0]
	        
	{0x63, 0x4d},	//CEEDNOPT	        
	{0x64, 0x4c},	//ESRNKCTRL
    	
	//SNR	
	{0x00, 0x01},
	{0x03, 0x01},
	{0xff, 0xff},
};

/* 1600X1200 UXGA */
static struct reginfo sensor_uxga[] =
{
	{0x00, 0x01},
	{0x04, 0x50},
	{0x30, 0x00},
	{0x16, 0xa9}, //64M
	
	//PLL setting
	{0x08, 0x90},
	{0x0a, 0x20},	//MCLK24M, PCLK64M
		
	//BLANK Time
	{0x39, 0x00},
	{0x3a, 0x26},
	{0x3d, 0x00},
	{0x3e, 0x1c},
    	         
	//CDS timing
	{0x60, 0x03},
	{0x61, 0x0b},
	{0x62, 0xfb},
	{0x63, 0x0d},
	{0x64, 0x23},
	{0x65, 0x8b},
	{0x66, 0xab},
	{0x67, 0x05},
	{0x68, 0x4b},
	{0x69, 0x08},
	{0x6a, 0x48},
	{0x6b, 0x0b},
	{0x6c, 0x45},
	{0x6d, 0x13},
	{0x6e, 0x43},
	{0x6f, 0x7d},
	{0x70, 0x23},
	{0x71, 0xf8},
	{0x72, 0x83},
	{0x73, 0x85},
	{0x74, 0x05},
	{0x75, 0x41},
	{0x76, 0xfb},
	{0x77, 0x08},
	{0x78, 0x21},
	{0x79, 0x88},
	{0x7a, 0x11},
	{0x7b, 0x18},
	{0x7c, 0x35},
	{0x7d, 0x10},
	{0x7e, 0x33},
	{0x7f, 0x00},
	{0x80, 0xfd},
	{0x81, 0x00},
	{0x82, 0xfd},
	{0x83, 0x10},
	{0x84, 0x35},
    	
	{0x00, 0x02},
	{0x36, 0x9c},
	
	{0x00, 0x03},
	{0x1d, 0x94},	//0x9c - h08
	
	//windowing  
	{0x00, 0x05},
	{0x90, 0x00},
	             
	{0x94, 0x06},
	{0x95, 0x40},
	{0x96, 0x04},
	{0x97, 0xb0},
	{0xff, 0xff},
};

/* 1280X1024 SXGA
static struct reginfo sensor_sxga[] =
{
	{0x00, 0x00},
	{0x04, 0x10},
	{0x05, 0x8b},
	{0x00, 0x03},
	{0x94, 0x05},
	{0x95, 0x00},
	{0x96, 0x04},
	{0x97, 0x00}, 
	{0xff, 0xff}, 
}; 

static struct reginfo sensor_xga[] =
{
	{0xff, 0xff}
};*/

/* 1280X720 HD*/
static struct reginfo sensor_hd[] = 
{
	{0x00, 0x01},
	{0x04, 0x50},
	{0x30, 0x80},
	{0x16, 0xa9},	//64M
	
	//PLL setting
	{0x08, 0x90},
	{0x0a, 0x20},	//MCLK24M, PCLK64M

	//BLANK Time		
	{0x39, 0x00},
	{0x3a, 0x26},
	{0x3d, 0x00},
	{0x3e, 0x1b},
    	         
	//CDS Timing    	         
	{0x60, 0x03},
	{0x61, 0x0b},
	{0x62, 0xfb},
	{0x63, 0x0d},
	{0x64, 0x23},
	{0x65, 0x8b},
	{0x66, 0xab},
	{0x67, 0x05},
	{0x68, 0x4b},
	{0x69, 0x08},
	{0x6a, 0x48},
	{0x6b, 0x0b},
	{0x6c, 0x45},
	{0x6d, 0x13},
	{0x6e, 0x43},
	{0x6f, 0x7d},
	{0x70, 0x23},
	{0x71, 0xf8},
	{0x72, 0x83},
	{0x73, 0x85},
	{0x74, 0x05},
	{0x75, 0x41},
	{0x76, 0xfb},
	{0x77, 0x08},
	{0x78, 0x21},
	{0x79, 0x88},
	{0x7a, 0x11},
	{0x7b, 0x18},
	{0x7c, 0x35},
	{0x7d, 0x10},
	{0x7e, 0x33},
	{0x7f, 0x00},
	{0x80, 0xfd},
	{0x81, 0x00},
	{0x82, 0xfd},
	{0x83, 0x10},
	{0x84, 0x35},
    	
	{0x00, 0x02},
	{0x36, 0xc0},
	
	{0x00, 0x03},
	{0x1d, 0xb8},	//0xc0 - h08
	
	//windowing
	{0x00, 0x05},  
	{0x90, 0x00},
	             
	{0x94, 0x05},
	{0x95, 0x00},
	{0x96, 0x02},
	{0x97, 0xd0},
	{0xff, 0xff},
};

/* 800X600 SVGA*/
static struct reginfo sensor_svga[] =
{
	{0x00, 0x01},
	{0x04, 0x40},
	{0x30, 0x55},
	{0x16, 0xaa},	//32M
    	    
	//PLL setting
	{0x00, 0x01},
	{0x08, 0x90},
	{0x0a, 0x29},	//MCLK24M, PCLK36M

	//BLANK Time	    	    
	{0x39, 0x00},
	{0x3a, 0x32},
	{0x3d, 0x00},
	{0x3e, 0x1c},
    	    
	//CDS Timing    	    
	{0x60, 0x0d},
	{0x61, 0x95},
	{0x62, 0x05},
	{0x63, 0x17},
	{0x64, 0x2d},
	{0x65, 0x95},
	{0x66, 0xb5},
	{0x67, 0x05},
	{0x68, 0x4b},
	{0x69, 0x08},
	{0x6a, 0x4b},
	{0x6b, 0x0b},
	{0x6c, 0x45},
	{0x6d, 0x13},
	{0x6e, 0x43},
	{0x6f, 0x87},
	{0x70, 0x2d},
	{0x71, 0x02},
	{0x72, 0x8d},
	{0x73, 0x8f},
	{0x74, 0xc3},
	{0x75, 0x4b},
	{0x76, 0x05},
	{0x77, 0x12},
	{0x78, 0x2b},
	{0x79, 0x92},
	{0x7a, 0x11},
	{0x7b, 0x22},
	{0x7c, 0x3f},
	{0x7d, 0x10},
	{0x7e, 0x3d},
	{0x7f, 0x10},
	{0x80, 0x07},
	{0x81, 0x01},
	{0x82, 0x07},
	{0x83, 0x10},
	{0x84, 0x3f},
    	    
	//AE Block
	{0x00, 0x02},
	{0x36, 0xa5},

	//AWB Block
	{0x00, 0x03},
	{0x1d, 0x9d},	//0xa5 -h08
	
	//windowing  
	{0x00, 0x05},
	{0x90, 0x00},
	             
	{0x94, 0x03},
	{0x95, 0x20},
	{0x96, 0x02},
	{0x97, 0x58},
	{0xff, 0xff},
};

/* 640X480 VGA 
static struct reginfo sensor_vga[] =
{
	{0x00,	0x00},
	{0x04,	0x00},
	{0x05,	0x0f},
	{0x00,	0x03},
	{0x94,	0x02}, //0x02
	{0x95,	0x80}, //0x80
	{0x96,	0x01},//0x01
	{0x97,	0xe0}, //0xe0
	{0xff,	0xff}, 
};*/

/* 352X288 CIF 
static struct reginfo sensor_cif[] =
{
	{0x00, 0x03},
	{0x94, 0x01},
	{0x95, 0x60},
	{0x96, 0x01},
	{0x97, 0x20}, 
	{0xff, 0xff}, 
};*/

/* 320*240 QVGA 
static  struct reginfo sensor_qvga[] =
{
	{0x00,	0x03},
	{0x94,	0x01},
	{0x95,	0x40},
	{0x96,	0x00},
	{0x97,	0xf0}, 
	{0xff,	0xff}, 
};*/

/* 176X144 QCIF
static struct reginfo sensor_qcif[] =
{
	{0x00, 0x03},
	{0x94, 0x00},
	{0x95, 0xB0},
	{0x96, 0x00},
	{0x97, 0x90}, 
	{0xff, 0xff},
};*/
#endif
static  struct reginfo sensor_ClrFmt_YUYV[]=
{
	{0xff, 0xff}
};

static  struct reginfo sensor_ClrFmt_UYVY[]=
{
	{0xff, 0xff}
};

#if CONFIG_SENSOR_WhiteBalance
static  struct reginfo sensor_WhiteB_Auto[]={
	{0x00, 0x03},
	{0x10, 0xd0},
	{0xff, 0xff},
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct reginfo sensor_WhiteB_Cloudy[]=
{
	{0x00, 0x03},
	{0x10, 0x00},
	{0x50, 0xb9},
	{0x51, 0x7c},
	{0xff, 0xff}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct reginfo sensor_WhiteB_ClearDay[]=
{
	{0x00, 0x03},
	{0x10, 0x00},
	{0x50, 0x93},
	{0x51, 0x92},
	{0xff, 0xff}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct reginfo sensor_WhiteB_TungstenLamp1[]=
{
	{0x00, 0x03},
	{0x10, 0x00},
	{0x50, 0x97},
	{0x51, 0xa6},
	{0xff, 0xff}
};
/* Home Colour Temperature : 2500K - 3500K  */
static  struct reginfo sensor_WhiteB_TungstenLamp2[]=
{
	{0x00, 0x03},
	{0x10, 0x00},
	{0x50, 0x80},
	{0x51, 0xa8},
	{0xff, 0xff}
};
static struct reginfo *sensor_WhiteBalanceSeqe[] = {sensor_WhiteB_Auto, sensor_WhiteB_TungstenLamp1,sensor_WhiteB_TungstenLamp2,
    sensor_WhiteB_ClearDay, sensor_WhiteB_Cloudy,NULL,
};
#endif

#if CONFIG_SENSOR_Brightness
static  struct reginfo sensor_Brightness0[]=
{
	// Brightness -2
	{0x00, 0x05},
	{0x21, 0xa0},
	{0xff, 0xff}
};

static  struct reginfo sensor_Brightness1[]=
{
	// Brightness -1
	{0x00, 0x05},
	{0x21, 0x90},
	{0xff, 0xff}
};

static  struct reginfo sensor_Brightness2[]=
{
	//  Brightness 0
	{0x00, 0x05},
	{0x21, 0x00},
	{0xff, 0xff}
};

static  struct reginfo sensor_Brightness3[]=
{
	 // Brightness +1
	{0x00, 0x05},
	{0x21, 0x10},
	{0xff, 0xff}
};

static  struct reginfo sensor_Brightness4[]=
{
	//  Brightness +2
	{0x00, 0x05},
	{0x21, 0x20},
	{0xff, 0xff}

};

static  struct reginfo sensor_Brightness5[]=
{
   {0x00, 0x05},
   {0x21, 0x30},
   {0xff, 0xff}
};
static struct reginfo *sensor_BrightnessSeqe[] = {sensor_Brightness0, sensor_Brightness1, sensor_Brightness2, sensor_Brightness3,
    sensor_Brightness4, sensor_Brightness5,NULL,
};

#endif

#if CONFIG_SENSOR_Effect
static  struct reginfo sensor_Effect_Normal[] =
{
    {0x00, 0x05},
    {0x70, 0x00},
    {0xff, 0xff}
};

static  struct reginfo sensor_Effect_WandB[] =
{
    {0x00, 0x05},
    {0x70, 0x04},
    {0xff, 0xff}
};

static  struct reginfo sensor_Effect_Sepia[] =
{
    {0x00, 0x05},
    {0x70, 0x20},
    {0xff, 0xff}
};

static  struct reginfo sensor_Effect_Negative[] =
{
   {0x00, 0x05},
   {0x70, 0x01},
   {0xff, 0xff}
};
static  struct reginfo sensor_Effect_Bluish[] =
{
	//Bluish
	{0x00, 0x05},
	{0x70, 0x10},
	{0xff, 0xff}
};

static  struct reginfo sensor_Effect_Green[] =
{
	//Greenish
	{0x00, 0x05},
	{0x70, 0x08},
	{0xff, 0xff}
};
static struct reginfo *sensor_EffectSeqe[] = {sensor_Effect_Normal, sensor_Effect_WandB, sensor_Effect_Negative,sensor_Effect_Sepia,
    sensor_Effect_Bluish, sensor_Effect_Green,NULL,
};
#endif
#if CONFIG_SENSOR_Exposure
//[Exposure Value]
static  struct reginfo sensor_Exposure0[]=
{
    //-6
    {0x00, 0x02},
    {0x12, 0x22},
    {0x13, 0x22},
    {0x14, 0x22},
    {0x26, 0x24},
		{0x00, 0x05},
		{0x21, 0xa4},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure1[]=
{
    //-5
    {0x00, 0x02},
    {0x12, 0x28},
    {0x13, 0x28},
    {0x14, 0x28},
    {0x26, 0x2a},
		{0x00, 0x05},
		{0x21, 0xa4},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure2[]=
{
    //-4
    {0x00, 0x02},
    {0x12, 0x28},
    {0x13, 0x28},
    {0x14, 0x28},
    {0x26, 0x2a},
		{0x00, 0x05},
		{0x21, 0x98},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure3[]=
{
    //-3
    {0x00, 0x02},
    {0x12, 0x2e},
    {0x13, 0x2e},
    {0x14, 0x2e},
    {0x26, 0x30},
		{0x00, 0x05},
		{0x21, 0x98},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure4[]=
{
    //-2
    {0x00, 0x02},
    {0x12, 0x2e},
    {0x13, 0x2e},
    {0x14, 0x2e},
    {0x26, 0x30},
		{0x00, 0x05},
		{0x21, 0x8c},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure5[]=
{
    //-0.3EV
    {0x00, 0x02},
    {0x12, 0x36},
    {0x13, 0x36},
    {0x14, 0x36},
    {0x26, 0x38},
		{0x00, 0x05},
		{0x21, 0x8c},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure6[]=
{
    //default
    {0x00, 0x02},
    {0x12, 0x36},
    {0x13, 0x36},
    {0x14, 0x36},
    {0x26, 0x38},
		{0x00, 0x05},
		{0x21, 0x00},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure7[]=
{
    // 1
    {0x00, 0x02},
    {0x12, 0x36},
    {0x13, 0x36},
    {0x14, 0x36},
    {0x26, 0x38},
		{0x00, 0x05},
		{0x21, 0x08},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure8[]=
{
    // 2
    {0x00, 0x02},
    {0x12, 0x3e},
    {0x13, 0x3e},
    {0x14, 0x3e},
    {0x26, 0x40},
		{0x00, 0x05},
		{0x21, 0x08},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure9[]=
{
    // 3
    {0x00, 0x02},
    {0x12, 0x3e},
    {0x13, 0x3e},
    {0x14, 0x3e},
    {0x26, 0x40},
		{0x00, 0x05},
		{0x21, 0x12},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure10[]=
{
    // 4
    {0x00, 0x02},
    {0x12, 0x46},
    {0x13, 0x46},
    {0x14, 0x46},
    {0x26, 0x48},
		{0x00, 0x05},
		{0x21, 0x12},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure11[]=
{
    // 5
    {0x00, 0x02},
    {0x12, 0x46},
    {0x13, 0x46},
    {0x14, 0x46},
    {0x26, 0x48},
		{0x00, 0x05},
		{0x21, 0x1a},
		{0xff, 0xff}
};

static  struct reginfo sensor_Exposure12[]=
{
    // 6
    {0x00, 0x02},
    {0x12, 0x50},
    {0x13, 0x50},
    {0x14, 0x50},
    {0x26, 0x52},
		{0x00, 0x05},
		{0x21, 0x1a},
		{0xff, 0xff}
};

#if ASUS_CAMERA_SUPPORT
static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure0, sensor_Exposure1, sensor_Exposure2, sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,sensor_Exposure7,sensor_Exposure8,sensor_Exposure9,
    sensor_Exposure10,sensor_Exposure11,sensor_Exposure12,NULL,
};
#else
static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,sensor_Exposure7,sensor_Exposure8,sensor_Exposure9,
    NULL,
};
#endif
#endif
#if CONFIG_SENSOR_Saturation
static  struct reginfo sensor_Saturation0[]=
{
		{0x00, 0x05},
		{0x48, 0x48},
		{0x58, 0x5c},
    {0xff, 0xff}
};

static  struct reginfo sensor_Saturation1[]=
{
		{0x00, 0x05},
		{0x48, 0x50},
		{0x58, 0x64},
		{0xff, 0xff}
};

static  struct reginfo sensor_Saturation2[]=
{
		{0x00, 0x05},
		{0x48, 0x58},
		{0x58, 0x6c},
		{0xff, 0xff}
};
static struct reginfo *sensor_SaturationSeqe[] = {sensor_Saturation0, sensor_Saturation1, sensor_Saturation2, NULL,};

#endif
#if CONFIG_SENSOR_Contrast
static  struct reginfo sensor_Contrast0[]=
{
    {0xff, 0xff}
};

static  struct reginfo sensor_Contrast1[]=
{
   {0xff, 0xff}
};

static  struct reginfo sensor_Contrast2[]=
{
    {0xff, 0xff}
};

static  struct reginfo sensor_Contrast3[]=
{
    {0xff, 0xff}
};

static  struct reginfo sensor_Contrast4[]=
{
   {0xff, 0xff}
};


static  struct reginfo sensor_Contrast5[]=
{
    {0xff, 0xff}
};

static  struct reginfo sensor_Contrast6[]=
{
    {0xff, 0xff}
};
static struct reginfo *sensor_ContrastSeqe[] = {sensor_Contrast0, sensor_Contrast1, sensor_Contrast2, sensor_Contrast3,
    sensor_Contrast4, sensor_Contrast5, sensor_Contrast6, NULL,
};

#endif
#if CONFIG_SENSOR_Mirror
static  struct reginfo sensor_MirrorOn[]=
{
		{0x00, 0x01},
		{0x04, 0x51},
    {0xff, 0xff}
};

static  struct reginfo sensor_MirrorOff[]=
{
		{0x00, 0x01},
		{0x04, 0x50},
    {0xff, 0xff}
};
static struct reginfo *sensor_MirrorSeqe[] = {sensor_MirrorOff, sensor_MirrorOn,NULL,};
#endif
#if CONFIG_SENSOR_Flip
static  struct reginfo sensor_FlipOn[]=
{
		{0x00, 0x01},
		{0x04, 0x52},
    {0xff, 0xff}
};

static  struct reginfo sensor_FlipOff[]=
{
		{0x00, 0x01},
		{0x04, 0x50},
		{0xff, 0xff}
};

static struct reginfo *sensor_FlipSeqe[] = {sensor_FlipOff, sensor_FlipOn,NULL,};

#endif
#if CONFIG_SENSOR_Scene
static  struct reginfo sensor_SceneAuto[] =
{
   {0x00, 0x02},
   {0x11, 0x08},
   {0x00, 0x05},
   {0x21, 0x00},
   {0xff, 0xff}
};

static  struct reginfo sensor_SceneNight[] =
{
   {0x00, 0x02},
   {0x11, 0x14},
   {0x00, 0x05},
   {0x21, 0x10},
   {0xff, 0xff}
};
static struct reginfo *sensor_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};
#endif

#if CONFIG_SENSOR_AntiBanding
static  struct reginfo sensor_antibanding60hz_uxga[] =
{
	//60Hz
	//uxga
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0x26},
	{0x3d, 0x00},
	{0x3e, 0x1c},
	{0x00, 0x02},
	{0x36, 0x9c},
	{0xff, 0xff}
};

static  struct reginfo sensor_antibanding60hz_hd[] =
{
	//60Hz
	//hd
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0x26},
	{0x3d, 0x00},
	{0x3e, 0x1b},
	{0x00, 0x02},
	{0x36, 0xc0},
	{0xff, 0xff}
};

static  struct reginfo sensor_antibanding60hz_svga[] =
{
	//60Hz
	//svga
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0x32},
	{0x3d, 0x00},
	{0x3e, 0x1c},
	{0x00, 0x02},
	{0x36, 0xa5},
	{0xff, 0xff}
};

static	struct reginfo sensor_antibanding50hz_uxga[] =
{
	//50Hz
	//uxga
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0x6a},
	{0x3d, 0x00},
	{0x3e, 0x15},
	{0x00, 0x02},
	{0x36, 0xbc},
	{0xff, 0xff}
};

static	struct reginfo sensor_antibanding50hz_hd[] =
{
	//50Hz
	//hd
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0xc2},
	{0x3d, 0x00},
	{0x3e, 0x18},
	{0x00, 0x02},
	{0x36, 0xe7},
	{0xff, 0xff}
};

static	struct reginfo sensor_antibanding50hz_svga[] =
{
	//50Hz
	//svga
	{0x00, 0x01},
	{0x39, 0x00},
	{0x3a, 0xb6},
	{0x3d, 0x00},
	{0x3e, 0x1c},
	{0x00, 0x02},
	{0x36, 0xc6},
	{0xff, 0xff}
};
static struct reginfo *sensor_antibanding_uxga[] = {sensor_antibanding50hz_uxga, sensor_antibanding60hz_uxga,NULL,};
static struct reginfo *sensor_antibanding_hd[] = {sensor_antibanding50hz_hd, sensor_antibanding60hz_hd,NULL,};
static struct reginfo *sensor_antibanding_svga[] = {sensor_antibanding50hz_svga, sensor_antibanding60hz_svga,NULL,};
#endif

#if CONFIG_SENSOR_ExposureLock
static	struct reginfo sensor_ExposureLock[] = 
{
	//AE Lock
	{0x00, 0x02},
	{0x10, 0xD8},
	{0xff, 0xff}
};

static	struct reginfo sensor_ExposureUnLock[] = 
{
	//AE Unlock
	{0x00, 0x02},
	{0x10, 0x98},
	{0xff, 0xff}
};
static struct reginfo *sensor_ExposureLk[] = {sensor_ExposureUnLock, sensor_ExposureLock, NULL,};
#endif

#if CONFIG_SENSOR_WhiteBalanceLock
static	struct reginfo sensor_WhiteBalanceLock[] = 
{
	//AWB Lock
	{0x00, 0x03},
	{0x10, 0x70},
	{0xff, 0xff}
};

static	struct reginfo sensor_WhiteBalanceUnLock[] = 
{
	//AWB Unlock
	{0x00, 0x03},
	{0x10, 0x50},
	{0xff, 0xff}
};
static struct reginfo *sensor_WhiteBalanceLk[] = {sensor_WhiteBalanceUnLock, sensor_WhiteBalanceLock,NULL,};
#endif

#if CONFIG_SENSOR_ISO
static	struct reginfo sensor_ISO_100[] = 
{
	{0x00, 0x02},
	{0x40, 0x0a},
	{0x41, 0x0a},
	{0x42, 0x00},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x4e, 0x14},
	{0x7b, 0x14},

	{0x00, 0x06},
	{0x15, 0x10},
	{0x16, 0x00},
	{0x17, 0x80},
	{0x18, 0x0a},
	{0x19, 0x0c},

	{0x30, 0x18},
	{0x31, 0x18},

	{0x00, 0x02},
	{0x32, 0x00},
	{0x33, 0x14},
	{0x34, 0x00},
	{0xff, 0xff}
};

static	struct reginfo sensor_ISO_200[] = 
{
	{0x00, 0x02},
	{0x40, 0x10},
	{0x41, 0x10},
	{0x42, 0x00},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x4e, 0x20},
	{0x7b, 0x20},
	
	{0x00, 0x06},
	{0x15, 0x10},
	{0x16, 0x10},
	{0x17, 0x80},
	{0x18, 0x10},
	{0x19, 0x18},
	
	{0x30, 0x18},
	{0x31, 0x18},
	
	{0x00, 0x02},
	{0x32, 0x01},
	{0x33, 0x10},
	{0x34, 0x00},
	{0xff, 0xff}
};

/*
static	struct reginfo sensor_ISO_400[] = 
{
	{0x00, 0x02},
	{0x40, 0x20},
	{0x41, 0x20},
	{0x42, 0x00},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x4e, 0x40},
	{0x7b, 0x40},
	
	{0x00, 0x06},
	{0x15, 0x10},
	{0x16, 0x40},
	{0x17, 0x40},
	{0x18, 0x20},
	{0x19, 0x38},
	
	{0x30, 0x18},
	{0x31, 0x18},
	
	{0x00, 0x02},
	{0x32, 0x02},
	{0x33, 0x10},
	{0x34, 0x00},
	{0xff, 0xff}
};
*/

static	struct reginfo sensor_ISO_Auto[] = 
{
	{0x00, 0x02},
	{0x40, 0x0a},
	{0x41, 0x0a},
	{0x42, 0x00},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x4e, 0x20},
	{0x7b, 0x20},
	
	{0x00, 0x06},
	{0x15, 0x10},
	{0x16, 0x10},
	{0x17, 0x80},
	{0x18, 0x0a},
	{0x19, 0x10},
	
	{0x30, 0x18},
	{0x31, 0x18},
	
	{0x00, 0x02},
	{0x32, 0x00},
	{0x33, 0x14},
	{0x34, 0x00},
	{0xff, 0xff}
};
static struct reginfo *sensor_ISO[] = {sensor_ISO_Auto, sensor_ISO_100, sensor_ISO_200, NULL};
#endif

#if CONFIG_SENSOR_DigitalZoom
static struct reginfo sensor_Zoom0[] =
{
    {0xff, 0xff}
};

static struct reginfo sensor_Zoom1[] =
{
     {0xff, 0xff}
};

static struct reginfo sensor_Zoom2[] =
{
    {0xff, 0xff}
};


static struct reginfo sensor_Zoom3[] =
{
    {0xff, 0xff}
};
static struct reginfo *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL,};
#endif
static const struct v4l2_querymenu sensor_menus[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "fluorescent",  .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "incandescent", .reserved = 0,}, {  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 4,  .name = "cloudy-daylight", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Effect
    { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 4, .name = "posterize", .reserved = 0,} ,{ .id = V4L2_CID_EFFECT,  .index = 5,  .name = "aqua", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Scene
    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_AntiBanding
    { .id = V4L2_CID_ANTIBANDING,  .index = 0, .name = "50hz", .reserved = 0,} ,{ .id = V4L2_CID_ANTIBANDING,  .index = 1, .name = "60hz", .reserved = 0,} ,
    #endif

	#if CONFIG_SENSOR_ISO
		{ .id = V4L2_CID_ISO,	.index = 0, .name = "auto", .reserved = 0, },
		{ .id = V4L2_CID_ISO,	.index = 1, .name = "100",	.reserved = 0, },
		{ .id = V4L2_CID_ISO,	.index = 2, .name = "200",	.reserved = 0, },
		//{ .id = V4L2_CID_ISO,	.index = 3, .name = "400",	.reserved = 0, },
		//{ .id = V4L2_CID_ISO,	.index = 4, .name = "800",	.reserved = 0, },
		//{ .id = V4L2_CID_ISO,	.index = 5, .name = "1600", .reserved = 0, },
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
        .default_value = 0,
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
        .maximum	= 5,
        .step		= 1,
        .default_value = 0,
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
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Saturation
	{
        .id		= V4L2_CID_SATURATION,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Saturation Control",
        .minimum	= 0,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Contrast
	{
        .id		= V4L2_CID_CONTRAST,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Contrast Control",
        .minimum	= -3,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
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
        .default_value = 1,
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
        .id		= V4L2_CID_ANTIBANDING,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Antibanding Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_WhiteBalanceLock
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
    #endif

	#if CONFIG_SENSOR_Flash
	{
        .id		= V4L2_CID_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
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
#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
static int sensor_deactivate(struct i2c_client *client);

static struct soc_camera_ops sensor_ops =
{
    .suspend                     = sensor_suspend,
    .resume                       = sensor_resume,
    .set_bus_param		= sensor_set_bus_param,
    .query_bus_param	= sensor_query_bus_param,
    .controls		= sensor_controls,
    .menus                         = sensor_menus,
    .num_controls		= ARRAY_SIZE(sensor_controls),
    .num_menus		= ARRAY_SIZE(sensor_menus),
};

/* only one fixed colorspace per pixelcode */
struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
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
    {V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}	
};

typedef struct sensor_info_priv_s
{
    int whiteBalance;
    int brightness;
    int contrast;
    int saturation;
    int effect;
    int scene;
	int antibanding;
	int WhiteBalanceLock;
	int ExposureLock;
	int iso;
    int digitalzoom;
    int focus;
    int flash;
    int exposure;
	bool snap2preview;
	bool video2preview;
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                          /* VFLIP */
    unsigned int winseqe_cur_addr;
	struct sensor_datafmt fmt;
    unsigned int funmodule_state;
	//preview or capture
	int outputSize; // supported resolution
	int curRes;
	int curPreviewCapMode;
	int supportedSize[10];
	int supportedSizeNum;
} sensor_info_priv_t;

struct sensor
{
    struct v4l2_subdev subdev;
    struct i2c_client *client;
    sensor_info_priv_t info_priv;
    int model;	/* V4L2_IDENT_OV* codes from v4l2-chip-ident.h */
#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_t tasklock_cnt;
#endif
	struct rk29camera_platform_data *sensor_io_request;
    struct rk29camera_gpio_res *sensor_gpio_res;
};

struct modify_iso_struct
{
	struct i2c_client *client;
	struct delayed_work modify_iso_work;
	int modify_iso_flag;
};
struct modify_iso_struct modify_iso;

static struct sensor* to_sensor(const struct i2c_client *client)
{
    return container_of(i2c_get_clientdata(client), struct sensor, subdev);
}

static int sensor_task_lock(struct i2c_client *client, int lock)
{
#if CONFIG_SENSOR_I2C_NOSCHED
	int cnt = 3;
    struct sensor *sensor = to_sensor(client);

	if (lock) {
		if (atomic_read(&sensor->tasklock_cnt) == 0) {
			while ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt>0)) {
				SENSOR_TR("\n %s will obtain i2c in atomic, but i2c bus is locked! Wait...\n",SENSOR_NAME_STRING());
				msleep(35);
				cnt--;
			}
			if ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt<=0)) {
				SENSOR_TR("\n %s obtain i2c fail in atomic!!\n",SENSOR_NAME_STRING());
				goto sensor_task_lock_err;
			}
			preempt_disable();
		}

		atomic_add(1, &sensor->tasklock_cnt);
	} else {
		if (atomic_read(&sensor->tasklock_cnt) > 0) {
			atomic_sub(1, &sensor->tasklock_cnt);

			if (atomic_read(&sensor->tasklock_cnt) == 0)
				preempt_enable();
		}
	}
	return 0;
sensor_task_lock_err:
	return -1;  
#else
    return 0;
#endif

}

/* sensor register write */
static int sensor_write(struct i2c_client *client, u8 reg, u8 val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[1];

    //buf[0] = reg >> 8;
    buf[0] = reg & 0xFF;
    buf[1] = val;

    msg->addr = client->addr;
    msg->flags = client->flags;
    msg->buf = buf;
    msg->len = sizeof(buf);
    msg->scl_rate = CONFIG_SENSOR_I2C_SPEED;         /* ddl@rock-chips.com : 100kHz */
    msg->read_type = 0;               /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;

    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            return 0;
        } else {
            SENSOR_TR("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n",SENSOR_NAME_STRING(),reg, val);
            udelay(10);
        }
    }

    return err;
}

/* sensor register read */
static int sensor_read(struct i2c_client *client, u8 reg, u8 *val)
{
    int err,cnt;
    u8 buf[1];
    struct i2c_msg msg[2];

   // buf[0] = reg >> 8;
    buf[0] = reg & 0xFF;

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = buf;
    msg[0].len = sizeof(buf);
    msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;       /* ddl@rock-chips.com : 100kHz */
    msg[0].read_type = 2;   /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    msg[1].addr = client->addr;
    msg[1].flags = client->flags|I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = 1;
    msg[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;                       /* ddl@rock-chips.com : 100kHz */
    msg[1].read_type = 2;                             /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;
    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 2);

        if (err >= 0) {
            *val = buf[0];
            return 0;
        } else {
        	SENSOR_TR("\n %s read reg(0x%x val:0x%x) failed, try to read again! \n",SENSOR_NAME_STRING(),reg, *val);
            udelay(10);
        }
    }

    return err;
}

/* write a array of registers  */
static int sensor_write_array(struct i2c_client *client, struct reginfo *regarray)
{
    int err = 0, cnt;
    int i = 0;
#if CONFIG_SENSOR_I2C_RDWRCHK    
	char valchk;
#endif

	cnt = 0;
	if (sensor_task_lock(client, 1) < 0)
		goto sensor_write_array_end;
    while (regarray[i].reg != 0xff)
    {
        err = sensor_write(client, regarray[i].reg, regarray[i].val);
        if (err < 0)
        {
            if (cnt-- > 0) {
			    SENSOR_TR("%s..write failed current reg:0x%x, Write array again !\n", SENSOR_NAME_STRING(),regarray[i].reg);
				i = 0;
				continue;
            } else {
                SENSOR_TR("%s..write array failed!!!\n", SENSOR_NAME_STRING());
                err = -EPERM;
				goto sensor_write_array_end;
            }
        } else {
        #if CONFIG_SENSOR_I2C_RDWRCHK
			sensor_read(client, regarray[i].reg, &valchk);
			if (valchk != regarray[i].val)
				SENSOR_TR("%s Reg:0x%x write(0x%x, 0x%x) fail\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);
		#endif
        }
        i++;
    }

sensor_write_array_end:
	sensor_task_lock(client,0);
	return err;
}
#if CONFIG_SENSOR_I2C_RDWRCHK
static int sensor_readchk_array(struct i2c_client *client, struct reginfo *regarray)
{
    int cnt;
    int i = 0;
	char valchk;

	cnt = 0;
	valchk = 0;
    while (regarray[i].reg != 0xff)
    {
		sensor_read(client, regarray[i].reg, &valchk);
		if (valchk != regarray[i].val)
			SENSOR_TR("%s Reg:0x%x read(0x%x, 0x%x) error\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);

        i++;
    }
    return 0;
}
#endif

u8 reg10, reg30, reg31;
static void HdrEvSave(struct i2c_client *client)
{
	sensor_write(client, 0x00, 0x02);	// switch bank 2
	sensor_read(client, 0x10, &reg10);	// save AE setting
	sensor_read(client, 0x30, &reg30);	// save exp time
	sensor_read(client, 0x31, &reg31);	// save exp time
	printk("\n %s..%s reg30(%x) reg31(%x)\n",SENSOR_NAME_STRING(),__FUNCTION__,reg30,reg31);
}
static void HdrEvRestore(struct i2c_client *client, bool ae)
{
	sensor_write(client, 0x00, 0x02);	// switch bank 2
	if(ae){
		sensor_write(client, 0x10, reg10);	// restore AE setting
	}
	sensor_write(client, 0x30, reg30);	// save exp time
	sensor_write(client, 0x31, reg31);	// save exp time
	printk("\n %s..%s reg30(%x) reg31(%x)\n",SENSOR_NAME_STRING(),__FUNCTION__,reg30,reg31);
}
////////////////////////////
// 1 : Positive
// 0 : Normal
// -1 : Negative
////////////////////////////
static void HdrEvSet(struct i2c_client *client,int step)
{
	int cur_exp_time, new_exp_time, calcop;
	u8 cur_exp_time0, cur_exp_time1, cur_shut_step;
	if(step){
		sensor_write(client, 0x00, 0x02);	// switch bank 2
		sensor_write(client, 0x10, 0x18);	// disable AE

		sensor_read(client, 0x30, &cur_exp_time0);
		sensor_read(client, 0x31, &cur_exp_time1);
		sensor_read(client, 0x36, &cur_shut_step);
		cur_exp_time = cur_exp_time0;
		cur_exp_time <<= 8;
		cur_exp_time |= cur_exp_time1;
		calcop = cur_shut_step;
		calcop *= step;
		new_exp_time = cur_exp_time + calcop;
		
		if(new_exp_time < 10){
			new_exp_time = 10;
		}
		
		cur_exp_time1 = new_exp_time&0xFF;
		cur_exp_time0 = new_exp_time>>8;
		sensor_write(client, 0x30, cur_exp_time0);
		sensor_write(client, 0x31, cur_exp_time1);

		printk("\n %s..%s new_exp_time(%x)\n",SENSOR_NAME_STRING(),__FUNCTION__,new_exp_time);
	}
	else{
		HdrEvRestore(client, false);
	}
}

static int sensor_hdr_exposure(struct i2c_client *client, unsigned int code)
{
	printk("sensor_hdr_exposure_cb: %d\n",code);
	switch (code)
	{
		case RK_VIDEOBUF_HDR_EXPOSURE_MINUS_1:
		{
			HdrEvSave(client);
			HdrEvSet(client,-2);
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_NORMAL:
		{
			HdrEvSet(client,0);
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_PLUS_1:
		{
			HdrEvSet(client,2);
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_FINISH:
		{
			HdrEvRestore(client, true);
			break;
		}
		default:
			break;
	}
	
	return 0;
}

static int sensor_ioctrl(struct soc_camera_device *icd,enum rk29sensor_power_cmd cmd, int on)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int ret = 0;

    SENSOR_DG("%s %s  cmd(%d) on(%d)\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd,on);
	switch (cmd)
	{
		case Sensor_PowerDown:
		{
			if (modify_iso.modify_iso_flag == 1)
			{
				modify_iso.modify_iso_flag = 0;
				cancel_delayed_work(&(modify_iso.modify_iso_work));
			}
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
		}
		case Sensor_Flash:
		{
			struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    		struct sensor *sensor = to_sensor(client);

			if (sensor->sensor_io_request && sensor->sensor_io_request->sensor_ioctrl) {
				sensor->sensor_io_request->sensor_ioctrl(icd->pdev,Cam_Flash, on);
                if(on){
                    //flash off after 2 secs
            		hrtimer_cancel(&(flash_off_timer.timer));
            		hrtimer_start(&(flash_off_timer.timer),ktime_set(0, 800*1000*1000),HRTIMER_MODE_REL);
                    }
			}
            break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_power_end:
	return ret;
}

static enum hrtimer_restart flash_off_func(struct hrtimer *timer){
	struct flash_timer *fps_timer = container_of(timer, struct flash_timer, timer);
    sensor_ioctrl(fps_timer->icd,Sensor_Flash,0);
	SENSOR_DG("%s %s !!!!!!",SENSOR_NAME_STRING(),__FUNCTION__);
    return 0;
    
}


static int sensor_awb_stable(struct i2c_client *client, int timeout)
{
	s8 last_RGain, last_BGain;
	s8 RGain, BGain;

	sensor_write(client, 0x00, 0x03);	// switch bank 2

	sensor_read(client, 0x50, &last_RGain);
	sensor_read(client, 0x51, &last_BGain);

	while(timeout){
		if(in_atomic())
			mdelay(10);
		else
			msleep(10);

		sensor_read(client, 0x50, &RGain);
		sensor_read(client, 0x51, &BGain);

		if(abs(RGain - last_RGain) < 10 && abs(BGain - last_BGain) < 10){
			break;
		}
		else{
			last_RGain = RGain;
			last_BGain = BGain;
		}
		timeout--;
	}
	return 0;
}

#if CONFIG_SENSOR_ISO
static void iso_modify_wq(struct work_struct* workp);
#endif


static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
    const struct sensor_datafmt *fmt;
    int ret;

    SENSOR_DG("\n%s..%s.. \n",SENSOR_NAME_STRING(),__FUNCTION__);

     if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
          ret = -ENODEV;
          goto sensor_INIT_ERR;
     }
     msleep(100);

	sensor->info_priv.outputSize = OUTPUT_UXGA | OUTPUT_720P;
	sensor->info_priv.supportedSizeNum = 2;
	sensor->info_priv.supportedSize[0] = OUTPUT_720P;
	sensor->info_priv.supportedSize[1] = OUTPUT_UXGA;

	modify_iso.modify_iso_flag = 0;
#if 0
    /* soft reset */
	if (sensor_task_lock(client,1)<0)
		goto sensor_INIT_ERR;
    ret = sensor_write(client, 0x3012, 0x80);
    if (ret != 0)
    {
        SENSOR_TR("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
        ret = -ENODEV;
		goto sensor_INIT_ERR;
    }

    mdelay(5);  //delay 5 microseconds
	/* check if it is an sensor sensor */
    ret = sensor_read(client, 0x300a, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    pid |= (value << 8);

    ret = sensor_read(client, 0x300b, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    pid |= (value & 0xff);
    SENSOR_DG("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }
#endif
#if 0
   sensor_read(client,0x01,&value);
    pid = (value & 0xff);
    SENSOR_DG("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }
#endif
    ret = sensor_write_array(client, sensor_init_data);
    if (ret != 0)
    {
        SENSOR_TR("error: %s initial failed\n",SENSOR_NAME_STRING());
        goto sensor_INIT_ERR;
    }
	sensor_awb_stable(client,50);
    sensor_task_lock(client,0);
    sensor->info_priv.winseqe_cur_addr  = (int)SENSOR_INIT_WINSEQADR;
    fmt = sensor_find_datafmt(SENSOR_INIT_PIXFMT,sensor_colour_fmts, ARRAY_SIZE(sensor_colour_fmts));
    if (!fmt) {
        SENSOR_TR("error: %s initial array colour fmts is not support!!",SENSOR_NAME_STRING());
        ret = -EINVAL;
        goto sensor_INIT_ERR;
    }
	sensor->info_priv.fmt = *fmt;

    /* sensor sensor information for initialization  */
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
	if (qctrl)
    	sensor->info_priv.whiteBalance = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_BRIGHTNESS);
	if (qctrl)
    	sensor->info_priv.brightness = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
	if (qctrl)
    	sensor->info_priv.effect = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE);
	if (qctrl)
        sensor->info_priv.exposure = qctrl->default_value;
#if CONFIG_SENSOR_AntiBanding
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ANTIBANDING);
	if (qctrl)
        sensor->info_priv.antibanding = qctrl->default_value;
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_WHITEBALANCE_LOCK);
	if (qctrl)
        sensor->info_priv.WhiteBalanceLock = qctrl->default_value;
#endif
#if CONFIG_SENSOR_ExposureLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE_LOCK);
	if (qctrl)
        sensor->info_priv.ExposureLock = qctrl->default_value;
#endif
#if CONFIG_SENSOR_ISO
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ISO);
	if (qctrl){
		sensor->info_priv.iso = qctrl->default_value;
		modify_iso.client = client;
		INIT_DELAYED_WORK(&(modify_iso.modify_iso_work), iso_modify_wq);
		printk("%s..%s(%d) : modify_iso.modify_iso_work %x, modify_iso_flag %x\n",SENSOR_NAME_STRING(),__FUNCTION__, __LINE__, modify_iso.modify_iso_work);
		if ((sensor->info_priv.iso == 0) && (modify_iso.modify_iso_flag == 0)){
			modify_iso.modify_iso_flag = 1;
			schedule_delayed_work(&(modify_iso.modify_iso_work),500);
		}
	}
#endif
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SATURATION);
	if (qctrl)
        sensor->info_priv.saturation = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_CONTRAST);
	if (qctrl)
        sensor->info_priv.contrast = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_HFLIP);
	if (qctrl)
        sensor->info_priv.mirror = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_VFLIP);
	if (qctrl)
        sensor->info_priv.flip = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);
	if (qctrl)
        sensor->info_priv.scene = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.digitalzoom = qctrl->default_value;

    /* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
	#if CONFIG_SENSOR_Focus
    sensor_set_focus();
    qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.focus = qctrl->default_value;
	#endif

	#if CONFIG_SENSOR_Flash
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FLASH);
	if (qctrl)
        sensor->info_priv.flash = qctrl->default_value;
    flash_off_timer.icd = icd;
	flash_off_timer.timer.function = flash_off_func;
    #endif

    SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),((val == 0)?__FUNCTION__:"sensor_reinit"),icd->user_width,icd->user_height);
    sensor->info_priv.funmodule_state |= SENSOR_INIT_IS_OK;
    return 0;
sensor_INIT_ERR:
    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	sensor_task_lock(client,0);
	sensor_deactivate(client);
    return ret;
}

static int sensor_deactivate(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
    
	SENSOR_DG("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

	/* ddl@rock-chips.com : all sensor output pin must change to input for other sensor */
	
	sensor_ioctrl(icd, Sensor_PowerDown, 1);

	/* ddl@rock-chips.com : sensor config init width , because next open sensor quickly(soc_camera_open -> Try to configure with default parameters) */
	icd->user_width = SENSOR_INIT_WIDTH;
    icd->user_height = SENSOR_INIT_HEIGHT;
	msleep(100);

    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	return 0;
}

static  struct reginfo sensor_power_down_sequence[]=
{
    {0xff,0xff}
};
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
    int ret;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if (pm_msg.event == PM_EVENT_SUSPEND) {
        SENSOR_DG("\n %s Enter Suspend.. \n", SENSOR_NAME_STRING());
        ret = sensor_write_array(client, sensor_power_down_sequence) ;
        if (ret != 0) {
            SENSOR_TR("\n %s..%s WriteReg Fail.. \n", SENSOR_NAME_STRING(),__FUNCTION__);
            return ret;
        } else {
            ret = sensor_ioctrl(icd, Sensor_PowerDown, 1);
            if (ret < 0) {
			    SENSOR_TR("\n %s suspend fail for turn on power!\n", SENSOR_NAME_STRING());
                return -EINVAL;
            }
        }
    } else {
        SENSOR_TR("\n %s cann't suppout Suspend..\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }
    return 0;
}

static int sensor_resume(struct soc_camera_device *icd)
{
	int ret;

    ret = sensor_ioctrl(icd, Sensor_PowerDown, 0);
    if (ret < 0) {
		SENSOR_TR("\n %s resume fail for turn on power!\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

	SENSOR_DG("\n %s Enter Resume.. \n", SENSOR_NAME_STRING());

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
static bool sensor_fmt_capturechk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1024) && (mf->height == 768)) {
		ret = true;
	} else if ((mf->width == 1280) && (mf->height == 1024)) {
		ret = true;
	} else if ((mf->width == 1600) && (mf->height == 1200)) {
		ret = true;
	} else if ((mf->width == 2048) && (mf->height == 1536)) {
		ret = true;
	} else if ((mf->width == 2592) && (mf->height == 1944)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is capture format\n", __FUNCTION__, mf->width, mf->height);
	return ret;
}

static bool sensor_fmt_videochk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1280) && (mf->height == 720)) {
		ret = true;
	} else if ((mf->width == 1920) && (mf->height == 1080)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is video format\n", __FUNCTION__, mf->width, mf->height);
	return ret;
}
static int sensor_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
	const struct v4l2_queryctrl *qctrl;
	struct soc_camera_device *icd = client->dev.platform_data;
    struct reginfo *winseqe_set_addr=NULL;
    int ret=0, set_w,set_h;

	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (!fmt) {
        ret = -EINVAL;
        goto sensor_s_fmt_end;
    }

	if (sensor->info_priv.fmt.code != mf->code) {
		switch (mf->code)
		{
			case V4L2_MBUS_FMT_YUYV8_2X8:
			{
				winseqe_set_addr = sensor_ClrFmt_YUYV;
				break;
			}
			case V4L2_MBUS_FMT_UYVY8_2X8:
			{
				winseqe_set_addr = sensor_ClrFmt_UYVY;
				break;
			}
			default:
				break;
		}
		if (winseqe_set_addr != NULL) {
            sensor_write_array(client, winseqe_set_addr);
			sensor->info_priv.fmt.code = mf->code;
            sensor->info_priv.fmt.colorspace= mf->colorspace;            
			SENSOR_DG("%s v4l2_mbus_code:%d set success!\n", SENSOR_NAME_STRING(),mf->code);
		} else {
			SENSOR_TR("%s v4l2_mbus_code:%d is invalidate!\n", SENSOR_NAME_STRING(),mf->code);
		}
	}

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w == 1280) && (set_h == 720)) &&( sensor_hd[0].reg!=0xff))
    {
        winseqe_set_addr = sensor_hd;
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=0xff))
    {
        winseqe_set_addr = sensor_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        winseqe_set_addr = SENSOR_INIT_WINSEQADR;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;		
		SENSOR_TR("\n %s..%s Format is Invalidate. pix->width = %d.. pix->height = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
    }
	
#if 0
	if (((set_w <= 800) && (set_h <= 600)) && (sensor_svga[0].reg!=0xff))
    {
        winseqe_set_addr = sensor_svga;
        set_w = 800;
        set_h = 600;
    }
	else if (((set_w <= 1280) && (set_h <= 720)) &&( sensor_hd[0].reg!=0xff))
    {
        winseqe_set_addr = sensor_hd;
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=0xff))
    {
        winseqe_set_addr = sensor_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        winseqe_set_addr = SENSOR_INIT_WINSEQADR;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;		
		SENSOR_TR("\n %s..%s Format is Invalidate. pix->width = %d.. pix->height = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
    }
#endif

	SENSOR_TR("%s set size %dx%d!\n", SENSOR_NAME_STRING(), set_w, set_h);

    if ((int)winseqe_set_addr  != sensor->info_priv.winseqe_cur_addr) {
		SENSOR_TR("%s set new config!\n", SENSOR_NAME_STRING());
        #if CONFIG_SENSOR_Flash
        if (sensor_fmt_capturechk(sd,mf) == true) {      /* ddl@rock-chips.com : Capture */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_On);
                SENSOR_DG("%s flash on in capture!\n", SENSOR_NAME_STRING());
            }           
        } else {                                        /* ddl@rock-chips.com : Video */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                SENSOR_DG("%s flash off in preivew!\n", SENSOR_NAME_STRING());
            }
        }
        #endif
        ret |= sensor_write_array(client, winseqe_set_addr);
        if (ret != 0) {
            SENSOR_TR("%s set format capability failed\n", SENSOR_NAME_STRING());
            #if CONFIG_SENSOR_Flash
            if (sensor_fmt_capturechk(sd,mf) == true) {
                if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                    sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                    SENSOR_TR("%s Capture format set fail, flash off !\n", SENSOR_NAME_STRING());
                }
            }
            #endif
            goto sensor_s_fmt_end;
        }

        sensor->info_priv.winseqe_cur_addr  = (int)winseqe_set_addr;

		if (sensor_fmt_capturechk(sd,mf) == true) {				    /* ddl@rock-chips.com : Capture */
        #if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			if (sensor->info_priv.whiteBalance != 0) {
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
				sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			}
        #endif
			sensor->info_priv.snap2preview = true;
		} else if (sensor_fmt_videochk(sd,mf) == true) {			/* ddl@rock-chips.com : Video */
		#if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
        #endif
			sensor->info_priv.video2preview = true;
		} else if ((sensor->info_priv.snap2preview == true) || (sensor->info_priv.video2preview == true)) {
		#if CONFIG_SENSOR_Effect
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
        #endif
        #if CONFIG_SENSOR_WhiteBalance
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
        #endif
			sensor->info_priv.video2preview = false;
			sensor->info_priv.snap2preview = false;
		}
        SENSOR_TR("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),__FUNCTION__,set_w,set_h);
    }
    else
    {
        SENSOR_DG("\n %s .. Current Format is validate. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),set_w,set_h);
    }

	mf->width = set_w;
    mf->height = set_h;

sensor_s_fmt_end:
	mdelay(270);
    return ret;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
    int ret = 0,set_w,set_h;
   
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

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w == 1280) && (set_h == 720)) &&( sensor_hd[0].reg!=0xff))
    {
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=0xff))
    {
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;		
    }
	
#if 0
	if (((set_w <= 800) && (set_h <= 600)) && (sensor_svga[0].reg!=0xff))
    {
        set_w = 800;
        set_h = 600;
    }
	else if (((set_w <= 1280) && (set_h <= 720)) &&( sensor_hd[0].reg!=0xff))
    {
        set_w = 1280;
        set_h = 720;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && (sensor_uxga[0].reg!=0xff))
    {
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;		
    }
#endif
    mf->width = set_w;
    mf->height = set_h;  
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
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_EffectSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_EffectSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ExposureSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Saturation
static int sensor_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SaturationSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SaturationSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Contrast
static int sensor_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ContrastSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ContrastSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
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
            if (sensor_write_array(client, sensor_MirrorSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
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
            if (sensor_write_array(client, sensor_FlipSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
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
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_AntiBanding
static int sensor_set_antibanding(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
		struct sensor *sensor = to_sensor(client);
		struct reginfo **sensor_antibanding = NULL;

		if( sensor->info_priv.winseqe_cur_addr == (unsigned int)sensor_uxga ){
			sensor_antibanding = sensor_antibanding_uxga;
		}else if( sensor->info_priv.winseqe_cur_addr == (unsigned int)sensor_hd ){
			sensor_antibanding = sensor_antibanding_hd;
		}else if( sensor->info_priv.winseqe_cur_addr == (unsigned int)sensor_svga ){
			sensor_antibanding = sensor_antibanding_svga;
		}else{
			SENSOR_TR("%s..%s Check winseqe_cur_addr Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
			return -EINVAL;
		}

        if (sensor_antibanding[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_antibanding[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_WhiteBalanceLock
static int sensor_whitebalance_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_WhiteBalanceLk[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_WhiteBalanceLk[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_ExposureLock
static int sensor_set_exposure_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureLk[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ExposureLk[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

#if CONFIG_SENSOR_JPEG_EXIF
static int sensor_get_exposure_time(struct i2c_client *client)
{
    struct sensor *sensor = to_sensor(client);
	u8 Stst,Shut_StepH,Shut_StepL;
	int	ExposureTime;

	sensor_write(client, 0x00, 2);
	sensor_read(client, 0x36, &Stst);

	sensor_read(client, 0x30, &Shut_StepH);
	sensor_read(client, 0x31, &Shut_StepL);
	ExposureTime = Shut_StepH;
	ExposureTime <<= 8;
	ExposureTime |= Shut_StepL;
	ExposureTime /= Stst;

	SENSOR_TR("%s..%s ExposureTime(%d) \n",SENSOR_NAME_STRING(), __FUNCTION__, ExposureTime);

	if(ExposureTime){
		if(sensor->info_priv.antibanding){	//60Hz
			ExposureTime = 120/ExposureTime;
		}else{								//50Hz
			ExposureTime = 100/ExposureTime;
		}
	}else{
		ExposureTime = 100;
	}

	SENSOR_TR("%s..%s ExposureTime(%d) \n",SENSOR_NAME_STRING(), __FUNCTION__, ExposureTime);
    return ExposureTime;
}

static int sensor_set_exposure_time(struct i2c_client *client , int EvDenom)
{
	struct sensor *sensor = to_sensor(client);
	u8 Stst,Shut_StepH,Shut_StepL;
	sensor_write(client, 0x00, 2);
	sensor_read(client, 0x36, &Stst);

	if(sensor->info_priv.antibanding){	//60Hz
		EvDenom = 120*Stst/EvDenom;
	}else{								//50Hz
		EvDenom = 100*Stst/EvDenom;
	}

	Shut_StepL = (u8)(EvDenom & 0xFF);
	Shut_StepH = (u8)((EvDenom>>8) & 0xFF);
	
	sensor_write(client, 0x30, Shut_StepH);
	sensor_write(client, 0x31, Shut_StepL);
	
	return 0;
}

#endif
#if CONFIG_SENSOR_ISO
static struct reginfo sensor_iso_mod_gain4[] = 
{	
	{0x00, 0x06},
	{0x16, 0xff},
	{0x18, 0x20},
	{0x19, 0x38},
	{0xff, 0xff}
};

static struct reginfo sensor_iso_mod_gain2[] = 
{	
	{0x00, 0x06},
	{0x16, 0x88},
	{0x18, 0x10},
	{0x19, 0x18},
	{0xff, 0xff}
};

static struct reginfo sensor_iso_mod_gain0[] = 
{	
	{0x00, 0x06},
	{0x16, 0x00},
	{0x18, 0x10},
	{0x19, 0x18},
	{0xff, 0xff}
};

static void iso_modify_wq(struct work_struct* workp)
{
	struct i2c_client *client = modify_iso.client;
	u8 gain;

    SENSOR_TR("\n %s..%s Entry modify_iso.modify_iso_flag = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,modify_iso.modify_iso_flag);

	if (modify_iso.modify_iso_flag == 0) 
		return;
	else
		schedule_delayed_work(&(modify_iso.modify_iso_work),500);

	sensor_write(client, 0x00, 4);
	sensor_read(client, 0x1e, &gain);

	if(gain >= 0x20){
		/* if gain>=4
		{0x00, 0x06}, {0x16, 0x40}, {0x18, 0x20}, {0x19, 0x38}
		*/
		sensor_write_array(client, sensor_iso_mod_gain4);
	}
	else if(gain >= 0x10 && gain < 0x20){
		/* if 2<=gain <4
		{0x00, 0x06}, {0x16, 0x10}, {0x18, 0x10}, {0x19, 0x18}
		*/
		sensor_write_array(client, sensor_iso_mod_gain2);
	}
	else{
		/* else
		{0x00, 0x06}, {0x16, 0x00}, {0x18, 0x10}, {0x19, 0x18}
		*/
		sensor_write_array(client, sensor_iso_mod_gain0);
	}
	return;

}


static int sensor_set_iso(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct sensor *sensor = to_sensor(client);
	//SENSOR_TR("%s..%s : %x qctrl->minimum(%d) qctrl->maximum(%d)\n",SENSOR_NAME_STRING(),__FUNCTION__, value, qctrl->minimum, qctrl->maximum);
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ISO[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ISO[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);

			printk("%s..%s(%d) : iso value %x, modify_iso_flag %x\n",SENSOR_NAME_STRING(),__FUNCTION__, __LINE__, value, modify_iso.modify_iso_flag);
			if ((value == 0) && (modify_iso.modify_iso_flag == 0))
			{
				modify_iso.modify_iso_flag = 1;
				printk("%s..%s(%d) : modify_iso.modify_iso_work %x\n",SENSOR_NAME_STRING(),__FUNCTION__, __LINE__, modify_iso.modify_iso_work);
				schedule_delayed_work(&(modify_iso.modify_iso_work),500);
			}
			else
			{
				printk("%s..%s(%d) : modify_iso.modify_iso_work %x\n",SENSOR_NAME_STRING(),__FUNCTION__, __LINE__, modify_iso.modify_iso_work);
				cancel_delayed_work(&(modify_iso.modify_iso_work));
				modify_iso.modify_iso_flag = 0;
			}
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}

static int sensor_get_iso(struct i2c_client *client)
{
	int ret = 0;
	u8 gain = 0;
	sensor_write(client, 0x00, 4);
	sensor_read(client, 0x1E, &gain);
	if( (gain >= 10) && (gain < 20 )){	//1.25x ~ 2.5x
		ret = 100;
	}else if( (gain >= 20) && (gain < 32 )){	//2x ~ 4x
		ret = 200;
	}else if( (gain >= 32) && (gain < 64 )){	//4x ~ 8x
		ret = 200;
	}else if( gain >= 64 ){ 
		ret = 200;
	}else{
		ret = 0;
	}
	return ret;
}
#endif

#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_WhiteBalanceSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_WhiteBalanceSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_DigitalZoom
static int sensor_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
    int digitalzoom_cur, digitalzoom_total;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl_info)
		return -EINVAL;

    digitalzoom_cur = sensor->info_priv.digitalzoom;
    digitalzoom_total = qctrl_info->maximum;

    if ((value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        SENSOR_TR("%s digitalzoom is maximum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if  ((value < 0) && (digitalzoom_cur <= qctrl_info->minimum))
    {
        SENSOR_TR("%s digitalzoom is minimum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if ((value > 0) && ((digitalzoom_cur + value) > digitalzoom_total))
    {
        value = digitalzoom_total - digitalzoom_cur;
    }

    if ((value < 0) && ((digitalzoom_cur + value) < 0))
    {
        value = 0 - digitalzoom_cur;
    }

    digitalzoom_cur += value;

    if (sensor_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (sensor_write_array(client, sensor_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
            return -EINVAL;
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }

    return -EINVAL;
}
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
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }
    
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct v4l2_queryctrl *qctrl;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
        case V4L2_CID_BRIGHTNESS:
            {
                ctrl->value = sensor->info_priv.brightness;
                break;
            }
        case V4L2_CID_SATURATION:
            {
                ctrl->value = sensor->info_priv.saturation;
                break;
            }
        case V4L2_CID_CONTRAST:
            {
                ctrl->value = sensor->info_priv.contrast;
                break;
            }
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                ctrl->value = sensor->info_priv.whiteBalance;
                break;
            }
        case V4L2_CID_EXPOSURE:
            {
                ctrl->value = sensor->info_priv.exposure;
                break;
            }
        case V4L2_CID_HFLIP:
            {
                ctrl->value = sensor->info_priv.mirror;
                break;
            }
        case V4L2_CID_VFLIP:
            {
                ctrl->value = sensor->info_priv.flip;
                break;
            }
        default :
                break;
    }
    return 0;
}



static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;
    const struct v4l2_queryctrl *qctrl;


    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
#if CONFIG_SENSOR_Brightness
        case V4L2_CID_BRIGHTNESS:
            {
                if (ctrl->value != sensor->info_priv.brightness)
                {
                    if (sensor_set_brightness(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.brightness = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Exposure
        case V4L2_CID_EXPOSURE:
            {
                if (ctrl->value != sensor->info_priv.exposure)
                {
                    if (sensor_set_exposure(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.exposure = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Saturation
        case V4L2_CID_SATURATION:
            {
                if (ctrl->value != sensor->info_priv.saturation)
                {
                    if (sensor_set_saturation(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.saturation = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Contrast
        case V4L2_CID_CONTRAST:
            {
                if (ctrl->value != sensor->info_priv.contrast)
                {
                    if (sensor_set_contrast(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.contrast = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalance
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                if (ctrl->value != sensor->info_priv.whiteBalance)
                {
                    if (sensor_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.whiteBalance = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Mirror
        case V4L2_CID_HFLIP:
            {
                if (ctrl->value != sensor->info_priv.mirror)
                {
                    if (sensor_set_mirror(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.mirror = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flip
        case V4L2_CID_VFLIP:
            {
                if (ctrl->value != sensor->info_priv.flip)
                {
                    if (sensor_set_flip(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.flip = ctrl->value;
                }
                break;
            }
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

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                ext_ctrl->value = sensor->info_priv.scene;
                break;
            }
#if CONFIG_SENSOR_AntiBanding
        case V4L2_CID_ANTIBANDING:
            {
                ext_ctrl->value = sensor->info_priv.antibanding;
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
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
#if CONFIG_SENSOR_ISO
		case V4L2_CID_ISO:
			{
				ext_ctrl->value = sensor->info_priv.iso;
				if(ext_ctrl->value == 0){
					ext_ctrl->value = sensor_get_iso(client);
				}
				break;
			}
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
        case V4L2_CID_EFFECT:
            {
                ext_ctrl->value = sensor->info_priv.effect;
                break;
            }
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.digitalzoom;
                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.focus;
                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FLASH:
            {
                ext_ctrl->value = sensor->info_priv.flash;
                break;
            }
        default :
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
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
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
#if CONFIG_SENSOR_WhiteBalanceLock
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
#if CONFIG_SENSOR_ISO
		case V4L2_CID_ISO:
			{
				if (ext_ctrl->value != sensor->info_priv.iso ) {
					if (sensor_set_iso(icd, qctrl, ext_ctrl->value) != 0) {
						return -EINVAL;
					}
					sensor->info_priv.iso = ext_ctrl->value;
				}
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
						sensor_write_array(client, sensor_ExposureLk[1]);
					}
					
					if(pExitInfo->ExposureTime.denom != sensor_get_exposure_time(client)){
						sensor_set_exposure_time(client, pExitInfo->ExposureTime.denom);
					}

					if(pExitInfo->ExposureTime.num == 1){
						// Enable AE
						sensor_write_array(client, sensor_ExposureLk[0]);
					}
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

                    SENSOR_DG("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->info_priv.digitalzoom);
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

                    SENSOR_DG("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.digitalzoom);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Focus
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.focus)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.focus;

                    sensor->info_priv.focus += val_offset;
                }

                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    sensor->info_priv.focus += ext_ctrl->value;

                    SENSOR_DG("%s focus is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.focus);
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

                SENSOR_DG("%s flash is %x\n",SENSOR_NAME_STRING(), sensor->info_priv.flash);
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

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
    int ret;
    u8 chipid = 0;

    /* We must have a parent by now. And it cannot be a wrong one.
     * So this entire test is completely redundant. */
    if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}

	ret = sensor_write(client, 0x00, 0);
	if (ret < 0) {
		SENSOR_TR("read chip id set bank 0 failed \n");
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}
	
	ret = sensor_read(client, 0x01, &chipid);
	if (ret < 0) {
		SENSOR_TR("read chip id failed\n");
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}
	SENSOR_TR("\n %s  pid = 0x%x \n", SENSOR_NAME_STRING(), chipid);
#if 0
    /* soft reset */
    ret = sensor_write(client, 0x3012, 0x80);
    if (ret != 0)
    {
        SENSOR_TR("soft reset %s failed\n",SENSOR_NAME_STRING());
        return -ENODEV;
    }
    mdelay(5);          //delay 5 microseconds

    /* check if it is an sensor sensor */
    ret = sensor_read(client, 0x300a, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    
    ret = sensor_read(client, 0x01, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    pid = (value & 0xff);
    SENSOR_DG("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
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
    int ret = 0;
#if CONFIG_SENSOR_Flash	
    int i;
#endif
    
	SENSOR_DG("\n%s..%s..cmd:%x \n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
	switch (cmd)
	{
		case RK29_CAM_SUBDEV_HDR_EXPOSURE:
		{
			sensor_hdr_exposure(client,(unsigned int)arg);
			break;
		}

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
                    SENSOR_TR("%s %s obtain gpio resource failed when RK29_CAM_SUBDEV_IOREQUEST \n",SENSOR_NAME_STRING(),__FUNCTION__);
                    ret = -EINVAL;
                    goto sensor_ioctl_end;
                }
            } else {
                SENSOR_TR("%s %s RK29_CAM_SUBDEV_IOREQUEST fail\n",SENSOR_NAME_STRING(),__FUNCTION__);
                ret = -EINVAL;
                goto sensor_ioctl_end;
            }
            /* ddl@rock-chips.com : if gpio_flash havn't been set in board-xxx.c, sensor driver must notify is not support flash control 
               for this project */
            #if CONFIG_SENSOR_Flash	
        	if (sensor->sensor_gpio_res) { 
                if (sensor->sensor_gpio_res->gpio_flash == INVALID_GPIO) {
                    for (i = 0; i < icd->ops->num_controls; i++) {
                		if (V4L2_CID_FLASH == icd->ops->controls[i].id) {
                			//memset((char*)&icd->ops->controls[i],0x00,sizeof(struct v4l2_queryctrl));  
                              sensor_controls[i].id=0xffff;         			
                		}
                    }
                    sensor->info_priv.flash = 0xff;
                    SENSOR_DG("%s flash gpio is invalidate!\n",SENSOR_NAME_STRING());
                }else{ //two cameras are the same,need to deal diffrently ,zyc
                    for (i = 0; i < icd->ops->num_controls; i++) {
                           if(0xffff == icd->ops->controls[i].id){
                              sensor_controls[i].id=V4L2_CID_FLASH;
                           }               
                    }
                }
        	}
            #endif
			break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown !\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
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
		if(fsize->reserved[0] == 0x5afefe5a){
			fsize->discrete.width = 1280;
			fsize->discrete.height = 720;
			fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		}
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
	.g_ext_ctrls          = sensor_g_ext_controls,
	.s_ext_ctrls          = sensor_s_ext_controls,
	.g_chip_ident	= sensor_g_chip_ident,
	.ioctl = sensor_ioctl,
};

static struct v4l2_subdev_video_ops sensor_subdev_video_ops = {
	.s_mbus_fmt	= sensor_s_fmt,
	.g_mbus_fmt	= sensor_g_fmt,
	.try_mbus_fmt	= sensor_try_fmt,
	.enum_mbus_fmt	= sensor_enum_fmt,
	.enum_framesizes = sensor_enum_framesizes,
};

static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
	.video = &sensor_subdev_video_ops,
};

static int sensor_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
    struct sensor *sensor;
    struct soc_camera_device *icd = client->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct soc_camera_link *icl;
    int ret;

    SENSOR_DG("\n%s..%s..%d..\n",__FUNCTION__,__FILE__,__LINE__);
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
	#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_set(&sensor->tasklock_cnt,0);
	#endif

    ret = sensor_video_probe(icd, client);
    if (ret < 0) {
        icd->ops = NULL;
        i2c_set_clientdata(client, NULL);
        kfree(sensor);
		sensor = NULL;
    }
	hrtimer_init(&(flash_off_timer.timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    SENSOR_DG("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);
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
    SENSOR_DG("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());
    return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
    i2c_del_driver(&sensor_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION(SENSOR_NAME_STRING(Camera sensor driver));
MODULE_AUTHOR("ddl <kernel@rock-chips>");
MODULE_LICENSE("GPL");
