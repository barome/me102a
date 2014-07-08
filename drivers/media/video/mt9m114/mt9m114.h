/*
 * Driver for MT9P111 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MT9M114_H__
#define __MT9M114_H__

#define	SEQ_BYTE	1	// BYTE
#define	SEQ_WORD	2	// WORD
#define	SEQ_DWORD	4	// DWORD

#define	SEQ_REG			1	//coding only. will be converted to SEQ_BIT_MASK
							//eg. set reg 0x098E to 0x0000, and delay 10ms
							// {SEQ_REG, 0x098E, SEQ_WORD, 0x0000, 10}

#define	SEQ_BIT_SET		2	//set bits of address to bits of value
							//eg. set reg 0x098E bit 0,1 to 1
							// {SEQ_BIT_SET, SEQ_WORD, 0x098E, 0x0003, 0}

#define	SEQ_BIT_CLR		3	//set bits of address to bits of value
							//eg. set reg 0x098E bit 0,1 to 0
							// {SEQ_BIT_CLR, SEQ_WORD, 0x098E, 0x0003, 0}

#define	SEQ_POLL_SET	4	//poll reg until the bit is set with timeout
							//eg. pool reg 0x098E bit 1 set with timeout 200ms
							// {SEQ_POOL_SET, 0x098E, SEQ_WORD, 0x0002, 200}
#define	SEQ_POLL_CLR	5	//poll reg until the bit is reset with timeout
							//eg. pool reg 0x098E bit 0 reset with timeout 200ms
							// {SEQ_POLL_CLR, 0x098E, SEQ_WORD, 0x0002, 200}

#define	SEQ_BURST		6	//burst setting regs
							//eg.	{SEQ_BURST_ST, burstlength, SEQ_WORD, data0, 0}
							//		{notcare, notcare, SEQ_WORD, data1, 0}
							//		{notcare, notcare, SEQ_WORD, data2, 0}
							//					..
							//		{notcare, notcare, SEQ_WORD, data(n-1), 0}
							//		{notcare, notcare, SEQ_WORD, data(n), 0}

#define	SEQ_US			7	//delay(us)
#define	SEQ_MS			8	//delay(ms)
#define	SEQ_END			0

struct seq_info{
	u8	ops;
	u16	reg;
	u8	len;
	u32	val;
	u32	tmo;	// time out if request
};


#if defined(CONFIG_TRACE_LOG_PRINTK)
 #define DEBUG_TRACE(format, ...) printk(KERN_WARNING format, ## __VA_ARGS__)
#else
 #define DEBUG_TRACE(format, ...)
#endif
#define LOG_TRACE(format, ...) printk(KERN_WARNING format, ## __VA_ARGS__)

#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

/* Sensor Driver Configuration */
#define SENSOR_NAME				RK29_CAM_SENSOR_MT9M114
#define SENSOR_V4L2_IDENT		V4L2_IDENT_MT9M114
#define	SENSOR_ID_NEED_PROBE	1
#define SENSOR_ID				0x2481
#define SENSOR_ID_REG			0x0
#define SENSOR_NEED_SOFTRESET	0
#define SENSOR_MIN_WIDTH		176
#define SENSOR_MIN_HEIGHT		144
#define SENSOR_MAX_WIDTH		1280
#define SENSOR_MAX_HEIGHT		960
#define SENSOR_INIT_WIDTH		1280			/* Sensor pixel size for init_seq_ops array */
#define SENSOR_INIT_HEIGHT		960
#define SENSOR_INIT_PIXFMT		V4L2_MBUS_FMT_UYVY8_2X8
#define YUV420_BUFFER_MAX_SIZE	7558272     /* 2592*1944*1.5*/

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast		1
#define CONFIG_SENSOR_Saturation	1
#define CONFIG_SENSOR_Effect		1
#define CONFIG_SENSOR_Scene			0
#define CONFIG_SENSOR_DigitalZoom	0
#define CONFIG_SENSOR_Exposure		1
#define CONFIG_SENSOR_Flash			0
#define CONFIG_SENSOR_Mirror		0
#define CONFIG_SENSOR_Flip			0
#define CONFIG_SENSOR_Focus			0
#define CONFIG_SENSOR_Sharpness		1
#define CONFIG_SENSOR_AntiBanding			1
#define CONFIG_SENSOR_WhiteBalanceLock		1
#define CONFIG_SENSOR_ExposureLock			1
#define CONFIG_SENSOR_ISO					1
#define CONFIG_SENSOR_JPEG_EXIF				1


#if CONFIG_SENSOR_AntiBanding
static int sensor_set_antibanding(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
static int sensor_whitebalance_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif
#if CONFIG_SENSOR_ExposureLock
static int sensor_set_exposure_lock(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
#endif

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

#define CONFIG_SENSOR_I2C_SPEED   350000       /* Hz */

/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING|\
                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_HIGH|\
                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_24MHZ)

#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SENSOR_AF_IS_ERR    (0x00<<0)
#define SENSOR_AF_IS_OK		(0x01<<0)
#define SENSOR_INIT_IS_ERR   (0x00<<28)
#define SENSOR_INIT_IS_OK    (0x01<<28)

/**optimize code to shoten open time******/
#define ADJUST_OPTIMIZE_TIME_FALG	0

enum sensor_work_state
{
	sensor_work_ready = 0,
	sensor_working,
};
struct sensor_work
{
	struct i2c_client *client;
	struct delayed_work dwork;
	enum sensor_work_state state;
};

/* only one fixed colorspace per pixelcode */
struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

typedef struct sensor_info_priv_s
{
#if CONFIG_SENSOR_WhiteBalance
	int whiteBalance;
#endif
#if CONFIG_SENSOR_Brightness
	int brightness;
#endif
#if CONFIG_SENSOR_Contrast
	int contrast;
#endif
#if CONFIG_SENSOR_Saturation
	int saturation;
#endif
#if CONFIG_SENSOR_Effect
	int effect;
#endif
#if CONFIG_SENSOR_Scene
	int scene;
#endif
	int antibanding;
	int WhiteBalanceLock;
	int ExposureLock;
#if CONFIG_SENSOR_DigitalZoom
	int digitalzoom;
#endif
#if CONFIG_SENSOR_Focus
	int focus;
	int auto_focus;
	int affm_reinit;
#endif
#if CONFIG_SENSOR_Flash
	int flash;
#endif
#if CONFIG_SENSOR_Exposure
	int exposure;
#endif
#if CONFIG_SENSOR_Sharpness
	int sharpness;
#endif
#if CONFIG_SENSOR_Mirror
	unsigned char mirror;                                        /* HFLIP */
#endif
#if CONFIG_SENSOR_Flip
	unsigned char flip;                                          /* VFLIP */
#endif
	int capture_w;
	int capture_h;
	int preview_w;
	int preview_h;
	struct sensor_datafmt fmt;
	unsigned int enable;
	unsigned int funmodule_state;
	int outputSize; // supported resolution
	int supportedSize[10];
	int supportedSizeNum;
} sensor_info_priv_t;

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

struct sensor_parameter
{
	unsigned short int preview_maxlines;
	unsigned short int preview_exposure;
	unsigned short int preview_line_width;
	unsigned short int preview_gain;

	unsigned short int capture_framerate;
	unsigned short int preview_framerate;
};

struct sensor
{
	struct v4l2_subdev subdev;
	struct i2c_client *client;
	sensor_info_priv_t info_priv;
	struct sensor_parameter parameter;
	struct workqueue_struct *sensor_wq;
	struct sensor_work sensor_wk;
	struct mutex wq_lock;
    int model;	/* V4L2_IDENT_OV* codes from v4l2-chip-ident.h */
	struct rk29camera_platform_data *sensor_io_request;
	struct rk29camera_gpio_res *sensor_gpio_res;
};

#endif	/* __MT9M114_H__ */
