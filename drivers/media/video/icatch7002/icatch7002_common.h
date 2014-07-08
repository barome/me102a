#ifndef ICATCH7002_COMMON_H
#define ICATCH7002_COMMON_H
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <plat/rk_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <mach/iomux.h>
#include "app_i2c_lib_icatch.h"

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
#define MAX(x,y)	((x>y) ? x: y)

#define	CALIBRATION_MODE_FUN		1

/* Front Sensor Driver Configuration */
#define SENSOR_REAR_NAME			RK29_CAM_ISP_ICATCH7002_OV5693
#define SENSOR_REAR_V4L2_IDENT		V4L2_IDENT_ICATCH7002_OV5693
#define SENSOR_REAR_ID				0x5690
#define SENSOR_REAR_MIN_WIDTH		1280//176
#define SENSOR_REAR_MIN_HEIGHT		960//144
#define SENSOR_REAR_MAX_WIDTH		2592
#define SENSOR_REAR_MAX_HEIGHT		1944
#define SENSOR_REAR_INIT_WIDTH		1280		/* Sensor pixel size for sensor_init_data array */
#define SENSOR_REAR_INIT_HEIGHT		960
#define SENSOR_REAR_INIT_PIXFMT		V4L2_MBUS_FMT_YUYV8_2X8

#define SENSOR_REAR_NAME_STRING(a)	STR(CONS(SENSOR_REAR_NAME, a))

/* Rear Sensor Driver Configuration */
#define SENSOR_FRONT_NAME			RK29_CAM_ISP_ICATCH7002_MI1040
#define SENSOR_FRONT_V4L2_IDENT		V4L2_IDENT_ICATCH7002_MI1040
#define SENSOR_FRONT_ID				0x2481
#define SENSOR_FRONT_MIN_WIDTH		1280//176
#define SENSOR_FRONT_MIN_HEIGHT		960//144
#define SENSOR_FRONT_MAX_WIDTH		1280
#define SENSOR_FRONT_MAX_HEIGHT		960
#define SENSOR_FRONT_INIT_WIDTH		1280		/* Sensor pixel size for sensor_init_data array */
#define SENSOR_FRONT_INIT_HEIGHT	960
#define SENSOR_FRONT_INIT_PIXFMT	V4L2_MBUS_FMT_YUYV8_2X8

#define SENSOR_FRONT_NAME_STRING(a)	STR(CONS(SENSOR_FRONT_NAME, a))

#define SENSOR_NAME_STRING(a)		"icatch7002_common"

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast		0
#define CONFIG_SENSOR_Saturation	0
#define CONFIG_SENSOR_Effect		1
#define CONFIG_SENSOR_Scene 		1
#define CONFIG_SENSOR_DigitalZoom	0
#define CONFIG_SENSOR_Focus 		1
#define CONFIG_SENSOR_Exposure		1
#define CONFIG_SENSOR_Flash 		0
#define CONFIG_SENSOR_Mirror		0
#define CONFIG_SENSOR_Flip		0
#define CONFIG_SENSOR_FOCUS_ZONE	0
#define CONFIG_SENSOR_FACE_DETECT	0
#define CONFIG_SENSOR_ISO		1
#define CONFIG_SENSOR_AntiBanding	1
#define CONFIG_SENSOR_WhiteBalanceLock	1
#define CONFIG_SENSOR_ExposureLock	1
#define CONFIG_SENSOR_MeteringAreas	1
#define CONFIG_SENSOR_Wdr			1
#define CONFIG_SENSOR_EDGE			1
#define CONFIG_SENSOR_JPEG_EXIF		1

#if CONFIG_SENSOR_Focus
#define SENSOR_AF_AUTO	0
#define SENSOR_AF_MACRO 1
#define SENSOR_AF_INFINITY 2
#define SENSOR_AF_CONTINUOUS 3
#define SENSOR_AF_FULL_SEARCH 4
static int sensor_set_auto_focus(struct i2c_client *client, int value);
#endif

enum sensor_work_state {
	sensor_work_ready = 0,
	sensor_working,
};

struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

#ifndef	__ICATCH7002_COMMON_CFILE__
extern struct spi_device* g_icatch_spi_dev;
extern struct i2c_client *g_icatch_i2c_client;
extern const struct sensor_datafmt icatch_colour_fmts[1];
extern const struct soc_camera_ops sensor_ops;
extern const struct v4l2_subdev_ops sensor_subdev_ops;
#endif
int icatch_sensor_write( u16 reg, u8 val);
u8 icatch_sensor_read( u16 reg);
int icatch_sensor_write_array(void *regarrayv);
#if CALIBRATION_MODE_FUN
void icatch_create_proc_entry();
void icatch_remove_proc_entry();
#endif
extern void BB_WrSPIFlash(u32 size);
extern int icatch_request_firmware(const struct firmware ** fw);
extern void icatch_release_firmware(const struct firmware * fw);
extern void icatch_sensor_power_ctr(struct soc_camera_device *icd ,int on,int power_mode);
extern struct isp_dev* to_sensor(const struct i2c_client *client);
extern int icatch_load_fw(struct soc_camera_device *icd,u8 sensorid);
const struct sensor_datafmt *sensor_find_datafmt(
	 enum v4l2_mbus_pixelcode code, const struct sensor_datafmt *fmt,
	 int n);
int icatch_get_rearid_by_lowlevelmode(struct soc_camera_device *icd,UINT16 *rear_id);
int icatch_get_frontid_by_lowlevelmode(struct soc_camera_device *icd,UINT16 *front_id);
int icatch_update_sensor_ops(
	const struct v4l2_queryctrl *controls,
	int num_controls,
	const struct v4l2_querymenu *menus,
	int num_menus);

struct reginfo
{
	u16 reg;
	u8 val;
};

enum ISP_OUTPUT_RES{
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


enum sensor_wq_cmd
{
	WqCmd_af_init,
	WqCmd_af_single,
	WqCmd_af_special_pos,
	WqCmd_af_far_pos,
	WqCmd_af_near_pos,
	WqCmd_af_continues,
	WqCmd_af_update_zone
};
enum sensor_wq_result
{
	WqRet_success = 0,
	WqRet_fail = -1,
	WqRet_inval = -2
};
struct sensor_work
{
	struct i2c_client *client;
	struct delayed_work dwork;
	enum sensor_wq_cmd cmd;
	wait_queue_head_t done;
	enum sensor_wq_result result;
	bool wait;
	int var;	
};

enum sensor_preview_cap_mode{
	PREVIEW_MODE,
	CAPTURE_MODE,
	CAPTURE_ZSL_MODE,
	CAPTURE_NONE_ZSL_MODE,
	IDLE_MODE,
};

struct focus_zone_s{
int lx;
int ty;
int rx;
int dy;
};

//flash and focus must be considered.

//soft isp or external isp used
//if soft isp is defined , the value in this sturct is used in cif driver . cif driver use this value to do isp func.
//value of this sturct MUST be defined(initialized) correctly. 
struct isp_data{
	int focus;
	int auto_focus;
	int flash;
	int whiteBalance;
	int brightness;
	int contrast;
	int saturation;
	int effect;
	int scene;
	int digitalzoom;
	int exposure;
	int iso;
	int face;
	int antibanding;
	int WhiteBalanceLock;
	int ExposureLock;
	int MeteringAreas;
	int Wdr;
	//mirror or flip
	unsigned char mirror;										 /* HFLIP */
	unsigned char flip; 										 /* VFLIP */
	//preview or capture
	int outputSize; // supported resolution
	int curRes;
	int curPreviewCapMode;
	int supportedSize[10];
	int supportedSizeNum;
	int had_setprvsize;

#if CALIBRATION_MODE_FUN
	int rk_query_PreviewCapMode;
	int sensor_id;
#endif
	struct focus_zone_s focus_zone;
	struct sensor_datafmt fmt;
	//mutex for access the isp data
	struct mutex access_data_lock;
};



struct isp_dev{
	struct v4l2_subdev subdev;
	struct i2c_client *client;
	struct workqueue_struct *sensor_wq;
	struct mutex wq_lock;
	int model;	/* V4L2_IDENT_OV* codes from v4l2-chip-ident.h */
	struct isp_data isp_priv_info;
	int (*before_init_cb)(const struct i2c_client *client);
	};

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING|\
						  SOCAM_HSYNC_ACTIVE_HIGH| SOCAM_VSYNC_ACTIVE_HIGH|\
						  SOCAM_DATA_ACTIVE_HIGH|SOCAM_DATAWIDTH_8	|SOCAM_MCLK_24MHZ)

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

#endif

