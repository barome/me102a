#define __ICATCH7002_COMMON_CFILE__
#include "icatch7002_common.h"
#include <linux/proc_fs.h>

#define CONFIG_SENSOR_I2C_SPEED 	300000		 /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED	0
#define CONFIG_SENSOR_I2C_RDWRCHK	0

#define ICATCHFWNAME "icatch7002boot.bin"

u8 g_Calibration_Option_Def = 0xff;

#ifdef CALIBRATION_MODE_FUN
typedef struct icatch_cali_fw_data {
	const char * const fname_option;
	struct firmware *fw_option;
	const char * const fname_3acali;
	struct firmware *fw_3acali;
	const char * const fname_lsc;
	struct firmware *fw_lsc;
	const char * const fname_lscdq;
	struct firmware *fw_lscdq;
};

struct icatch_cali_fw_data g_cali_fw_data_front = {
	.fname_option = "icatch7002/calibration_option.BIN",
	.fname_3acali = "icatch7002/3ACALI_F.BIN",
	.fname_lsc = "icatch7002/LSC_F.BIN",
	.fname_lscdq = "icatch7002/LSC_DQ_F.BIN",
};

struct icatch_cali_fw_data g_cali_fw_data_back = {
	.fname_option = "icatch7002/calibration_option.BIN",
	.fname_3acali = "icatch7002/3ACALI.BIN",
	.fname_lsc = "icatch7002/LSC.BIN",
	.fname_lscdq = "icatch7002/LSC_DQ.BIN",
};
#endif

#define ICATCH_BOOT_FROM_SPI 0
#define ICATCH_BOOT_FROM_HOST 1
#define ICATCH_BOOT ICATCH_BOOT_FROM_HOST
#define ASUS_CAMERA_SUPPORT 1

#ifndef FAIL
#define FAIL	0
#endif
#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE	0
#endif

static struct reginfo rs_reset_data[]={
	{0x1011,0x01},
	{0x001c,0x08},
	{0x001c,0x00},
	{0x1010,0x02},
	{0x1010,0x00},
	{0x1306,0x00},//0 rear,1 front
	{0x1011,0x00},
	{0x00,0x00},
};

static struct reginfo fs_reset_data[]={
	{0x1011,0x01},
	{0x001c,0x08},
	{0x001c,0x00},
	{0x1010,0x02},
	{0x1010,0x00},
	{0x1306,0x01},//0 rear,1 front
	{0x1011,0x00},
	{0x00,0x00},
};

#if 0
static struct reginfo init_data[]={
	{SP7K_MSG_COLOR_EFFECT,		0x0}, // normal
	{SP7K_MSG_EV_COMPENSATION,	0x6}, // 0
	{SP7K_MSG_FLASH_MODE,		0x1}, // off
	{SP7K_MSG_FOCUS_MODE,		0x0}, // auto
	{SP7K_MSG_PV_SIZE,			0x0}, // 1280*960
	{SP7K_MSG_SCENE_MODE,		0x0}, // normal
	{SP7K_MSG_WHITE_BALANCE,	0x0}, // auto
	{SP7K_MSG_CAP_ISO,			0x0}, //auto
	{SP7K_MSG_AURA_COLOR_INDEX, 0x0}, // disable
	{SP7K_MSG_PV_CAP_MODE,		0x4}, // idle
	{0x00,0x00},
};
#endif

struct i2c_client *g_icatch_i2c_client = NULL;
const struct sensor_datafmt icatch_colour_fmts[1] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG} 
};

#if CONFIG_SENSOR_WhiteBalance
static int icatch_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	u8 set_val = 0;

	DEBUG_TRACE("%s: value = %d\n", __FUNCTION__, value);

	if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
	{
		switch(value){
			case 0: //enable auto
				set_val = 0x0;
				break;
			case 1: //incandescent		Tungsten
				set_val = 0x6;
				break;
			case 2: //fluorescent
				set_val = 0x5;
				break;
			case 3: //daylight
				set_val = 0x1;
				break;
			case 4: //cloudy-daylight
				set_val = 0x2;
				break;
			default:
				break;
		}
		//awb
		EXISP_I2C_WhiteBalanceSet(set_val);
		return 0;
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..	\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -1;
}
#endif

#if CONFIG_SENSOR_Effect
static int icatch_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value, int auravalue)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	u8 set_val = 0;
	DEBUG_TRACE("set effect,value = %d ......\n",value);
	if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
	{
		switch(value){
			case 0: //normal
			case 5: //none
				set_val = 0x00;
				break;
			case 1: //aqua
				set_val = 0x01;
				break;
			case 2: //negative
				set_val = 0x02;
				break;
			case 3: //sepia
				set_val = 0x03;
				break;
			case 4: //mono	Grayscale
				set_val = 0x04;
				break;
			case 6: //aura
				set_val = 0x06;
				break;
			case 7: //vintage
				set_val = 0x07;
				break;
			case 8: //vintage2
				set_val = 0x08;
				break;
			case 9: //lomo
				set_val = 0x09;
				break;
			case 10: //red
				set_val = 0x0A;
				break;
			case 11: //blue
				set_val = 0x0B;
				break;
			case 12: //green
				set_val = 0x0C;
				break;
			default:
				set_val = value;
				break;
		}
		EXISP_I2C_ColorEffectSet(set_val);
		if(set_val == 6){
			EXISP_I2C_AuraColorIndexSet(auravalue);
		}
		return 0;
	}
	DEBUG_TRACE("\n%s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -1;
}
#endif

#if CONFIG_SENSOR_Scene
static int icatch_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	u8 set_val = 0;
//when scene mod is working , face deteciton and awb and iso are not recomemnded.
	if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
	{
		switch(value){
			case 0: //normal
				set_val = 0x00;
				break;
			case 1: //auto
				set_val = 0x00;
				break;
			case 2: //landscape
				set_val = 0x10;
				break;
			case 3: //night
				set_val = 0x07;
				break;
			case 4: //night_portrait
				set_val = 0x08;
				break;
			case 5: //snow
				set_val = 0x0B;
				break;
			case 6: //sports
				set_val = 0x0C;
				break;
			case 7: //candlelight
				set_val = 0x04;
				break;

			default:
				break;
		}

		EXISP_I2C_SceneModeSet(set_val);
		DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
		return 0;
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..	\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Focus
static void sensor_af_workqueue(struct work_struct *work);

static void icatch_af_workqueue(struct work_struct *work)
{
	struct sensor_work *sensor_work = container_of(work, struct sensor_work, dwork.work);
	struct i2c_client *client = sensor_work->client;
	struct isp_dev *sensor = to_sensor(client);

	DEBUG_TRACE("%s %s Enter, cmd:0x%x \n",SENSOR_NAME_STRING(), __FUNCTION__,sensor_work->cmd);

	mutex_lock(&sensor->wq_lock);

//	  DEBUG_TRACE("%s:auto focus, val = %d\n",__func__,sensor_work->var);
	//auto focus
	if(sensor_set_auto_focus(client,sensor_work->var) == 0){
		sensor_work->result = WqRet_success;
	}else{
		DEBUG_TRACE("%s:auto focus failed\n",__func__);
	}
//	  DEBUG_TRACE("%s:auto focus done\n",__func__);

//set_end:
	if (sensor_work->wait == false) {
		kfree((void*)sensor_work);
	} else {
		wake_up(&sensor_work->done);
	}
	mutex_unlock(&sensor->wq_lock);
	return;
}

static int icatch_af_workqueue_set(struct soc_camera_device *icd, enum sensor_wq_cmd cmd, int var, bool wait)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct isp_dev *sensor = to_sensor(client);
	struct sensor_work *wk;
	int ret=0;

	if (sensor->sensor_wq == NULL) {
		ret = -EINVAL;
		goto sensor_af_workqueue_set_end;
	}

	wk = kzalloc(sizeof(struct sensor_work), GFP_KERNEL);
	if (wk) {
		wk->client = client;
		INIT_WORK(&(wk->dwork.work), sensor_af_workqueue);
		wk->cmd = cmd;
		wk->result = WqRet_inval;
		wk->wait = wait;
		wk->var = var;
		init_waitqueue_head(&wk->done);

		queue_delayed_work(sensor->sensor_wq,&(wk->dwork),0);

		/* ddl@rock-chips.com:
		* video_lock is been locked in v4l2_ioctl function, but auto focus may slow,
		* As a result any other ioctl calls will proceed very, very slowly since each call
		* will have to wait for the AF to finish. Camera preview is pause,because VIDIOC_QBUF
		* and VIDIOC_DQBUF is sched. so unlock video_lock here.
		*/
		if (wait == true) {
			mutex_unlock(&icd->video_lock);
			if (wait_event_timeout(wk->done, (wk->result != WqRet_inval), msecs_to_jiffies(5000)) == 0) {  //hhb
				DEBUG_TRACE("%s %s cmd(%d) is timeout!\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			}
			ret = wk->result;
			kfree((void*)wk);
			mutex_lock(&icd->video_lock);
		}

	} else {
		DEBUG_TRACE("%s %s cmd(%d) ingore,because struct sensor_work malloc failed!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
		ret = -1;
	}
sensor_af_workqueue_set_end:
	return ret;
}
#endif

#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	u8 set_val = 0x0;
	if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
	{
		set_val = 6 - value;
		EXISP_I2C_EvSet(set_val);
		DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
		return 0;
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..	\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -EINVAL;
}
#endif

#if CONFIG_SENSOR_Mirror
static int sensor_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	return 0;
}
#endif

#if CONFIG_SENSOR_Flip
//off 0x00;mirror 0x01,flip 0x10;
static int sensor_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	return 0;

}
#endif

#if CONFIG_SENSOR_ISO
static int sensor_set_iso(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	u8 set_val = 0x0;
	if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
	{
		set_val = value;
		EXISP_I2C_ISOSet(set_val);
		DEBUG_TRACE("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
		return 0;
	}
	DEBUG_TRACE("\n %s..%s valure = %d is invalidate..	\n",SENSOR_NAME_STRING(),__FUNCTION__,value);
	return -EINVAL;
}
#endif

static void sensor_set_hdr()
{
	icatch_sensor_write(SP7K_RDREG_INT_STS_REG_0, 0x04);
	EXISP_I2C_CapModeSet(0x01);
	EXISP_I2C_PvCapModeSet(0x01);
}

static void sensor_set_hdr_wait_clear()
{
	while (!(icatch_sensor_read(SP7K_RDREG_INT_STS_REG_0)&0x04)) {
		msleep(10);
	}
	icatch_sensor_write(SP7K_RDREG_INT_STS_REG_0, 0x04);
}

#if CONFIG_SENSOR_Wdr
// EXISP_I2C_IspFuncSet(), bit 0 : DWDR
static void sensor_set_wdr(bool Enable)
{
	if(Enable){
		EXISP_I2C_IspFuncSet( icatch_sensor_read(SP7K_REG_BASE|(SP7K_MSG_ISP_FUNCTION&0x7F)) | 0x01);
	}else{
		EXISP_I2C_IspFuncSet( icatch_sensor_read(SP7K_REG_BASE|(SP7K_MSG_ISP_FUNCTION&0x7F)) &~0x01);
	}
}
#endif

#if CONFIG_SENSOR_EDGE
static void sensor_set_edge(bool Enable)
{
	if(Enable){
		EXISP_I2C_IspFuncSet( icatch_sensor_read(SP7K_REG_BASE|(SP7K_MSG_ISP_FUNCTION&0x7F)) | 0x02);
	}else{
		EXISP_I2C_IspFuncSet( icatch_sensor_read(SP7K_REG_BASE|(SP7K_MSG_ISP_FUNCTION&0x7F)) &~0x02);
	}
}
#endif

static const struct v4l2_querymenu icatch_sensor_menus[] =
{
	#if CONFIG_SENSOR_WhiteBalance
	{ .id = V4L2_CID_DO_WHITE_BALANCE,	.index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
	{ .id = V4L2_CID_DO_WHITE_BALANCE,	.index = 2,  .name = "fluorescent", .reserved = 0,}, {	.id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
	{ .id = V4L2_CID_DO_WHITE_BALANCE,	.index = 4,  .name = "cloudy-daylight", .reserved = 0,},
	#endif

	#if CONFIG_SENSOR_Effect
	{ .id = V4L2_CID_EFFECT,  .index = 0,  .name = "normal",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "aqua",	.reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 4,  .name = "mono", .reserved = 0,}, { .id = V4L2_CID_EFFECT,  .index = 5,  .name = "none", .reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 6,  .name = "aura", .reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 7,  .name = "vintage", .reserved = 0,}, { .id = V4L2_CID_EFFECT,  .index = 8,  .name = "vintage2", .reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 9,  .name = "lomo", .reserved = 0,}, { .id = V4L2_CID_EFFECT,  .index = 10,  .name = "red", .reserved = 0,},
	{ .id = V4L2_CID_EFFECT,  .index = 11,  .name = "blue", .reserved = 0,}, { .id = V4L2_CID_EFFECT,  .index = 12,  .name = "green", .reserved = 0,},
	#endif

	#if CONFIG_SENSOR_Scene
	{ .id = V4L2_CID_SCENE,  .index = 0, .name = "normal", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "auto", .reserved = 0,},
	{ .id = V4L2_CID_SCENE,  .index = 2, .name = "landscape", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,	.index = 3,  .name = "night", .reserved = 0,},
	{ .id = V4L2_CID_SCENE,  .index = 4, .name = "night_portrait", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 5,  .name = "snow", .reserved = 0,},
	{ .id = V4L2_CID_SCENE,  .index = 6, .name = "sports", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 7,  .name = "candlelight", .reserved = 0,},
	#endif

	#if CONFIG_SENSOR_AntiBanding
    { .id = V4L2_CID_ANTIBANDING,  .index = 0, .name = "50hz", .reserved = 0,} ,{ .id = V4L2_CID_ANTIBANDING,  .index = 1, .name = "60hz", .reserved = 0,} ,
	#endif

	#if CONFIG_SENSOR_Flash
	{ .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
	{ .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {	.id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
	#endif

#if CONFIG_SENSOR_ISO
	{ .id = V4L2_CID_ISO,	.index = 0,	.name = "auto",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 1,	.name = "50",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 2,	.name = "100",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 3,	.name = "200",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 4,	.name = "400",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 5,	.name = "800",	.reserved = 0, },
	{ .id = V4L2_CID_ISO,	.index = 6,	.name = "1600",	.reserved = 0, },
#endif
};

static	struct v4l2_queryctrl icatch_sensor_controls[] =
{
	#if CONFIG_SENSOR_WhiteBalance
	{
		.id 		= V4L2_CID_DO_WHITE_BALANCE,
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
		.id 		= V4L2_CID_BRIGHTNESS,
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
		.id 		= V4L2_CID_EFFECT,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Effect Control",
		.minimum	= 0,
		.maximum	= 12,
		.step		= 1,
		.default_value = 5,
	},
	#endif

	#if CONFIG_SENSOR_Exposure
	{
		.id 		= V4L2_CID_EXPOSURE,
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
		.id 		= V4L2_CID_SATURATION,
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
		.id 		= V4L2_CID_CONTRAST,
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
		.id 		= V4L2_CID_HFLIP,
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
		.id 		= V4L2_CID_VFLIP,
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
		.id 		= V4L2_CID_SCENE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Scene Control",
		.minimum	= 0,
		.maximum	= 7,
		.step		= 1,
		.default_value = 1,
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

	#if CONFIG_SENSOR_MeteringAreas
	{
		.id 	= V4L2_CID_METERING_AREAS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "MeteringAreas Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 1,
	},
	#endif

	#if CONFIG_SENSOR_Wdr
	{
		.id 		= V4L2_CID_WDR,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "WDR Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
	#endif

	#if CONFIG_SENSOR_EDGE
	{
		.id 		= V4L2_CID_EDGE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "EDGE Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 1,
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
		.id 		= V4L2_CID_ZOOM_ABSOLUTE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "DigitalZoom Control",
		.minimum	= 100,
		.maximum	= 275, // app pass 275-25 maximum
		.step		= 25,
		.default_value = 100,
	},
	#endif

	#if CONFIG_SENSOR_Focus
	/*{
		.id 	= V4L2_CID_FOCUS_RELATIVE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Focus Control",
		.minimum	= -1,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	}, */
	{
		.id		= V4L2_CID_FOCUSZONE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "FocusZone Control",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 1,
		.default_value = 0,
	},
	{
		.id 	= V4L2_CID_FOCUS_ABSOLUTE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Focus Control",
		.minimum	= 0,
		.maximum	= 0xff,
		.step		= 1,
		.default_value = 0,
	},
	{
		.id 	= V4L2_CID_FOCUS_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Focus Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},
	/*
	{
		.id 	= V4L2_CID_FOCUS_CONTINUOUS,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Focus Control",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 0,
	},*/
	#endif

	#if CONFIG_SENSOR_Flash
	{
		.id 	= V4L2_CID_FLASH,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flash Control Focus",
		.minimum	= 0,
		.maximum	= 3,
		.step		= 1,
		.default_value = 0,
	},
	#endif
	#if CONFIG_SENSOR_FOCUS_ZONE
	{
		.id 	= V4L2_CID_FOCUSZONE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Focus Zone support",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 1,
	},
	#endif
	#if CONFIG_SENSOR_FACE_DETECT
	{
		.id 	= V4L2_CID_FACEDETECT,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "face dectect support",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value = 1,
	},
	#endif
#if CONFIG_SENSOR_ISO
	{
		.id		= V4L2_CID_ISO,
		.type		= V4L2_CTRL_TYPE_MENU,
		.minimum 	= 0,
		.maximum	= 6,
		.step		= 1,
		.default_value	= 0,
	},
#endif
};

static int icatch_sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
	return 0;
}
static int icatch_sensor_resume(struct soc_camera_device *icd)
{
	return 0;

}
static int icatch_sensor_set_bus_param(struct soc_camera_device *icd,
								unsigned long flags)
{

	return 0;
}
static unsigned long icatch_sensor_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SENSOR_BUS_PARAM;

	return soc_camera_apply_sensor_flags(icl, flags);
}

struct soc_camera_ops sensor_ops =
{
	.suspend			= icatch_sensor_suspend,
	.resume				= icatch_sensor_resume,
	.set_bus_param		= icatch_sensor_set_bus_param,
	.query_bus_param	= icatch_sensor_query_bus_param,
	.controls			= icatch_sensor_controls,
	.menus				= icatch_sensor_menus,
	.num_controls		= ARRAY_SIZE(icatch_sensor_controls),
	.num_menus			= ARRAY_SIZE(icatch_sensor_menus),
};

int icatch_update_sensor_ops(
	const struct v4l2_queryctrl *controls,
	int num_controls,
	const struct v4l2_querymenu *menus,
	int num_menus) {
	//const struct v4l2_queryctrl **ppctrl = NULL;
	//const struct v4l2_querymenu **ppmenus = NULL;
	//int * pnum_controls = NULL;
	//int * pnum_menus = NULL;

	if (controls == NULL || menus == NULL) {
		return -EINVAL;
	}

	if (	(controls != NULL && num_controls == 0) ||
		(menus != NULL && num_menus == 0)) {
		return -EINVAL;
	}

	DEBUG_TRACE("%s:input pctrl=0x%x num_controls = %d pmenus=0x%x num_menus=%d\n",
		__FUNCTION__,
		controls, num_controls,
		menus, num_menus);

	DEBUG_TRACE("%s:old pctrl=0x%x num_controls = %d pmenus=0x%x num_menus=%d\n",
		__FUNCTION__,
		sensor_ops.controls, sensor_ops.num_controls,
		sensor_ops.menus, sensor_ops.num_menus);

	//ppctrl = (const struct v4l2_queryctrl **) &(sensor_ops.controls);
	//pnum_controls = (int *) &(sensor_ops.num_controls);
	//ppmenus = (const struct v4l2_querymenu **) &(sensor_ops.menus);
	//pnum_menus = (int *) &(sensor_ops.num_menus);

	sensor_ops.controls = controls;
	sensor_ops.num_controls = num_controls;

	sensor_ops.menus = menus;
	sensor_ops.num_menus = num_menus;

	DEBUG_TRACE("%s:new pctrl=0x%x num_controls = %d pmenus=0x%x num_menus=%d\n",
		__FUNCTION__,
		sensor_ops.controls, sensor_ops.num_controls,
		sensor_ops.menus, sensor_ops.num_menus);

	return 0;
}

#if CALIBRATION_MODE_FUN
struct proc_dir_entry *g_icatch7002_proc_entry = NULL;
#define PROC_ENTRY_NAME	SENSOR_NAME_STRING()
int g_is_calibrationMode = 0;
char g_procbuff[1024];
int icatch_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	if (off > 0) {
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "is_calibration %d\n", g_is_calibrationMode);
	return len;
}

int icatch_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	char *ptr = NULL;
	if (count >= sizeof(g_procbuff)) {
		DEBUG_TRACE("%s no space\n", __FUNCTION__);
		return -ENOSPC;
	}

	if (copy_from_user(g_procbuff, buffer, count)) {
		DEBUG_TRACE("%s copy from user fail %d\n", __FUNCTION__, count);
		return -EFAULT;
	}
	g_procbuff[count] = 0;

	if ( (ptr = strstr(g_procbuff, "is_calibration")) == NULL) {
		goto l_ret;
	}

	ptr += strlen("is_calibration");
	while(*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
		ptr++;
	}

	switch (*ptr) {
	case '0':
		g_is_calibrationMode = 0;
		DEBUG_TRACE("%s disable calibration mode\n", __FUNCTION__);
		break;
	case '1':
		g_is_calibrationMode = 1;
		DEBUG_TRACE("%s enable calibration mode\n", __FUNCTION__);
		break;
	}

l_ret:
	return count;
}

void icatch_create_proc_entry()
{
	if (g_icatch7002_proc_entry == NULL) {
		DEBUG_TRACE("%s need create_proc_entry\n", __FUNCTION__);
		g_icatch7002_proc_entry = create_proc_entry(PROC_ENTRY_NAME, O_RDWR, NULL);
		if (g_icatch7002_proc_entry) {
			memset(g_procbuff, 0, sizeof(g_procbuff));
			g_icatch7002_proc_entry->read_proc = icatch_proc_read;
			g_icatch7002_proc_entry->write_proc = icatch_proc_write;
		} else {
			DEBUG_TRACE("%s create_proc_entry fail\n", __FUNCTION__);
		}
	}
}

void icatch_remove_proc_entry()
{
	if (g_icatch7002_proc_entry != NULL) {
		remove_proc_entry(PROC_ENTRY_NAME, NULL);
		g_icatch7002_proc_entry = NULL;
	}
}
#endif

#if CONFIG_SENSOR_Focus

void __dump_i2c(UINT16 addr_s, UINT16 addr_e) {
	int size = (addr_e - addr_s + 1);
	int i = 0;
	int soffset = addr_s%16;
	char buf[100] = {0};
	char lbuf[12];

	for (i = 0; i < soffset; i++) {
		if (i == 0) {
			sprintf(lbuf, "%08X:", addr_s / 16 * 16);
			strcat(buf, lbuf);
		}
		sprintf(lbuf, "   ");
		strcat(buf, lbuf);
	}

	size += soffset;
	i = soffset;
	while( i < size) {
		if ((i%16 == 0) && (i != 0)) {
			DEBUG_TRACE("%s\n", buf);
		}
		if (i%16 == 0) {
			buf[0] = 0;
			sprintf(lbuf, "%08X:", (addr_s + i - soffset) / 16 * 16);
			strcat(buf, lbuf);
		}
		sprintf(lbuf, " %02X", icatch_sensor_read(addr_s + i - soffset));
		strcat(buf, lbuf);
		i++;
	}

	DEBUG_TRACE("%s\n", buf);
}

static int sensor_set_auto_focus(struct i2c_client *client, int value)
{
	struct isp_dev *sensor = to_sensor(client);
	//u8 zone_x = 0x0,zone_y = 0x0; // 0->0x0f
	UINT16 w = 0;
	int ret = 0;
	int cnt = 100;

	EXISP_I2C_FocusModeSet(value);
	//EXISP_I2C_AFROITriggerSet();
	if (value != SENSOR_AF_INFINITY) {
		EXISP_I2C_ROISwitchSet(01);
		//set the zone
		DEBUG_TRACE("%s: lx = %d,rx = %d,ty = %d,dy = %d\n", __FUNCTION__, sensor->isp_priv_info.focus_zone.lx,sensor->isp_priv_info.focus_zone.rx,sensor->isp_priv_info.focus_zone.ty,sensor->isp_priv_info.focus_zone.dy);
		w = sensor->isp_priv_info.focus_zone.rx - sensor->isp_priv_info.focus_zone.lx;
		//zone_x = (sensor->isp_priv_info.focus_zone.lx << 4) | (sensor->isp_priv_info.focus_zone.rx & 0x0f);
		//zone_y = (sensor->isp_priv_info.focus_zone.ty << 4) | (sensor->isp_priv_info.focus_zone.dy & 0x0f);
		//auto focus
		//sendI2cCmd(client, 0x0E, 0x00);
		if( w != 0) {
			EXISP_TAFTAEROISet(
					TAFTAE_TAF_ONLY,
					w,
					sensor->isp_priv_info.focus_zone.lx,
					sensor->isp_priv_info.focus_zone.ty,
					0, 0, 0);
			DEBUG_TRACE("%s:auto focus, val = %d, w = 0x%x, x = 0x%x y = 0x%x\n",__func__,value, w, sensor->isp_priv_info.focus_zone.lx, sensor->isp_priv_info.focus_zone.ty);
		}else{
			EXISP_TAFTAEROISet(TAFTAE_TAF_ONLY, 0x80, 0x1bf, 0x1bf, 0, 0, 0);
			DEBUG_TRACE("%s:auto focus, all zero, val = %d, size=0x80, x=0x1bf, y=0x1bf\n",__func__,value);
		}
	}

	if (value == SENSOR_AF_CONTINUOUS) {
		DEBUG_TRACE("%s:continuous focus done\n",__func__);
		return 0;
	}

	while (cnt--) {
		if (EXISP_I2C_AFStatusGet() == 0) {
			 break;
		}
		msleep(30);
	}

	//__dump_i2c(0x7200, 0x727f);

	if (cnt <= 0) {
		DEBUG_TRACE("%s: focus timeout %d\n",__func__, value);
		//__dump_i2c(0x7005, 0x7005);
		return 1;
	}

	if (EXISP_I2C_AFResultGet() != 0) {
		DEBUG_TRACE("%s: focus fail %d\n",__func__, value);
		return 1;
	}

	DEBUG_TRACE("%s: focus success %d\n\n",__func__, value);
	return 0;
}

/*
enum sensor_work_state
{
	sensor_work_ready = 0,
	sensor_working,
};
enum sensor_wq_result
{
    WqRet_success = 0,
    WqRet_fail = -1,
    WqRet_inval = -2
};
enum sensor_wq_cmd
{
    WqCmd_af_init,
    WqCmd_af_single,
    WqCmd_af_special_pos,
    WqCmd_af_far_pos,
    WqCmd_af_near_pos,
    WqCmd_af_continues,
    WqCmd_af_return_idle,
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
};*/

static void sensor_af_workqueue(struct work_struct *work)
{
	struct sensor_work *sensor_work = container_of(work, struct sensor_work, dwork.work);
	struct i2c_client *client = sensor_work->client;
	struct isp_dev *sensor = to_sensor(client);

	DEBUG_TRACE("%s %s Enter, cmd:0x%x \n",SENSOR_NAME_STRING(), __FUNCTION__,sensor_work->cmd);

	mutex_lock(&sensor->wq_lock);

	DEBUG_TRACE("%s:auto focus, val = %d\n",__func__,sensor_work->var);
	//auto focus
	if(sensor_set_auto_focus(client,sensor_work->var) == 0){
		sensor_work->result = WqRet_success;
	}else{
		sensor_work->result = WqRet_fail;
		DEBUG_TRACE("%s:auto focus failed\n",__func__);
	}
	DEBUG_TRACE("%s:auto focus done\n",__func__);

	//set_end:
	if (sensor_work->wait == false) {
		kfree((void*)sensor_work);
	} else {
		wake_up(&sensor_work->done);
	}

	mutex_unlock(&sensor->wq_lock);
	return;
}

static int sensor_af_workqueue_set(struct soc_camera_device *icd, enum sensor_wq_cmd cmd, int var, bool wait)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct isp_dev *sensor = to_sensor(client);
	struct sensor_work *wk;
	int ret=0;

	if (sensor->sensor_wq == NULL) {
		ret = -EINVAL;
		goto sensor_af_workqueue_set_end;
	}

	wk = kzalloc(sizeof(struct sensor_work), GFP_KERNEL);
	if (wk) {
		wk->client = client;
		INIT_WORK(&(wk->dwork.work), sensor_af_workqueue);
		wk->cmd = cmd;
		wk->result = WqRet_inval;
		wk->wait = wait;
		wk->var = var;
		init_waitqueue_head(&wk->done);

		queue_delayed_work(sensor->sensor_wq,&(wk->dwork),0);

		/* ddl@rock-chips.com:
		* video_lock is been locked in v4l2_ioctl function, but auto focus may slow,
		* As a result any other ioctl calls will proceed very, very slowly since each call
		* will have to wait for the AF to finish. Camera preview is pause,because VIDIOC_QBUF
		* and VIDIOC_DQBUF is sched. so unlock video_lock here.
		*/
		if (wait == true) {
			mutex_unlock(&icd->video_lock);
			if (wait_event_timeout(wk->done, (wk->result != WqRet_inval), msecs_to_jiffies(5000)) == 0) {  //hhb
				DEBUG_TRACE("%s %s cmd(%d) is timeout!\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			}
			flush_workqueue(sensor->sensor_wq);
			ret = wk->result;
			kfree((void*)wk);
			mutex_lock(&icd->video_lock);
		}

	} else {
		DEBUG_TRACE("%s %s cmd(%d) ingore,because struct sensor_work malloc failed!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
		ret = -1;
	}

sensor_af_workqueue_set_end:
	return ret;
}
#endif

static int icatch_sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct isp_dev *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
	int pid = 0;
	u8 sensorid = 0;

	sensorid = strcmp(icl->board_info->type, SENSOR_REAR_NAME_STRING());
	if (sensorid) sensorid = 1;

#if CALIBRATION_MODE_FUN
	if (g_is_calibrationMode == 1 ) {
		if (sensorid == SENSOR_ID_REAR)
			sensorid = SENSOR_ID_REAR_CALIBRATION;

		DEBUG_TRACE("%s CALIBRATION MODE is enable, SENSOR_ID:%d\n", __FUNCTION__, sensorid);
	}
	sensor->isp_priv_info.sensor_id = sensorid;
#endif

	DEBUG_TRACE("@@NY@@%s: %d\n", __FUNCTION__, sensorid);
	//define the outputsize , qcif - cif is down-scaling from vga , or jaggies is very serious
	if(sensorid == SENSOR_ID_FRONT){
		// front camera
		/*OUTPUT_QCIF|OUTPUT_HQVGA|OUTPUT_QVGA |OUTPUT_CIF |OUTPUT_VGA | OUTPUT_SVGA|OUTPUT_720P|OUTPUT_XGA|OUTPUT_QXGA|OUTPUT_UXGA|OUTPUT_QSXGA|OUTPUT_QUADVGA*/
		sensor->isp_priv_info.outputSize =OUTPUT_QUADVGA;
		sensor->isp_priv_info.supportedSizeNum = 1;
		sensor->isp_priv_info.supportedSize[0] = OUTPUT_QUADVGA;
	} else {
		// rear camera
		/*OUTPUT_QCIF|OUTPUT_HQVGA|OUTPUT_QVGA |OUTPUT_CIF |OUTPUT_VGA | OUTPUT_SVGA|OUTPUT_720P|OUTPUT_XGA|OUTPUT_QXGA|OUTPUT_UXGA|OUTPUT_QSXGA|OUTPUT_QUADVGA*/
		sensor->isp_priv_info.outputSize = OUTPUT_QSXGA|OUTPUT_QUADVGA;
		sensor->isp_priv_info.supportedSizeNum = 2;
		sensor->isp_priv_info.supportedSize[0] = OUTPUT_QUADVGA;
		sensor->isp_priv_info.supportedSize[1] = OUTPUT_QSXGA;
	}
	sensor->isp_priv_info.curRes = -1;
	sensor->isp_priv_info.curPreviewCapMode = IDLE_MODE;
	sensor->isp_priv_info.had_setprvsize = 0;

#if CALIBRATION_MODE_FUN
	sensor->isp_priv_info.rk_query_PreviewCapMode = IDLE_MODE;
#endif
	g_icatch_i2c_client = client;
	if (sensor->before_init_cb) {
		if (sensor->before_init_cb(client) != 0) {
			DEBUG_TRACE("%s: before init callback return fail\n", __FUNCTION__);
			return -EINVAL;
		}
	}


#if 0
		//get id check
{
	int count = 200;
		while(count--){
		pid = icatch_sensor_read(0x0004);
		DEBUG_TRACE("\n %s	pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
		mdelay(1000);
		}
}
#endif

	if(icatch_load_fw(icd,sensorid)<0){
		DEBUG_TRACE("icatch7002 load sensor %d firmware failed!!-->%s:%d\n",sensorid, __FUNCTION__, __LINE__);
		return -ENODEV;
	}
	if(sensorid == SENSOR_ID_FRONT){
		// front camera
		icatch_sensor_write_array((void*)fs_reset_data);
	}
	else{
		// rear camera
		icatch_sensor_write_array((void*)rs_reset_data);
	}

	//50Hz
	EXISP_I2C_BandSelectionSet(0x01);
	//DEBUG_TRACE("%s Set BandSelection to 50Hz\n", __FUNCTION__);

#if 0
	//get id check
	mdelay(100);
	pid = EXISP_I2C_RearSensorIdGet();
	DEBUG_TRACE("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
	if (pid != SENSOR_ID) {
		DEBUG_TRACE("error: %s mismatched pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
		return -ENODEV;
	}
#endif

	/* sensor sensor information for initialization  */
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
	if (qctrl)
		sensor->isp_priv_info.whiteBalance = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_BRIGHTNESS);
	if (qctrl)
		sensor->isp_priv_info.brightness = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
	if (qctrl)
		sensor->isp_priv_info.effect = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE);
	if (qctrl){
		sensor->isp_priv_info.exposure = qctrl->default_value;
		}

	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SATURATION);
	if (qctrl)
		sensor->isp_priv_info.saturation = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_CONTRAST);
	if (qctrl)
		sensor->isp_priv_info.contrast = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_HFLIP);
	if (qctrl)
		sensor->isp_priv_info.mirror = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_VFLIP);
	if (qctrl)
		sensor->isp_priv_info.flip = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);
	if (qctrl){
		sensor->isp_priv_info.scene = qctrl->default_value;
		}
#if	CONFIG_SENSOR_AntiBanding
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ANTIBANDING);
	if (qctrl)
        sensor->isp_priv_info.antibanding = qctrl->default_value;
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_WHITEBALANCE_LOCK);
	if (qctrl)
        sensor->isp_priv_info.WhiteBalanceLock = qctrl->default_value;
#endif
#if CONFIG_SENSOR_ExposureLock
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE_LOCK);
	if (qctrl)
        sensor->isp_priv_info.ExposureLock = qctrl->default_value;
#endif
#if CONFIG_SENSOR_MeteringAreas
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_METERING_AREAS);
	if (qctrl)
        sensor->isp_priv_info.MeteringAreas = qctrl->default_value;
#endif
#if CONFIG_SENSOR_Wdr
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_WDR);
	if (qctrl)
        sensor->isp_priv_info.Wdr = qctrl->default_value;
#endif
#if CONFIG_SENSOR_EDGE
	sensor_set_edge(1);
#endif
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl)
		sensor->isp_priv_info.digitalzoom = qctrl->default_value;
	/* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
	//qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_AUTO);
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_AUTO);
	if (qctrl)
		sensor->isp_priv_info.auto_focus = SENSOR_AF_AUTO;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FACEDETECT);
	if (qctrl)
		sensor->isp_priv_info.face = qctrl->default_value;
#if CONFIG_SENSOR_ISO
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ISO);
	if (qctrl)
		sensor->isp_priv_info.iso = qctrl->default_value;
#endif
	return 0;
}

static int icatch_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct isp_dev *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;

	qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

	if (!qctrl)
	{
		DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id)
	{
		case V4L2_CID_BRIGHTNESS:
			{
				ctrl->value = sensor->isp_priv_info.brightness;
				break;
			}
		case V4L2_CID_SATURATION:
			{
				ctrl->value = sensor->isp_priv_info.saturation;
				break;
			}
		case V4L2_CID_CONTRAST:
			{
				ctrl->value = sensor->isp_priv_info.contrast;
				break;
			}
		case V4L2_CID_DO_WHITE_BALANCE:
			{
				ctrl->value = sensor->isp_priv_info.whiteBalance;
				break;
			}
		case V4L2_CID_EXPOSURE:
			{
				ctrl->value = sensor->isp_priv_info.exposure;
				break;
			}
		case V4L2_CID_HFLIP:
			{
				ctrl->value = sensor->isp_priv_info.mirror;
				break;
			}
		case V4L2_CID_VFLIP:
			{
				ctrl->value = sensor->isp_priv_info.flip;
				break;
			}
		case V4L2_CID_ZOOM_ABSOLUTE:
			{
				ctrl->value = sensor->isp_priv_info.digitalzoom;
				break;
			}
		default :
				break;
	}
	return 0;
}

static int icatch_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct isp_dev *sensor = to_sensor(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	const struct v4l2_queryctrl *qctrl;

	qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

	if (!qctrl)
	{
		DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id)
	{
#if CONFIG_SENSOR_Brightness
		case V4L2_CID_BRIGHTNESS:
			{
				if (ctrl->value != sensor->isp_priv_info.brightness)
				{
					if (sensor_set_brightness(icd, qctrl,ctrl->value) != 0)
					{
						return -EINVAL;
					}
					sensor->isp_priv_info.brightness = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Exposure
		case V4L2_CID_EXPOSURE:
			{
				if (ctrl->value != sensor->isp_priv_info.exposure)
				{
					if (sensor_set_exposure(icd, qctrl,ctrl->value) != 0)
					{
						return -EINVAL;
					}
					sensor->isp_priv_info.exposure = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Saturation
		case V4L2_CID_SATURATION:
			{
				if (ctrl->value != sensor->isp_priv_info.saturation)
				{
					if (sensor_set_saturation(icd, qctrl,ctrl->value) != 0)
					{
						return -EINVAL;
					}
					sensor->isp_priv_info.saturation = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Contrast
		case V4L2_CID_CONTRAST:
			{
				if (ctrl->value != sensor->isp_priv_info.contrast)
				{
					if (sensor_set_contrast(icd, qctrl,ctrl->value) != 0)
					{
						return -EINVAL;
					}
					sensor->isp_priv_info.contrast = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_WhiteBalance
		case V4L2_CID_DO_WHITE_BALANCE:
			{
				if (ctrl->value != sensor->isp_priv_info.whiteBalance)
				{
					if (icatch_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
					{
						return -EINVAL;
					}
					sensor->isp_priv_info.whiteBalance = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Mirror
		case V4L2_CID_HFLIP:
			{
				if (ctrl->value != sensor->isp_priv_info.mirror)
				{
					if (sensor_set_mirror(icd, qctrl,ctrl->value) != 0)
						return -EINVAL;
					sensor->isp_priv_info.mirror = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Flip
		case V4L2_CID_VFLIP:
			{
				if (ctrl->value != sensor->isp_priv_info.flip)
				{
					if (sensor_set_flip(icd, qctrl,ctrl->value) != 0)
						return -EINVAL;
					sensor->isp_priv_info.flip = ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_DigitalZoom
		case V4L2_CID_ZOOM_ABSOLUTE:
			{
				int val_offset = 0;
				DEBUG_TRACE("V4L2_CID_ZOOM_ABSOLUTE ...... ctrl->value = %d\n",ctrl->value);
				if ((ctrl->value < qctrl->minimum) || (ctrl->value > qctrl->maximum)){
					return -EINVAL;
					}

				if (ctrl->value != sensor->isp_priv_info.digitalzoom)
				{
					val_offset = ctrl->value -sensor->isp_priv_info.digitalzoom;

					if (sensor_set_digitalzoom(icd, qctrl,&val_offset) != 0)
						return -EINVAL;
					sensor->isp_priv_info.digitalzoom += val_offset;

					DEBUG_TRACE("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->isp_priv_info.digitalzoom);
				}

				break;
			}
		case V4L2_CID_ZOOM_RELATIVE:
			{
				if (ctrl->value)
				{
					if (sensor_set_digitalzoom(icd, qctrl,&ctrl->value) != 0)
						return -EINVAL;
					sensor->isp_priv_info.digitalzoom += ctrl->value;

					DEBUG_TRACE("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->isp_priv_info.digitalzoom);
				}
				break;
			}
#endif
		default:
			break;
	}

	return 0;
}

static int icatch_g_ext_control(struct soc_camera_device *icd , struct v4l2_ext_control *ext_ctrl)
{
	const struct v4l2_queryctrl *qctrl;
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct isp_dev *sensor = to_sensor(client);

	qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

	if (!qctrl)
	{
		DEBUG_TRACE("\n %s ioctrl id = 0x%x  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
		return -EINVAL;
	}

	switch (ext_ctrl->id)
	{
		case V4L2_CID_SCENE:
			{
				ext_ctrl->value = sensor->isp_priv_info.scene;
				break;
			}
#if CONFIG_SENSOR_AntiBanding
        case V4L2_CID_ANTIBANDING:
            {
                ext_ctrl->value = sensor->isp_priv_info.antibanding;
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
        case V4L2_CID_WHITEBALANCE_LOCK:
            {
                ext_ctrl->value = sensor->isp_priv_info.WhiteBalanceLock;
                break;
            }
#endif
#if CONFIG_SENSOR_ExposureLock
		case V4L2_CID_EXPOSURE_LOCK:
            {
                ext_ctrl->value = sensor->isp_priv_info.ExposureLock;
                break;
            }
#endif
#if CONFIG_SENSOR_MeteringAreas
		case V4L2_CID_METERING_AREAS:
            {
                ext_ctrl->value = sensor->isp_priv_info.MeteringAreas;
                break;
            }
#endif
#if CONFIG_SENSOR_Wdr
		case V4L2_CID_WDR:
			{
				ext_ctrl->value = sensor->isp_priv_info.Wdr;
				break;
			}
#endif
#if CONFIG_SENSOR_EDGE
		case V4L2_CID_EDGE:
			{
				ext_ctrl->value = EXISP_I2C_CapEdgeInfoGet();
				break;
			}
#endif
		case V4L2_CID_EFFECT:
			{
				ext_ctrl->value = sensor->isp_priv_info.effect;
				break;
			}
		case V4L2_CID_ZOOM_ABSOLUTE:
			{
				ext_ctrl->value = sensor->isp_priv_info.digitalzoom;
				break;
			}
		case V4L2_CID_ZOOM_RELATIVE:
			{
				return -EINVAL;
			}
		case V4L2_CID_FOCUS_ABSOLUTE:
			{
				return -EINVAL;
			}
		case V4L2_CID_FOCUS_RELATIVE:
			{
				return -EINVAL;
			}
		case V4L2_CID_FLASH:
			{
				ext_ctrl->value = sensor->isp_priv_info.flash;
				break;
			}
		case V4L2_CID_FACEDETECT:
			{
				ext_ctrl->value =sensor->isp_priv_info.face ;
				break;
			}
#if CONFIG_SENSOR_ISO
		case V4L2_CID_ISO:
		{
			ext_ctrl->value = sensor->isp_priv_info.iso;
			if(ext_ctrl->value == 0){
				ext_ctrl->value = icatch_sensor_read(SP7K_RDREG_ISO_H);
				ext_ctrl->value <<= 8;
				ext_ctrl->value |= icatch_sensor_read(SP7K_RDREG_ISO_L);
			}
			break;
		}
#endif
#if CONFIG_SENSOR_JPEG_EXIF
		case V4L2_CID_JPEG_EXIF:
		{
			RkExifInfo *pExitInfo = (RkExifInfo *)ext_ctrl->value;

			UINT8 ucExpTimeNumerator;
			UINT32 ulExpTimeDenominator;
			UINT8 ucExpTimeCompensation;
			UINT16 usLensFocalLength;
			UINT16 usIsoInfo;
			UINT8 ucFlashInfo;

			EXISP_ExifInfoGet(
				&ucExpTimeNumerator,
				&ulExpTimeDenominator,
				&ucExpTimeCompensation,
				&usLensFocalLength,
				&usIsoInfo,
				&ucFlashInfo);

			pExitInfo->ExposureTime.num = ucExpTimeNumerator;
			pExitInfo->ExposureTime.denom = ulExpTimeDenominator;
			//pExitInfo->Flash = ucFlashInfo;
			pExitInfo->ISOSpeedRatings = usIsoInfo;
			pExitInfo->FocalPlaneYResolution.num = EXISP_I2C_CapEdgeInfoGet();

			break;
		}
#endif
		default :
			break;
	}
	return 0;
}

static int icatch_g_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	int i, error_cnt=0, error_idx=-1;

	for (i=0; i<ext_ctrl->count; i++) {
		if (icatch_g_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
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

static int icatch_s_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
	const struct v4l2_queryctrl *qctrl;
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct isp_dev *sensor = to_sensor(client);
	int val_offset;
	int ret = 0;

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
				if (ext_ctrl->value != sensor->isp_priv_info.scene)
				{
					if (icatch_set_scene(icd, qctrl,ext_ctrl->value) != 0)
						return -EINVAL;
					sensor->isp_priv_info.scene = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_AntiBanding
		case V4L2_CID_ANTIBANDING:
			{
				if (ext_ctrl->value != sensor->isp_priv_info.antibanding)
				{
					if(ext_ctrl->value){
						EXISP_I2C_BandSelectionSet(2);	//60Hz
					}else{
						EXISP_I2C_BandSelectionSet(1);	//50Hz
					}
					sensor->isp_priv_info.antibanding = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_WhiteBalanceLock
		case V4L2_CID_WHITEBALANCE_LOCK:
			{
				if (ext_ctrl->value != sensor->isp_priv_info.WhiteBalanceLock)
				{
					if(ext_ctrl->value){
						EXISP_I2C_VendreqCmdSet(4);
					}else{
						EXISP_I2C_VendreqCmdSet(6);
					}
					sensor->isp_priv_info.WhiteBalanceLock = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_ExposureLock
		case V4L2_CID_EXPOSURE_LOCK:
			{
				if (ext_ctrl->value != sensor->isp_priv_info.ExposureLock)
				{
					if(ext_ctrl->value){
						EXISP_I2C_VendreqCmdSet(3);
					}else{
						EXISP_I2C_VendreqCmdSet(5);
					}
					sensor->isp_priv_info.ExposureLock = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_MeteringAreas
		case V4L2_CID_METERING_AREAS:
			{
				if (ext_ctrl->value)
				{
					int lx,ty,rx,dy, w;
					DEBUG_TRACE("V4L2_CID_METERING_AREAS EXISP_TAFTAEROISet TAFTAE_TAE_ONLY rect = %d,%d,%d,%d\n",
						ext_ctrl->rect[0],ext_ctrl->rect[1],ext_ctrl->rect[2],ext_ctrl->rect[3]);


					lx = ext_ctrl->rect[0];
					ty = ext_ctrl->rect[1];
					rx = ext_ctrl->rect[2];
					dy = ext_ctrl->rect[3];

					lx = ((lx + 1000) * 1024)/2001;
					ty = ((ty + 1000) * 1024)/2001;
					rx = ((rx + 1000) * 1024)/2001;
					dy = ((dy + 1000) * 1024)/2001;
					DEBUG_TRACE("V4L2_CID_METERING_AREAS EXISP_TAFTAEROISet TAFTAE_TAE_ONLY real rect = %d,%d,%d,%d\n",
						lx, ty, rx, dy);
					w = rx - lx;
					if (w < 0) {
						w = -w;
					}

					if (w == 0) {
						ret = EXISP_TAFTAEROISet(TAFTAE_TAE_ONLY, 0, 0, 0, 0x80, 0x1bf, 0x1bf);
					} else {
						ret = EXISP_TAFTAEROISet(
								TAFTAE_TAE_ONLY,
								0, 0, 0,
								w,
								lx,
								ty );
					}

					if (ret != SUCCESS) {
						return -EINVAL;
					}
				}else{
					DEBUG_TRACE("V4L2_CID_METERING_AREAS TAFTAE_TAE_OFF TAFTAE_TAE_ONLY\n");
					if (
						EXISP_TAFTAEROISet( TAFTAE_TAE_OFF,0,0,0,0,0,0 ) != SUCCESS
					){
							return -EINVAL;
					}
				}
				sensor->isp_priv_info.MeteringAreas = ext_ctrl->value;
				break;
			}
#endif
#if CONFIG_SENSOR_Wdr
		case V4L2_CID_WDR:
			{
				if (ext_ctrl->value != sensor->isp_priv_info.Wdr)
				{
					sensor_set_wdr(ext_ctrl->value);
					sensor->isp_priv_info.Wdr = ext_ctrl->value;
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Effect
		case V4L2_CID_EFFECT:
			{
				if (ext_ctrl->value != sensor->isp_priv_info.effect)
				{
					if (icatch_set_effect(icd, qctrl,ext_ctrl->value,ext_ctrl->rect[0]) != 0)
						return -EINVAL;
					sensor->isp_priv_info.effect= ext_ctrl->value;
				}
				if(sensor->isp_priv_info.effect == 6)
				{
					EXISP_I2C_AuraColorIndexSet(ext_ctrl->rect[0]);
				}
				break;
			}
#endif
#if CONFIG_SENSOR_DigitalZoom
		case V4L2_CID_ZOOM_ABSOLUTE:
			{
				DEBUG_TRACE("V4L2_CID_ZOOM_ABSOLUTE ...... ext_ctrl->value = %d\n",ext_ctrl->value);
				if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum)){
					return -EINVAL;
				}

				if (ext_ctrl->value != sensor->isp_priv_info.digitalzoom)
				{
					val_offset = ext_ctrl->value -sensor->isp_priv_info.digitalzoom;

					if (sensor_set_digitalzoom(icd, qctrl,&val_offset) != 0)
						return -EINVAL;
					sensor->isp_priv_info.digitalzoom += val_offset;

					DEBUG_TRACE("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->isp_priv_info.digitalzoom);
				}

				break;
			}
		case V4L2_CID_ZOOM_RELATIVE:
			{
				if (ext_ctrl->value)
				{
					if (sensor_set_digitalzoom(icd, qctrl,&ext_ctrl->value) != 0)
						return -EINVAL;
					sensor->isp_priv_info.digitalzoom += ext_ctrl->value;

					DEBUG_TRACE("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->isp_priv_info.digitalzoom);
				}
				break;
			}
#endif
#if CONFIG_SENSOR_Focus
	case V4L2_CID_FOCUS_ABSOLUTE:
		{
			//DO MACRO
			DEBUG_TRACE("%s: V4L2_CID_FOCUS_ABSOLUTE value %d\n", __FUNCTION__, ext_ctrl->value);
			if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
				return -EINVAL;

			/*
			if (SENSOR_AF_CONTINUOUS == sensor->isp_priv_info.auto_focus) {
				sensor_af_workqueue_set(icd,0,SENSOR_AF_CONTINUOUS_OFF,true);
			}*/

			if (ext_ctrl->value == 0xff) {
				ret = sensor_af_workqueue_set(icd,0,SENSOR_AF_MACRO,true);
				//if (sensor_af_workqueue_set(icd,0,SENSOR_AF_MACRO,true) != 0)
				//	ret = -EAGAIN;

				sensor->isp_priv_info.auto_focus = SENSOR_AF_MACRO;
			} else if(ext_ctrl->value == 0){
				if (sensor_af_workqueue_set(icd,0,SENSOR_AF_INFINITY,true) != 0)
					ret = -EAGAIN;
				sensor->isp_priv_info.auto_focus = SENSOR_AF_INFINITY;
			}

			break;
		}
	case V4L2_CID_FOCUS_RELATIVE:
		{
	  //	  if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
	//			  return -EINVAL;

	 // 	  sensor_set_focus_relative(icd, qctrl,ext_ctrl->value);
			break;
		}

	case V4L2_CID_FOCUS_AUTO:
		{

			/*
			if (SENSOR_AF_CONTINUOUS == sensor->isp_priv_info.auto_focus) {
				sensor_af_workqueue_set(icd,0,SENSOR_AF_CONTINUOUS_OFF,true);
			}*/
			DEBUG_TRACE("%s: V4L2_CID_FOCUS_AUTO value %d\n", __FUNCTION__, ext_ctrl->value);
			if ((ext_ctrl->value == 1)||(sensor->isp_priv_info.auto_focus == SENSOR_AF_AUTO)) {
				ret = sensor_af_workqueue_set(icd,0,SENSOR_AF_AUTO,true);
				//if (sensor_af_workqueue_set(icd,0,SENSOR_AF_AUTO,true) != 0)
				//	ret = -EAGAIN;
				sensor->isp_priv_info.auto_focus = SENSOR_AF_AUTO;
			} else if(ext_ctrl->value == 0){
				ret = sensor_af_workqueue_set(icd,0,SENSOR_AF_INFINITY,true);
				//if (sensor_af_workqueue_set(icd,0,SENSOR_AF_INFINITY,true) != 0)
				//	ret = -EAGAIN;

				sensor->isp_priv_info.auto_focus = SENSOR_AF_INFINITY;
			}
			break;
		}
	case V4L2_CID_FOCUS_CONTINUOUS:
		{
			DEBUG_TRACE("%s: V4L2_CID_FOCUS_CONTINUOUS value %d\n", __FUNCTION__, ext_ctrl->value);
			if ((ext_ctrl->value == 1) && (SENSOR_AF_CONTINUOUS != sensor->isp_priv_info.auto_focus)) {
				ret = sensor_af_workqueue_set(icd,0,SENSOR_AF_CONTINUOUS,true);
				//sensor_af_workqueue_set(icd,0,SENSOR_AF_MODE_CLOSE,true);
				//if (sensor_af_workqueue_set(icd,0,SENSOR_AF_CONTINUOUS,true) != 0)
				//	ret = -EAGAIN;
				sensor->isp_priv_info.auto_focus = SENSOR_AF_CONTINUOUS;
			}else if(ext_ctrl->value == 0){
				ret = sensor_af_workqueue_set(icd,0,SENSOR_AF_INFINITY,true);
				//if (sensor_af_workqueue_set(icd,0,SENSOR_AF_INFINITY,true) != 0)
				//	ret = -EAGAIN;;
				sensor->isp_priv_info.auto_focus = SENSOR_AF_INFINITY;
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

				DEBUG_TRACE("%s flash is %x\n",SENSOR_NAME_STRING(), sensor->isp_priv_info.flash);
				break;
			}
#endif
#if CONFIG_SENSOR_FACE_DETECT
	case V4L2_CID_FACEDETECT:
		{
			if(sensor->isp_priv_info.face != ext_ctrl->value){
				if (sensor_set_face_detect(client, ext_ctrl->value) != 0)
					return -EINVAL;
				sensor->isp_priv_info.face = ext_ctrl->value;
				DEBUG_TRACE("%s face value is %x\n",SENSOR_NAME_STRING(), sensor->isp_priv_info.face);
				}
			break;
		}
#endif
#if CONFIG_SENSOR_ISO
	case V4L2_CID_ISO:
	{
		if (sensor->isp_priv_info.iso != ext_ctrl->value) {
			if (sensor_set_iso(icd, qctrl, ext_ctrl->value) != 0) {
				return -EINVAL;
			}
			sensor->isp_priv_info.iso = ext_ctrl->value;
			DEBUG_TRACE("%s set ISO to %d\n", SENSOR_NAME_STRING(), sensor->isp_priv_info.iso);
		}
		break;
	}
#endif
		default:
			break;
	}

	return ret;
}

static int icatch_s_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct isp_dev *sensor = to_sensor(client);

	int i, error_cnt=0, error_idx=-1;
	for (i=0; i<ext_ctrl->count; i++) {
		if(ext_ctrl->controls[i].id == V4L2_CID_FOCUS_AUTO){
			int lx,ty,rx,dy;

			//lx = -y2, ty = x1
			//rx = -y1, dy = x2
			/*
			ty = ext_ctrl->controls[i].rect[0];
			rx = - ext_ctrl->controls[i].rect[1];
			dy = ext_ctrl->controls[i].rect[2];
			lx = - ext_ctrl->controls[i].rect[3];

			lx = ((lx + 1000) * 1024)/2001;
			ty = ((ty + 1000) * 1024)/2001;
			rx = ((rx + 1000) * 1024)/2001;
			dy = ((dy + 1000) * 1024)/2001;

			sensor->isp_priv_info.focus_zone.lx = ty;//1023 - rx;
			sensor->isp_priv_info.focus_zone.rx = dy;//1023 - lx;
			sensor->isp_priv_info.focus_zone.ty = 1023 - rx;//ty;
			sensor->isp_priv_info.focus_zone.dy = 1023 - lx;//dy;
			*/
			lx = ext_ctrl->controls[i].rect[0];
			ty = ext_ctrl->controls[i].rect[1];
			rx = ext_ctrl->controls[i].rect[2];
			dy = ext_ctrl->controls[i].rect[3];

			sensor->isp_priv_info.focus_zone.lx = ((lx + 1000) * 1024)/2001;
			sensor->isp_priv_info.focus_zone.ty = ((ty + 1000) * 1024)/2001;
			sensor->isp_priv_info.focus_zone.rx = ((rx + 1000) * 1024)/2001;
			sensor->isp_priv_info.focus_zone.dy = ((dy + 1000) * 1024)/2001;

			DEBUG_TRACE("%s: lx = %d,ty = %d,rx = %d,dy = %d\n",
					__FUNCTION__,
					sensor->isp_priv_info.focus_zone.lx,
					sensor->isp_priv_info.focus_zone.ty,
					sensor->isp_priv_info.focus_zone.rx,
					sensor->isp_priv_info.focus_zone.dy);
		 }
		if(ext_ctrl->controls[i].id == V4L2_CID_ZOOM_ABSOLUTE){
			DEBUG_TRACE("%s: digtal zoom \n",__func__);
		}

		if (icatch_s_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
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

static int sensor_hdr_exposure(struct i2c_client *client, unsigned int code)
{
	struct isp_dev *sensor = to_sensor(client);
	printk("sensor_hdr_exposure_cb: %d %d\n",code,sensor->isp_priv_info.exposure);
	switch (code)
	{
		case RK_VIDEOBUF_HDR_EXPOSURE_MINUS_1:
		{
			if( (sensor->isp_priv_info.exposure - 1 >= -6) && (sensor->isp_priv_info.exposure - 1 <= 6) ){
				EXISP_I2C_EvSet(6 - sensor->isp_priv_info.exposure - 1);
			}
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_NORMAL:
		{
			EXISP_I2C_EvSet(6 - sensor->isp_priv_info.exposure);
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_PLUS_1:
		{
			if( (sensor->isp_priv_info.exposure + 1 >= -6) && (sensor->isp_priv_info.exposure + 1 <= 6) ){
				EXISP_I2C_EvSet(6 - sensor->isp_priv_info.exposure + 1);
			}
			break;
		}

		case RK_VIDEOBUF_HDR_EXPOSURE_FINISH:
		{
			EXISP_I2C_EvSet(6 - sensor->isp_priv_info.exposure);
			break;
		}
		default:
			break;
	}
	
	return 0;
}

static long icatch_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct sensor *sensor = to_sensor(client);
	int ret = 0;
	switch (cmd){
		case RK29_CAM_SUBDEV_HDR_EXPOSURE:
		{
			sensor_hdr_exposure(client,(unsigned int)arg);
			break;
		}
		
		default:
		{
			//SENSOR_TR("%s %s cmd(0x%x) is unknown !\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}

	return 0;
}

static int sensor_ioctrl(struct soc_camera_device *icd,enum rk29sensor_power_cmd cmd, int on)
{
#if 0
	struct soc_camera_link *icl = to_soc_camera_link(icd);
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
			DEBUG_TRACE("%s %s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_power_end:
	return ret;
#endif
	return 0;
}

static int icatch_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct isp_dev *sensor = to_sensor(client);

	mf->width		= icd->user_width;
	mf->height		= icd->user_height;
	mf->code		= sensor->isp_priv_info.fmt.code;
	mf->colorspace	= sensor->isp_priv_info.fmt.colorspace;
	mf->field		= V4L2_FIELD_NONE;
	return 0;
}

static int icatch_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	 struct i2c_client *client = v4l2_get_subdevdata(sd);
	 const struct sensor_datafmt *fmt;
	 struct isp_dev *sensor = to_sensor(client);
	 //const struct v4l2_queryctrl *qctrl;
	 //struct soc_camera_device *icd = client->dev.platform_data;
	 int ret=0;
	 int set_w,set_h;
	 int supported_size = sensor->isp_priv_info.outputSize;
	 int res_set = 0;
	 u8 preview_cap_mode = 0;//0 is preview mode
	 fmt = sensor_find_datafmt(mf->code, icatch_colour_fmts,
					ARRAY_SIZE(icatch_colour_fmts));
	 if (!fmt) {
		 ret = -EINVAL;
		 goto sensor_s_fmt_end;
	 }
	 set_w = mf->width;
	 set_h = mf->height;
	 if (((set_w <= 176) && (set_h <= 144)) && (supported_size & OUTPUT_QCIF))
	 {
		 set_w = 176;
		 set_h = 144;
		 res_set = OUTPUT_QCIF;
	 }
	 else if (((set_w <= 320) && (set_h <= 240)) && (supported_size & OUTPUT_QVGA))
	 {
		 set_w = 320;
		 set_h = 240;
		 res_set = OUTPUT_QVGA;

	 }
	 else if (((set_w <= 352) && (set_h<= 288)) && (supported_size & OUTPUT_CIF))
	 {
		 set_w = 352;
		 set_h = 288;
		 res_set = OUTPUT_CIF;

	 }
	 else if (((set_w <= 640) && (set_h <= 480)) && (supported_size & OUTPUT_VGA))
	 {
		 set_w = 640;
		 set_h = 480;
		 res_set = OUTPUT_VGA;

	 }
	 else if (((set_w <= 800) && (set_h <= 600)) && (supported_size & OUTPUT_SVGA))
	 {
		 set_w = 800;
		 set_h = 600;
		 res_set = OUTPUT_SVGA;

	 }
	 else if (((set_w <= 1024) && (set_h <= 768)) && (supported_size & OUTPUT_XGA))
	 {
		 set_w = 1024;
		 set_h = 768;
		 res_set = OUTPUT_XGA;

	 }
	 else if (((set_w <= 1280) && (set_h <= 720)) && (supported_size & OUTPUT_720P))
	 {
		 set_w = 1280;
		 set_h = 720;
		 res_set = OUTPUT_720P;
	 }

	 else if (((set_w <= 1280) && (set_h <= 960)) && (supported_size & OUTPUT_QUADVGA))
	 {
		 set_w = 1280;
		 set_h = 960;
		 res_set = OUTPUT_QUADVGA;
	 }
	 else if (((set_w <= 1280) && (set_h <= 1024)) && (supported_size & OUTPUT_XGA))
	 {
		 set_w = 1280;
		 set_h = 1024;
		 res_set = OUTPUT_XGA;
	 }
	 else if (((set_w <= 1600) && (set_h <= 1200)) && (supported_size & OUTPUT_UXGA))
	 {
		 set_w = 1600;
		 set_h = 1200;
		 res_set = OUTPUT_UXGA;
	 }
	 else if (((set_w <= 1920) && (set_h <= 1080)) && (supported_size & OUTPUT_1080P))
	 {
		 set_w = 1920;
		 set_h = 1080;
		 res_set = OUTPUT_1080P;
	 }
	 else if (((set_w <= 2048) && (set_h <= 1536)) && (supported_size & OUTPUT_QXGA))
	 {
		 set_w = 2048;
		 set_h = 1536;
		 res_set = OUTPUT_QXGA;
	 }
	 else if (((set_w <= 2592) && (set_h <= 1944)) && (supported_size & OUTPUT_QSXGA))
	 {
		 set_w = 2592;
		 set_h = 1944;
		 res_set = OUTPUT_QSXGA;
	 }
	 else
	 {
		 set_w = 1280;
		 set_h = 960;
		 res_set = OUTPUT_QUADVGA;
	 }

#if CALIBRATION_MODE_FUN
	 if (g_is_calibrationMode) {
		if (sensor->isp_priv_info.sensor_id == SENSOR_ID_FRONT) {
			set_w = SENSOR_FRONT_MAX_WIDTH;
			set_h = SENSOR_FRONT_MAX_HEIGHT;
			res_set = OUTPUT_QUADVGA;
		} else {
			set_w = SENSOR_REAR_MAX_WIDTH;
			set_h = SENSOR_REAR_MAX_HEIGHT;
			if (res_set == OUTPUT_QSXGA) {
				sensor->isp_priv_info.rk_query_PreviewCapMode = CAPTURE_NONE_ZSL_MODE;
			} else {
				sensor->isp_priv_info.rk_query_PreviewCapMode = PREVIEW_MODE;
			}
			res_set = OUTPUT_QSXGA;
			DEBUG_TRACE("--------------%s: Prev/CapMode:%d\n",__func__, sensor->isp_priv_info.rk_query_PreviewCapMode);
		}
	 }
#endif

	 //if(res_set != sensor->isp_priv_info.curRes)
	 //  sensor_set_isp_output_res(client,res_set);
	 //res will be setted
	 sensor->isp_priv_info.curRes = res_set;
	 mf->width = set_w;
	 mf->height = set_h;
	 //enter capture or preview mode
	 //EXISP_I2C_PvCapModeSet(preview_cap_mode);
	 DEBUG_TRACE("%s:setw = %d,seth = %d\n",__func__,set_w,set_h);

 sensor_s_fmt_end:
	 return ret;
 }

static int icatch_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	int err = 0,i=0,num = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct isp_dev *sensor = to_sensor(client);
	

	//get supported framesize num

	
	if (fsize->index >= sensor->isp_priv_info.supportedSizeNum) {
		err = -1;
		goto end;
	}

	if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_QCIF))
	{
		
		fsize->discrete.width = 176;
		fsize->discrete.height = 144;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_QVGA))
	{
		fsize->discrete.width = 320;
		fsize->discrete.height = 240;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_CIF) )
	{
	
		fsize->discrete.width = 352;
		fsize->discrete.height = 288;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_VGA))
	{
	
		fsize->discrete.width = 640;
		fsize->discrete.height = 480;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_SVGA))
	{
	
		fsize->discrete.width = 800;
		fsize->discrete.height = 600;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_XGA))
	{
	
		fsize->discrete.width = 1024;
		fsize->discrete.height = 768;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_720P))
	{
	
		fsize->discrete.width = 1280;
		fsize->discrete.height = 720;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_QUADVGA))
	{
		fsize->discrete.width = 1280;
		fsize->discrete.height = 960;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_XGA))
	{
		fsize->discrete.width = 1280;
		fsize->discrete.height = 1024;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_UXGA) )
	{
		fsize->discrete.width = 1600;
		fsize->discrete.height = 1200;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_1080P))
	{
		fsize->discrete.width = 1920;
		fsize->discrete.height = 1080;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_QXGA))
	{
		fsize->discrete.width = 2048;
		fsize->discrete.height = 1536;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	}
	else if ((sensor->isp_priv_info.supportedSize[fsize->index] & OUTPUT_QSXGA))
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

static int icatch_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct isp_dev *sensor = to_sensor(client);
	u8 sensorid = 0;
	__u32 smaxh, sminh, smaxw, sminw;
	const struct sensor_datafmt *fmt;
	int ret = 0,set_w,set_h;
	int supported_size = sensor->isp_priv_info.outputSize;

	sensorid = strcmp(icl->board_info->type, SENSOR_REAR_NAME_STRING());
	if (sensorid) sensorid = 1;
	if(sensorid){
		//front camera
		smaxh = SENSOR_FRONT_MAX_HEIGHT;
		sminh = SENSOR_FRONT_MIN_HEIGHT;
		smaxw = SENSOR_FRONT_MAX_WIDTH;
		sminw = SENSOR_FRONT_MIN_WIDTH;
	}
	else{
		//rear camera
		smaxh = SENSOR_REAR_MAX_HEIGHT;
		sminh = SENSOR_REAR_MIN_HEIGHT;
		smaxw = SENSOR_REAR_MAX_WIDTH;
		sminw = SENSOR_REAR_MIN_WIDTH;
	}

	fmt = sensor_find_datafmt(mf->code, icatch_colour_fmts,
				   ARRAY_SIZE(icatch_colour_fmts));

	if (fmt == NULL) {
		fmt = &sensor->isp_priv_info.fmt;
		mf->code = fmt->code;
	}
	if (mf->height > smaxh)
		mf->height = smaxh;
	else if (mf->height < sminh)
		mf->height = sminh;

	if (mf->width > smaxw)
		mf->width = smaxw;
	else if (mf->width < sminw)
		mf->width = sminw;

	set_w = mf->width;
	set_h = mf->height;

	if (((set_w <= 176) && (set_h <= 144)) && (supported_size & OUTPUT_QCIF))
	{
		set_w = 176;
		set_h = 144;
	}
	else if (((set_w <= 320) && (set_h <= 240)) && (supported_size & OUTPUT_QVGA))
	{
		set_w = 320;
		set_h = 240;
	}
	else if (((set_w <= 352) && (set_h<= 288)) && (supported_size & OUTPUT_CIF))
	{
		set_w = 352;
		set_h = 288;
	}
	else if (((set_w <= 640) && (set_h <= 480)) && (supported_size & OUTPUT_VGA))
	{
		set_w = 640;
		set_h = 480;
	}
	else if (((set_w <= 800) && (set_h <= 600)) && (supported_size & OUTPUT_SVGA))
	{
		set_w = 800;
		set_h = 600;
	}
	else if (((set_w <= 1024) && (set_h <= 768)) && (supported_size & OUTPUT_XGA))
	{
		set_w = 1024;
		set_h = 768;
	}
	else if (((set_w <= 1280) && (set_h <= 720)) && (supported_size & OUTPUT_720P))
	{
		set_w = 1280;
		set_h = 720;
	}
	else if (((set_w <= 1280) && (set_h <= 960)) && (supported_size & OUTPUT_QUADVGA))
	{
		set_w = 1280;
		set_h = 960;
	}
	else if (((set_w <= 1280) && (set_h <= 1024)) && (supported_size & OUTPUT_XGA))
	{
		set_w = 1280;
		set_h = 1024;
	}
	else if (((set_w <= 1600) && (set_h <= 1200)) && (supported_size & OUTPUT_UXGA))
	{
		set_w = 1600;
		set_h = 1200;
	}
	else if (((set_w <= 1920) && (set_h <= 1080)) && (supported_size & OUTPUT_1080P))
	{
		set_w = 1920;
		set_h = 1080;
	}
	else if (((set_w <= 2048) && (set_h <= 1536)) && (supported_size & OUTPUT_QXGA))
	{
		set_w = 2048;
		set_h = 1536;
	}
	else if (((set_w <= 2592) && (set_h <= 1944)) && (supported_size & OUTPUT_QSXGA))
	{
		set_w = 2592;
		set_h = 1944;
	}
	else
	{
		if(sensorid){
			//front camera
			set_w = SENSOR_FRONT_INIT_WIDTH;
			set_h = SENSOR_FRONT_INIT_HEIGHT;
		}
		else{
			//rear camera
			set_w = SENSOR_REAR_INIT_WIDTH;
			set_h = SENSOR_REAR_INIT_HEIGHT;
		}

	}

#if CALIBRATION_MODE_FUN
	if (g_is_calibrationMode == 1) {
		if (sensorid == SENSOR_ID_FRONT) {
			set_w = SENSOR_FRONT_MAX_WIDTH;
			set_h = SENSOR_FRONT_MAX_HEIGHT;
		} else {
			set_w = SENSOR_REAR_MAX_WIDTH;
			set_h = SENSOR_REAR_MAX_HEIGHT;
		}
	}
#endif

	mf->width = set_w;
	mf->height = set_h;
	mf->colorspace = fmt->colorspace;
	return ret;
}

static int icatch_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(icatch_colour_fmts))
		return -EINVAL;

	*code = icatch_colour_fmts[index].code;
	return 0;

}
/*-------------------------------------------------------------------------
 *	Function Name : EXISP_I2C_PvSizeSet
 *	ucParam:
	 0x00	1280x960
	 0x01	3264x2448
	 0x02	1920x1080
	 0x03	320x240(reserved)
	 0x04	1280x720
	 0x05	1040x780
	 0x06	2080x1560
	 0x07	3648x2736
	 0x08	4160x3120
	 0x09	3360x1890
	 0x0A	2592x1944
	 0x0B	640x480
	 0x0C	1408x1408
	 0x0D	1920x1088
 *	Return : None
 *------------------------------------------------------------------------*/

static int icatch_set_isp_output_res(struct i2c_client *client,enum ISP_OUTPUT_RES outputSize){
	u8 check_val = 0;
	struct isp_dev *sensor = to_sensor(client);
	u8 res_sel = 0;
	switch(outputSize) {
	case OUTPUT_QCIF:
	case OUTPUT_HQVGA:
	case OUTPUT_QVGA:
	case OUTPUT_CIF:
	case OUTPUT_VGA:
	case OUTPUT_SVGA:
	case OUTPUT_720P:
	case OUTPUT_QUADVGA:
		res_sel = 0;
		break;
	case OUTPUT_XGA:
	case OUTPUT_SXGA:
	case OUTPUT_UXGA:
	case OUTPUT_1080P:
	case OUTPUT_QXGA:
	case OUTPUT_QSXGA:
		res_sel = IMAGE_CAP_NONZSL_SINGLE;// non-zsl single
		break;
	default:
		DEBUG_TRACE("%s %s  isp not support this resolution!\n",SENSOR_NAME_STRING(),__FUNCTION__);
		break;
	}
	//preview mode set
	if(outputSize == OUTPUT_QSXGA){
		if (sensor->isp_priv_info.had_setprvsize == 0) {
			sensor->isp_priv_info.had_setprvsize = 1;
			EXISP_PvSizeSet(res_sel);
		}
		icatch_sensor_write(SP7K_RDREG_INT_STS_REG_0,icatch_sensor_read(SP7K_RDREG_INT_STS_REG_0));
		EXISP_ImageCapSet(res_sel);
		while((icatch_sensor_read(SP7K_RDREG_INT_STS_REG_0) & 0x04) == 0){
			msleep(10);
		}
		DEBUG_TRACE("%s:%d,wait capture out\n",__func__,__LINE__);
		icatch_sensor_write(SP7K_RDREG_INT_STS_REG_0,(icatch_sensor_read(SP7K_RDREG_INT_STS_REG_0)|0x04));
		sensor->isp_priv_info.curPreviewCapMode = CAPTURE_NONE_ZSL_MODE;
	}
	else{
		sensor->isp_priv_info.had_setprvsize = 1;
		EXISP_PvSizeSet(res_sel);
		sensor->isp_priv_info.curPreviewCapMode = PREVIEW_MODE;
#if 1
		DEBUG_TRACE("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), EXISP_I2C_VendorIdGet);
		DEBUG_TRACE("fw version is 0x%x\n ",EXISP_I2C_FWVersionGet());

		DEBUG_TRACE("front id= 0x%x,rear id = 0x%x\n ",EXISP_I2C_FrontSensorIdGet(),EXISP_I2C_RearSensorIdGet());
#endif
	}
	return 0;
}

static int icatch_s_stream(struct v4l2_subdev *sd, int enable){
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct isp_dev *sensor = to_sensor(client);
	DEBUG_TRACE("%s:enable = %d\n",__func__,enable);
	DEBUG_TRACE("@@NY@@%s: client %0x\n", __FUNCTION__, client);
	if(enable == 0){
		//   sensor_set_face_detect(client,0);
		//  sensor_af_workqueue_set(icd,0,0,true);
#if CALIBRATION_MODE_FUN
		if (g_is_calibrationMode)
			EXISP_I2C_PvStreamSet(0); // stream off
#endif
		sensor->isp_priv_info.curPreviewCapMode = IDLE_MODE;
	}else{
#if CALIBRATION_MODE_FUN
		if (g_is_calibrationMode &&
		    sensor->isp_priv_info.sensor_id ==  SENSOR_ID_REAR_CALIBRATION &&
		    sensor->isp_priv_info.rk_query_PreviewCapMode == PREVIEW_MODE) {
			EXISP_PvSizeSet(0x0A);
			sensor->isp_priv_info.curPreviewCapMode = PREVIEW_MODE;
			EXISP_I2C_PvStreamSet(1); // stream on
			return 0;
		}
#endif
		printk(KERN_ERR"icatch_set_isp_output_res: curRes(%x)\n",sensor->isp_priv_info.curRes);
		//sensor_set_face_detect(client,1);
		icatch_set_isp_output_res(client, sensor->isp_priv_info.curRes);


#if CALIBRATION_MODE_FUN
		if (g_is_calibrationMode)
			EXISP_I2C_PvStreamSet(1); // stream on
#endif
	}

	 return 0;
}
static struct v4l2_subdev_core_ops isp_subdev_core_ops = {
	.init			= icatch_sensor_init,
	.g_ctrl			= icatch_g_control,
	.s_ctrl			= icatch_s_control,
	.g_ext_ctrls	= icatch_g_ext_controls,
	.s_ext_ctrls	= icatch_s_ext_controls,
//	.g_chip_ident	= sensor_g_chip_ident,
	.ioctl			= icatch_ioctl,
};
static struct v4l2_subdev_video_ops isp_subdev_video_ops = {
	.s_mbus_fmt		= icatch_s_fmt,
	.g_mbus_fmt		= icatch_g_fmt,
	.try_mbus_fmt	= icatch_try_fmt,
	.enum_mbus_fmt	= icatch_enum_fmt,
	.s_stream		= icatch_s_stream,
	.enum_framesizes = icatch_enum_framesizes,
	//.g_face_area = sensor_g_face_area,
};

const struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &isp_subdev_core_ops,
	.video = &isp_subdev_video_ops,
};

struct isp_dev* to_sensor(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct isp_dev, subdev);
}

int icatch_sensor_write( u16 reg, u8 val)
{
	int err,cnt;
	u8 buf[3];
	struct i2c_msg msg[1];
	struct i2c_client* client = g_icatch_i2c_client;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	buf[2] = val;

	msg->addr = client->addr;
	msg->flags = client->flags;
	msg->buf = buf;
	msg->len = sizeof(buf);
	msg->scl_rate = CONFIG_SENSOR_I2C_SPEED;		 /* ddl@rock-chips.com : 100kHz */
	msg->read_type = 0; 			  /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */

	cnt = 3;
	err = -EAGAIN;
	while ((cnt-- > 0) && (err < 0)) {						 /* ddl@rock-chips.com :  Transfer again if transent is failed	 */
		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		} else {
			DEBUG_TRACE("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n",SENSOR_NAME_STRING(),reg, val);
			udelay(10);
		}
	}

	return err;
}

/* sensor register read */
u8 icatch_sensor_read( u16 reg)
{
	int err,cnt;
	u8 buf[2];
	struct i2c_msg msg[2];
	struct i2c_client* client =g_icatch_i2c_client;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);
	msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;		 /* ddl@rock-chips.com : 100kHz */
	msg[0].read_type = 2;	/* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

	msg[1].addr = client->addr;
	msg[1].flags = client->flags|I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;
	msg[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;						 /* ddl@rock-chips.com : 100kHz */
	msg[1].read_type = 2;							  /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

	cnt = 3;
	err = -EAGAIN;
	while ((cnt-- > 0) && (err < 0)) {						 /* ddl@rock-chips.com :  Transfer again if transent is failed	 */
		err = i2c_transfer(client->adapter, &msg[0], 1);

		if (err >= 0) {
			break;
		} else {
			DEBUG_TRACE("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n",SENSOR_NAME_STRING(),reg, buf[0]);
			udelay(10);
		}
	}

	if(err <0)
		return FAIL;

	cnt = 3;
	err = -EAGAIN;
	while ((cnt-- > 0) && (err < 0)) {						 /* ddl@rock-chips.com :  Transfer again if transent is failed	 */
		err = i2c_transfer(client->adapter, &msg[1], 1);

		if (err >= 0) {
			break;
		} else {
			DEBUG_TRACE("\n %s read reg(0x%x val:0x%x) failed, try to read again! \n",SENSOR_NAME_STRING(),reg, buf[0]);
			udelay(10);
		}
	}
	if(err >=0)
		return buf[0];
	else
		return FAIL;
}

/* write a array of registers  */
int icatch_sensor_write_array(void *regarrayv)
{
	int err = 0, cnt;
	int i = 0;
#if CONFIG_SENSOR_I2C_RDWRCHK
	char valchk;
#endif
	struct i2c_client* client = g_icatch_i2c_client;
	struct reginfo * regarray = (struct reginfo *)regarrayv;

	DEBUG_TRACE("@@NY@@%s:\n", __FUNCTION__);
	cnt = 0;
	while (regarray[i].reg != 0)
	{
		err = icatch_sensor_write( regarray[i].reg, regarray[i].val);
		if (err < 0)
		{
			if (cnt-- > 0) {
				DEBUG_TRACE("%s..write failed current reg:0x%x, Write array again !\n", SENSOR_NAME_STRING(),regarray[i].reg);
				i = 0;
				continue;
			} else {
				DEBUG_TRACE("%s..write array failed!!!\n", SENSOR_NAME_STRING());
				err = -EPERM;
				goto sensor_write_array_end;
			}
		} else {
		#if CONFIG_SENSOR_I2C_RDWRCHK
			icatch_sensor_read(regarray[i].reg, &valchk);
			if (valchk != regarray[i].val)
				DEBUG_TRACE("%s Reg:0x%x write(0x%x, 0x%x) fail\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);
		#endif
		}
		i++;
	}

sensor_write_array_end:
	return err;
}

int icatch_request_firmware(const struct firmware ** fw){
	int ret = 0;
	if( request_firmware(fw, ICATCHFWNAME, &g_icatch_i2c_client->dev) !=0){
		DEBUG_TRACE("%s:%d, request firmware erro,please check firmware!\n");
		ret = -1;
	}else{
		ret = 0;
	}
	return ret;
}

void icatch_release_firmware(const struct firmware * fw){
	if(fw)
		release_firmware(fw);
}

void icatch_sensor_power_ctr(struct soc_camera_device *icd ,int on,int power_mode){
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct rk29camera_platform_data* pdata = (struct rk29camera_platform_data*)(to_soc_camera_host(icd->dev.parent)->v4l2_dev.dev->platform_data);
	if(!on){
		//power down
		if(icl->power)
			icl->power(icd->pdev,0);
		if(pdata && pdata->rk_host_mclk_ctrl)
			pdata->rk_host_mclk_ctrl(icd,0);
		if(icl->powerdown)
			icl->powerdown(icd->pdev,0);
		iomux_set(GPIO1_A4);
		iomux_set(GPIO1_A5) ;
		iomux_set(GPIO1_A6);
		iomux_set(GPIO1_A7);
		iomux_set(GPIO3_B6);
		iomux_set(GPIO3_B7);
		iomux_set(GPIO0_C0);
		gpio_set_value(RK30_PIN1_PA4,0);
		gpio_set_value(RK30_PIN1_PA5,0);
		gpio_set_value(RK30_PIN1_PA6,0);	//Vincent_Liu@asus.com for clk 24M
		gpio_set_value(RK30_PIN1_PA7,0);
		gpio_set_value(RK30_PIN3_PB6,0);
		gpio_set_value(RK30_PIN3_PB7,0);
		gpio_set_value(RK30_PIN0_PC0,0);
		//msleep(500);
	}else{
		//power ON
		gpio_set_value(RK30_PIN1_PA6,1);	//Vincent_Liu@asus.com for clk 24M
		
		if(icl->power)
			icl->power(icd->pdev,1);
		if(icl->powerdown){
			if(power_mode == 0)//from spi
				icl->powerdown(icd->pdev,1);
			else
				icl->powerdown(icd->pdev,0);
			}
		if(pdata && pdata->rk_host_mclk_ctrl)
			pdata->rk_host_mclk_ctrl(icd,1);
			//reset , reset pin low ,then high
		if (icl->reset)
			icl->reset(icd->pdev);
		if(power_mode == 0)//from spi
			icl->powerdown(icd->pdev,0);
		iomux_set(SPI0_CLK);
		iomux_set(SPI0_RXD);
		iomux_set(SPI0_TXD);
		iomux_set(SPI0_CS0);
		iomux_set(I2C3_SDA);
		iomux_set(I2C3_SCL);
		msleep(100);
	}
}

#if CALIBRATION_MODE_FUN
#if 0
void __dump(const u8 *data, size_t size) {
	size_t i = 0;
	char buf[100] = {0};
	char lbuf[12];

	while( i < size) {
		if ((i%16 == 0) && (i != 0)) {
			DEBUG_TRACE("%s\n", buf);
		}
		if (i%16 == 0) {
			buf[0] = 0;
			sprintf(lbuf, "%08X:", i);
			strcat(buf, lbuf);
		}
		sprintf(lbuf, " %02X", *(data + i));
		strcat(buf, lbuf);
		i++;
	}

	DEBUG_TRACE("%s\n", buf);
}
#endif

struct icatch_cali_fw_data * icatch_load_cali_fw_data(u8 sensorid) {

	if (g_is_calibrationMode) {
		return NULL;
	}

	struct icatch_cali_fw_data * fw = NULL;
	switch (sensorid) {
	case SENSOR_ID_FRONT:
		fw = &g_cali_fw_data_front;
		break;
	case SENSOR_ID_REAR:
		fw = &g_cali_fw_data_back;
		break;
	default:
		break;
	}
	if (fw == NULL)
		return NULL;

	if (request_firmware(
		&(fw->fw_option),
		fw->fname_option,
		&g_icatch_i2c_client->dev) != 0) {
		DEBUG_TRACE("%s: load calibration fw data: %s fail", __FUNCTION__, fw->fname_option);
		fw->fw_option = NULL;
	}

	if (request_firmware(
		&(fw->fw_3acali),
		fw->fname_3acali,
		&g_icatch_i2c_client->dev) != 0) {
		DEBUG_TRACE("%s: load calibration fw data: %s fail", __FUNCTION__, fw->fname_3acali);
		fw->fw_3acali = NULL;
	}
#if 0
	else {
		DEBUG_TRACE("%s: dump %s (size: %d)\n", __FUNCTION__, fw->fname_3acali, fw->fw_3acali->size);
		__dump(fw->fw_3acali->data, fw->fw_3acali->size);
	}
#endif

	if (request_firmware(
		&(fw->fw_lsc),
		fw->fname_lsc,
		&g_icatch_i2c_client->dev) != 0) {
		DEBUG_TRACE("%s: load calibration fw data: %s fail", __FUNCTION__, fw->fname_lsc);
		fw->fw_lsc = NULL;
	}
#if 0
	else {
		DEBUG_TRACE("%s: dump %s (size: %d)\n", __FUNCTION__, fw->fname_lsc, fw->fw_lsc->size);
		__dump(fw->fw_lsc->data, fw->fw_lsc->size);
	}
#endif

	if (request_firmware(
		&(fw->fw_lscdq),
		fw->fname_lscdq,
		&g_icatch_i2c_client->dev) != 0) {
		DEBUG_TRACE("%s: load calibration fw data: %s fail", __FUNCTION__, fw->fname_lscdq);
		fw->fw_lscdq = NULL;
	}
#if 0
	else {
		DEBUG_TRACE("%s: dump %s (size: %d)\n", __FUNCTION__, fw->fname_lscdq, fw->fw_lscdq->size);
		__dump(fw->fw_lscdq->data, fw->fw_lscdq->size);
	}
#endif

	return fw;
}

void icatch_free_cali_fw_data(struct icatch_cali_fw_data * data) {
	if (data == NULL)
		return ;

	if (data->fw_option != NULL) {
		release_firmware(data->fw_option);
	}

	if (data->fw_3acali != NULL) {
		release_firmware(data->fw_3acali);
	}

	if (data->fw_lsc != NULL) {
		release_firmware(data->fw_lsc);
	}

	if (data->fw_lscdq != NULL) {
		release_firmware(data->fw_lscdq);
	}
}
#endif

//#include "BOOT_OV5693_126MHz(075529).c"
 int icatch_load_fw(struct soc_camera_device *icd,u8 sensorid){
	 struct firmware *fw =NULL;
#if CALIBRATION_MODE_FUN
	 struct icatch_cali_fw_data * cali_data = NULL;
#endif
	 int ret = 0;
	 icatch_sensor_power_ctr(icd,0,ICATCH_BOOT);
	 icatch_sensor_power_ctr(icd,1,ICATCH_BOOT);
	 if(ICATCH_BOOT == ICATCH_BOOT_FROM_HOST){
		 DEBUG_TRACE("@@NY@@%s: %d\n", __FUNCTION__, sensorid);
		 if(icatch_request_firmware(&fw)!= 0){
			 ret = -1;
			 goto icatch_load_fw_out;
		 }

#if CALIBRATION_MODE_FUN
		 cali_data = icatch_load_cali_fw_data(sensorid);
		 if (cali_data != NULL) {
			 DEBUG_TRACE("%s:%d,load calibration fw data success !!!!\n",__func__,__LINE__);
			 ret = EXISP_LoadCodeStart(
					ICATCH_BOOT,
					sensorid,
					g_is_calibrationMode,
					(u8*)(fw->data),
					cali_data->fw_option ? (u8*)(cali_data->fw_option->data) : &g_Calibration_Option_Def,
					cali_data->fw_3acali ? (u8*)(cali_data->fw_3acali->data) : NULL,
					cali_data->fw_lsc ? (u8*)(cali_data->fw_lsc->data) : NULL,
					cali_data->fw_lscdq ? (u8*)(cali_data->fw_lscdq->data) : NULL);
		 } else {
			 DEBUG_TRACE("%s:%d,load calibration fw data fail !!!!\n",__func__,__LINE__);
			 ret = EXISP_LoadCodeStart(
					ICATCH_BOOT,
					sensorid,
					g_is_calibrationMode,
					(u8*)(fw->data),
					&g_Calibration_Option_Def,
					NULL,
					NULL,
					NULL);
		 }
#else
		 ret = EXISP_LoadCodeStart(
				ICATCH_BOOT,
				sensorid,
				0,
				(u8*)(fw->data),
				&g_Calibration_Option_Def,
				NULL,
				NULL,
				NULL);
#endif

		 if (ret != SUCCESS) {
			 DEBUG_TRACE("%s:%d,load firmware failed !!!!\n",__func__,__LINE__);
			 ret = -1;
		 } else {
			 ret = 0;
		 }

		 icatch_release_firmware(fw);
		 icatch_free_cali_fw_data(cali_data);

		 if(ret < 0)
			 goto icatch_load_fw_out;

	 }else{
	#if 1
		 BB_WrSPIFlash(0xffffff);
		 
		 icatch_sensor_power_ctr(icd,0,0);
		 icatch_sensor_power_ctr(icd,1,0);
		 gpio_set_value(RK30_PIN1_PA6,0);	//Vincent_Liu@asus.com for clk 24M
	#endif
	}
	// msleep(100);
	 return 0;
icatch_load_fw_out:
	return ret;
}
 
const struct sensor_datafmt *sensor_find_datafmt(
	 enum v4l2_mbus_pixelcode code, const struct sensor_datafmt *fmt,
	 int n)
 {
	 int i;
	 for (i = 0; i < n; i++)
		 if (fmt[i].code == code)
			 return fmt + i;
 
	 return NULL;
 }

int icatch_get_rearid_by_lowlevelmode(struct soc_camera_device *icd,UINT16 *rear_id)
{
	static int ret = 0;
	static int done = 0;
	static UINT16 _rear_id = 0;

	static const struct reginfo reset_1[]={
		{0x1011, 0x01},	/* CPU suspend */
		{0x0084, 0x14},  /* To sensor clock divider */
		{0x0034, 0xFF},  /* Turn on all clock */
		{0x9032, 0x00},
		{0x9033, 0x10},
		{0x9030, 0x3f},
		{0x9031, 0x04},
		{0x9034, 0xf2},
		{0x9035, 0x04},
		{0x9032, 0x10},
		{0x00,  0x00},
	};

	//tmrUsWait(10000); /*10ms*/

	static const struct reginfo reset_2[] = {
		{0x9032, 0x30},
		{0x00,  0x00},
	};

	//tmrUsWait(10000); /*10ms*/


	static const struct reginfo reset_3[] = {
		/*End - Power on sensor & enable clock */
		{0x9008, 0x00},
		{0x9009, 0x00},
		{0x900A, 0x00},
		{0x900B, 0x00},

		/*Start - I2C Read*/
		{0x923C, 0x01},  /* Sub address enable */
		{0x9238, 0x30},  /* Sub address enable */
		{0x9240, 0x6C},  /* Slave address      */
		{0x9200, 0x03},  /* Read mode          */
		{0x9210, 0x00},  /* Register addr MSB  */
		{0x9212, 0x00},  /* Register addr LSB  */
		{0x9204, 0x01},  /* Trigger I2C read   */
		{0x00,  0x00},
	};

	//	tmrUsWait(2000);/*2ms*/

	DEBUG_TRACE("%s: entry!\n", __FUNCTION__);

	if (done == 1) {
		if (rear_id != NULL)
			*rear_id = _rear_id;
		return ret;
	}

	icatch_sensor_power_ctr(icd,0,ICATCH_BOOT);
	icatch_sensor_power_ctr(icd,1,ICATCH_BOOT);

	if (icatch_sensor_write_array(reset_1) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}
	msleep(10);

	if (icatch_sensor_write_array(reset_2) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(10);
	if (icatch_sensor_write_array(reset_3) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(2);
	_rear_id = (UINT16)icatch_sensor_read(0x9211) << 8;
	_rear_id += icatch_sensor_read(0x9213);
	DEBUG_TRACE("%s: rear_id = 0x%04X\n", __FUNCTION__, _rear_id);
	*rear_id = _rear_id;


l_ret:
	icatch_sensor_power_ctr(icd,0,ICATCH_BOOT);
	done = 1;
	return ret;
}


int icatch_get_frontid_by_lowlevelmode(struct soc_camera_device *icd,UINT16 *front_id)
{
	static int ret = 0;
	static int done = 0;
	static UINT16 _front_id = 0;

	static const struct reginfo reset_1[]={
		{ 0x1011, 0x01},  /* CPU Suspend */

		{ 0x0084, 0x14},  /* To sensor clock divider */
		{ 0x0034, 0xFF},  /* Turn on all clock */
		{ 0x9030, 0x3f},
		{ 0x9031, 0x04},
		{ 0x9034, 0xf2},
		{ 0x9035, 0x04},
		{ 0x9033, 0x04},
		{ 0x9032, 0x3c},
		{0x00,  0x00},
	};

	//tmrUsWait(10000); /* 10ms */

	static const struct reginfo reset_2[] = {
		{0x9033, 0x00},
		{0x00,  0x00},
	};

	//tmrUsWait(10000); /*10ms*/


	static const struct reginfo reset_3[] = {
		{ 0x9033, 0x04},
		{ 0x9032, 0x3e},
		{0x00,  0x00},
	};

	//tmrUsWait(10000); /* 10ms */

	static const struct reginfo reset_4[] = {
		{ 0x9032, 0x3c },
		/*End - Power on sensor & enable clock */

		/*Start - I2C Read ID*/
		{ 0x9138, 0x30},  /* Sub address enable */
		{ 0x9140, 0x90},  /* Slave address      */
		{ 0x9100, 0x03},  /* Read mode          */
		{ 0x9110, 0x00},  /* Register addr MSB  */
		{ 0x9112, 0x00}, /* Register addr LSB  */
		{ 0x9104, 0x01 }, /* Trigger I2C read   */
		{0x00,  0x00},
	};

	//tmrUsWait(100);   /* 0.1ms */

	static const struct reginfo reset_5[] = {
		{ 0x9110, 0x00},  /* Register addr MSB  */
		{ 0x9112, 0x01},  /* Register addr LSB  */
		{ 0x9104, 0x01},  /* Trigger I2C read   */
		{0x00,  0x00},
	};

	DEBUG_TRACE("%s: entry!\n", __FUNCTION__);

	if (done == 1) {
		if (front_id != NULL)
			*front_id = _front_id;
		return ret;
	}

	icatch_sensor_power_ctr(icd,0,ICATCH_BOOT);
	icatch_sensor_power_ctr(icd,1,ICATCH_BOOT);

	if (icatch_sensor_write_array(reset_1) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}
	msleep(10);

	if (icatch_sensor_write_array(reset_2) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(10);
	if (icatch_sensor_write_array(reset_3) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(10);


	if (icatch_sensor_write_array(reset_4) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(1);
	_front_id = (UINT16)icatch_sensor_read(0x9111) << 8;

	if (icatch_sensor_write_array(reset_5) != 0) {
		ret = -ENXIO;
		goto l_ret;
	}

	msleep(1);
	_front_id += icatch_sensor_read(0x9111);
	DEBUG_TRACE("%s: front_id = 0x%04X\n", __FUNCTION__, _front_id);
	*front_id = _front_id;

l_ret:
	icatch_sensor_power_ctr(icd,0,ICATCH_BOOT);
	done = 1;
	return ret;
}
