#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <plat/rk_camera.h>
//#include "app_i2c_lib_icatch.h"
#include <mach/iomux.h>
#include "icatch7002_common.h"

#ifdef CONFIG_SENSOR_Focus
#undef CONFIG_SENSOR_Focus
#endif
#define CONFIG_SENSOR_Focus 0

#define ASUS_CAMERA_SUPPORT 1

//module_param(0, int, S_IRUGO|S_IWUSR);
static const struct v4l2_querymenu mi1040_sensor_menus[] =
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

static	struct v4l2_queryctrl mi1040_sensor_controls[] =
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

static int mi1040_before_init_cb(const struct i2c_client *client)
{
	return icatch_update_sensor_ops(
			mi1040_sensor_controls,
			ARRAY_SIZE(mi1040_sensor_controls),
			mi1040_sensor_menus,
			ARRAY_SIZE(mi1040_sensor_menus));
}

static int mi1040_ensor_probe(struct i2c_client *client,
	const struct i2c_device_id *did)
{
	struct isp_dev *sensor;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	struct i2c_board_info *icb;
	UINT16 sensorid = 0;
//	int ret;
	
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
	icb = icl->board_info;
	if(icb){
		DEBUG_TRACE("@@NY@@i2c name[%s] adapter id[%d]\n", icb->type, icl->i2c_adapter_id);
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
		return -EIO;
	}
	
	sensor = kzalloc(sizeof(struct isp_dev), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(struct isp_dev));
	g_icatch_i2c_client = client;
	sensor->before_init_cb = mi1040_before_init_cb;
	if ((icatch_get_frontid_by_lowlevelmode(icd, &sensorid) != 0) || sensorid != 0x2481) {
		kfree(sensor);
		DEBUG_TRACE("%s: get sensor id fail!\n", __FUNCTION__);
		return -ENXIO;
	}

	v4l2_i2c_subdev_init(&sensor->subdev, client, &sensor_subdev_ops);
	
#if CALIBRATION_MODE_FUN
	icatch_create_proc_entry();
#endif

	/* Second stage probe - when a capture adapter is there */
	icd->ops		= &sensor_ops;
	sensor->isp_priv_info.fmt = icatch_colour_fmts[0];
	

//	ret = sensor_video_probe(icd, client);
#if CONFIG_SENSOR_Focus
	sensor->sensor_wq = create_workqueue(SENSOR_NAME_STRING(_af_workqueue));
	if (sensor->sensor_wq == NULL)
		DEBUG_TRACE("%s create fail!", SENSOR_NAME_STRING(_af_workqueue));
	mutex_init(&sensor->wq_lock);

	sensor->isp_priv_info.focus_zone.lx = 256;
	sensor->isp_priv_info.focus_zone.rx = 768;
	sensor->isp_priv_info.focus_zone.ty = 256;
	sensor->isp_priv_info.focus_zone.dy = 768;
#endif

	mutex_init(&sensor->isp_priv_info.access_data_lock);
	return 0;
}

static int mi1040_sensor_remove(struct i2c_client *client)
{
	struct isp_dev *sensor = to_sensor(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	#if CONFIG_SENSOR_Focus
	if (sensor->sensor_wq) {
		destroy_workqueue(sensor->sensor_wq);
		sensor->sensor_wq = NULL;
	}
	#endif

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(sensor);
	sensor = NULL;
	g_icatch_i2c_client = NULL;


#if CALIBRATION_MODE_FUN
	icatch_remove_proc_entry();
#endif

	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_FRONT_NAME_STRING(), 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);
 
static struct i2c_driver sensor_i2c_driver = {
	.driver = {
			.name = SENSOR_FRONT_NAME_STRING(),
	},
	.probe  = mi1040_ensor_probe,
	.remove  = mi1040_sensor_remove,
	.id_table = sensor_id,
};
 
static int __init sensor_mod_init(void)
{
	DEBUG_TRACE("\n%s..%s.. \n",__FUNCTION__,SENSOR_FRONT_NAME_STRING());
	return i2c_add_driver(&sensor_i2c_driver);
}
 
static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sensor_i2c_driver);
}
 
module_init(sensor_mod_init);
module_exit(sensor_mod_exit);
 
MODULE_DESCRIPTION(SENSOR_FRONT_NAME_STRING(Camera sensor driver));
MODULE_AUTHOR("ddl <kernel@rock-chips>");
MODULE_LICENSE("GPL");

