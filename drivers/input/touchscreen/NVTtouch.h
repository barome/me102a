/*
 * include/linux/NVTtouch_touch.h
 *
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>

//*************************TouchScreen Work Part*****************************

#define NVTTOUCH_I2C_NAME "NVT-ts"
//define default resolution of the touchscreen
#define TOUCH_MAX_HEIGHT 	800		
#define TOUCH_MAX_WIDTH		1280

//set i2c Bus number
#define NVT_BUS_NUM 0
#define NVT_I2C_SCLK 400*1000

//set trigger mode
#define INT_TRIGGER		1   //IRQ_TYPE_EDGE_FALLING

#define POLL_TIME		10	//actual query spacing interval:POLL_TIME+6

#define NVTTOUCH_MULTI_TOUCH
#ifdef NVTTOUCH_MULTI_TOUCH
	#define MAX_FINGER_NUM	10  
#else
	#define MAX_FINGER_NUM	1	
#endif

#define ModeB	0
#define Qualcomm	1
#define NVT_TOUCH_CTRL_DRIVER 1
#define UPDATE_FIRMWARE 1
#define ReportRate	1

#define IC 3    //2:NT11002 3:NT11003 4:NT11004
#if (IC==2||IC==4)
#define BUFFER_LENGTH    16384
#else
#define BUFFER_LENGTH    28672
#endif

#define	DEBUG_MODE	0	

struct NVTtouch_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;		//use RESET flag
	int use_irq;		//use EINT flag
	int read_mode;		//read moudle mode,20110221 by andrew
	struct hrtimer timer;
	struct work_struct  work;
	char phys[32];
	int retry;
	struct early_suspend early_suspend;
	int (*power)(struct NVTtouch_ts_data * ts, int on);
	uint8_t X_num;
	uint8_t Y_num;
	int reset_pin;
	int int_pin;
	int id_pin0;
	int id_pin1;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint8_t int_trigger_type;
	uint8_t chip_ID;
	uint8_t green_wake_mode;
};

#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data {
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};
#endif
static const char *NVT_TS_NAME = "touchscreen";//"NVTCapacitiveTouchScreen";
static struct workqueue_struct *NVTtouch_wq;
struct i2c_client * i2c_connect_client_NVTtouch = NULL; 
	
#ifdef CONFIG_HAS_EARLYSUSPEND
static void NVTtouch_ts_early_suspend(struct early_suspend *h);
static void NVTtouch_ts_late_resume(struct early_suspend *h);
#endif 

//*****************************End of Part I *********************************

//*************************Touchkey Surpport Part*****************************
#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
	#define READ_COOR_ADDR 0x00
	const uint16_t touch_key_array[]={
									  KEY_BACK,				//MENU
									  KEY_HOME,				//HOME
									  KEY_MENU				//CALL
									 }; 
	#define MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#else
	#define READ_COOR_ADDR 0x00
#endif

//*************************End of Touchkey Surpport Part*****************************


#endif /* _LINUX_NVT_TOUCH_H */
