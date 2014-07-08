 /* drivers/input/touchscreen/NVTtouch_ts.c
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>

#include "NVTtouch.h"
#include "NVT_Alg.h"

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/random.h>
#if ModeB
#include <linux/input/mt.h>
#endif

//#define UPDATE_FIRMWARE 1

#if UPDATE_FIRMWARE
#include "Wintek101_ASUSME102A_N327_V116_20131025.h" //for wintek tp
#include "O-Film101_ASUSME102A_N327_V36_20131011.h" //for O-Film tp
#endif


#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) > (b) ? (b) : (a))

#define TP_VENDOR_O_FILM  0
#define TP_VENDOR_WINTEK  1
#define VENDOR_O_FILM  0xA5
#define VENDOR_WINTEK  0x5A
#define STATUS_NONE            0
#define STATUS_IN         1
#define STATUS_MOVE             2
#define STATUS_BREAK          3
void GetRawData(void);
//void GetDiffData(void);
void TrackingFunc_MinDis(int TrackThreshold, unsigned char X_Jitter_Max, unsigned char Y_Jitter_Max, unsigned char X_Jitter_Offset, unsigned char Y_Jitter_Offset, int IIR_Weight, int IIR_Start, unsigned int IIR_Distance);
void Boundary(int Max_X, int Max_Y, int XEdgeDist_H, int XEdgeDist_L, int YEdgeDist_H, int YEdgeDist_L, unsigned char XCorrectionGradient_H, unsigned char XCorrectionGradient_L, unsigned char YCorrectionGradient_H, unsigned char YCorrectionGradient_L);
unsigned long int Distance_Sqrt(unsigned long int PosX1, unsigned long int PosY1, unsigned long int PosX2, unsigned long int PosY2);
void Report_Buffer_Before(int);
void Report_Buffer_After(int bufferNum);
void Para_Parser(unsigned char *pData);
void nv_touch_mode(struct NVTtouch_ts_data *ts ,int g_charger_mode);
void Point_FIR(unsigned char index, char MIN_Fir_Cnt, char MAX_Fir_Cnt, unsigned int Fir_Step);
unsigned int getArea(unsigned char index);
unsigned int getPressure(unsigned char index);
void MedianFilter_init(unsigned char ID, unsigned int crood_x, unsigned int crood_y);
void MedianFilter(unsigned short InX, unsigned short InY, unsigned char ID, unsigned char Order, unsigned char index);


//struct NVTtouch_ts_data *ts;

//char DiffTemp[1000]={0};
extern g_charger_mode;
int usb_state=0;
int X_Num, Y_Num;

//unsigned char ID_Table[20];
int tp_id =0;
int tp_id1 =0;

int workfinish = 0;





struct TouchConfig NVT_Config;

struct PointState ReportPointState;
struct PointState Boundary_ReportPointState;
struct PointState preReportPointState;
struct PointState mPointState;
struct PointState prePointState;
struct PointState IIRPointState;
struct PointState preIIRPointState;

/*******************************************************	
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = client->flags;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];
	msgs[0].scl_rate = NVT_I2C_SCLK;
	msgs[0].udelay = client->udelay;

	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];
	msgs[1].scl_rate = NVT_I2C_SCLK;
	msgs[1].udelay = client->udelay;
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	
			break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = client->flags;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = data;		
	msg.scl_rate = NVT_I2C_SCLK;
	msg.udelay = client->udelay;
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	
			break;
		retries++;
	}
	return ret;
}


#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"
struct nvt_flash_data *flash_priv;

/*******************************************************
Description:
	Novatek touchscreen control driver initialize function.

Parameter:
	priv:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	
	char *str;
	int ret=-1;
	int retries = 0;
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	ret=copy_from_user(str, buff, count);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = str[1];
	msgs[0].buf   = &str[2];
	msgs[0].scl_rate = NVT_I2C_SCLK;
	
	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 1);
		if(ret == 1)	break;
		else
			printk("write error %d\n", retries);
		retries++;
	}
 	return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msgs[2];	 
 char *str;
 int ret = -1;
 int retries = 0;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 if(copy_from_user(str, buff, count))
	return -EFAULT;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = 1;
	msgs[0].buf   = &str[2];
	msgs[0].scl_rate = NVT_I2C_SCLK;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = str[0];
	msgs[1].len   = str[1]-1;
	msgs[1].buf   = &str[3];
	msgs[1].scl_rate = NVT_I2C_SCLK;
	
	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 2);
		if(ret == 2)	
			break;
		else
			printk("read error %d\n", retries);
		retries++;
	}
	ret=copy_to_user(buff, str, count);
 return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev) {
		kfree(dev);
	}
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_init(struct NVTtouch_ts_data *ts)
{		
	int ret = 0;
  	NVT_proc_entry = create_proc_entry(DEVICE_NAME, 0666, NULL);
	if(NVT_proc_entry == NULL)
	{
		printk("Couldn't create proc entry!\n");
		ret = -ENOMEM;
		return ret ;
	}
	else
	{
		printk("Create proc entry success!\n");
		NVT_proc_entry->proc_fops = &nvt_flash_fops;
	}
	flash_priv=kmalloc(sizeof(*flash_priv),GFP_KERNEL);	
	if (ts == NULL) {
                ret = -ENOMEM;
                goto error;
	}
	flash_priv->client = ts->client;
	printk("============================================================\n");
	printk("NVT_flash driver loaded\n");
	printk("============================================================\n");	
	return 0;
error:
	if(ret != 0)
	{
	printk("flash_priv error!\n");
	}
	return -1;
}

#endif
#if ReportRate
struct timeval TimeOrgin;
struct timeval TimeNow;
int TouchCount;
int ShowReportRate(void)
{
	if(TouchCount==0)
		do_gettimeofday(&TimeOrgin);
	do_gettimeofday(&TimeNow);
	if(TimeNow.tv_sec>TimeOrgin.tv_sec)
	{
		do_gettimeofday(&TimeOrgin);
		return 1;
	}		
	else
	{
		return 0;		
	}
}
#endif
/*******************************************************
Description:
	Novatek touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int Soft_Reset(struct NVTtouch_ts_data *ts)
{
	struct i2c_msg msg;
	unsigned char data[2]={0x00, 0x5A};
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = 0x7F;
	msg.len   = 2;
	msg.buf   = data;		
	
	while(retries < 5)
	{
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}

int CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len);

static int NVTtouch_init_panel(struct NVTtouch_ts_data *ts)
{
	int ret = -1;
	uint8_t rd_cfg_buf[16] = {0x78,};
    	struct i2c_client *client = ts->client;
	ts->int_trigger_type = INT_TRIGGER;


	ret = i2c_read_bytes(ts->client, rd_cfg_buf, 16);
	if(ret != 2) {
		dev_info(&client->dev, "Read resolution & max_touch_num failed, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		ts->max_button_num = MAX_KEY_NUM;
		ts->chip_ID = IC;
		return 0;
	}
	X_Num = rd_cfg_buf[3];
	Y_Num = rd_cfg_buf[4];
	printk("X=%d, Y=%d \n", X_Num, Y_Num);
	ts->abs_x_max = (rd_cfg_buf[5]<<8) + rd_cfg_buf[6];
	ts->abs_y_max = (rd_cfg_buf[7]<<8) + rd_cfg_buf[8];
	ts->max_touch_num = rd_cfg_buf[10];
	ts->max_button_num = rd_cfg_buf[11];
	ts->int_trigger_type = rd_cfg_buf[12];
	ts->chip_ID = rd_cfg_buf[15];
	dev_info(&client->dev, "ts->chip_ID=%d\n",rd_cfg_buf[15]);
	
	if(ts->chip_ID != IC){
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		ts->max_button_num = MAX_KEY_NUM;
		ts->chip_ID = IC;
		dev_info(&client->dev, "Read FW error!!!");
		return 0;
	}
	
	if((!ts->abs_x_max)||(!ts->abs_y_max)||(!ts->max_touch_num)) {
		dev_info(&client->dev, "Read invalid resolution & max_touch_num, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
		ret = -1;
	}
	
	dev_info(&ts->client->dev,"X_MAX = %d,Y_MAX = %d,MAX_TOUCH_NUM = %d\n",ts->abs_x_max,ts->abs_y_max,ts->max_touch_num);
	
	if((rd_cfg_buf[13] & 0x0f) == 0x05) {
		dev_info(&ts->client->dev, "Touchscreen works in INT wake up green mode!\n");
		ts->green_wake_mode = 1;
	}
	else {
		dev_info(&ts->client->dev, "Touchscreen works in IIC wake up green mode!\n");
		ts->green_wake_mode = 0;
	}
	//msleep(10);
	return 0; 
}



/*******************************************************
Description:
	Novatek touchscreen update firmware function.

Parameter:
	ts:	i2c client private strduct.
	
return:
	none.
*******************************************************/


#if UPDATE_FIRMWARE

int CTP_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];
	msgs[0].scl_rate = NVT_I2C_SCLK;
	msgs[0].udelay = client->udelay;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];
	msgs[1].scl_rate = NVT_I2C_SCLK;
	msgs[1].udelay = client->udelay;
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	
			break;
		retries++;
	}
	return ret;	
}


int CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;		
	msg.scl_rate = NVT_I2C_SCLK;
	msg.udelay = client->udelay;	
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	
			break;
		retries++;
	}
	return ret;
}

#define FW_VERSION	0x05		//depends on the data in nt11003_firmware.h
int Check_FW_Ver(struct NVTtouch_ts_data *ts, uint8_t *FWversion)
{
	int ret = -1;
	uint8_t FWversion_Inverse;
	struct i2c_client *client = ts->client;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		I2C_Buf[0] = 0x78;
		ret = CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 3);
		
		if(ret < 0)
			return ret;
			
		*FWversion = I2C_Buf[1];
		FWversion_Inverse = I2C_Buf[2];
		
		dev_info(&client->dev, " ~FWversion_Inverse= %d\n",(uint8_t)(~FWversion_Inverse));
		dev_info(&client->dev, "FWversion = %d \n", I2C_Buf[1]);
		
		if( (*FWversion) != (uint8_t)(~FWversion_Inverse))
			return -1;
				
		return 1;

}

void Update_Firmware(struct NVTtouch_ts_data *ts,const uint8_t *BUFFER_DATA, uint16_t const FW_Length)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint16_t i = 0;
	uint16_t j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = ts->client;
 
	//-------------------------------
	// Step1 --> initial BootLoader
 	// Note. 0x7F -> 0x00 -> 0x00 ;
 	// 須配合 Reset Pin 
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a “A5H” to NT1100x

	msleep(2);	// Delay.2mS
 
	//Step 1 : Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a 0x00 to NT1100x

	msleep(5);	// Delay.5mS


	// Read NT1100x status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
	// if return “AAH” then going next step
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "Program : init get status(0x%2X) error", I2C_Buf[1]);
		return;
	}
	dev_info(&client->dev, "Program : init get status(0x%2X) success", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 
 	//---------------------------------------------------------

	if(ts->chip_ID==3)
	{
	 	for (i = 0 ; i < FW_Length/128 ; i++)	// 26K equals 208 Rows 
	 	{
	 		Row_Address = i * 128; 															
	 						 					
	 		I2C_Buf [0] = 0x00;
	 		I2C_Buf [1] = 0x30;	// Row Erase command : 30H  
	 		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
	 		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 
	 						
	 		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 4);	// Write 30H, Addr_H & Addr_L to NT11003
	 
	 		msleep(15);	// Delay 15 ms 
		          
	    	// Read NT11003 status
	 		CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2); 
		          
	    	// if NT1003 return AAH then going next step          
		    	if (I2C_Buf[1] != 0xAA)
		    	{
		    		dev_info(&client->dev, "Program : erase(0x%2X) error", I2C_Buf[1]);
		    		return;
		    	}			
	 	}
	}
	else if(ts->chip_ID==2)
	{
	    	for(i=0;i<FW_Length/2048;i++)
		{
			Row_Address=i>>11;
			I2C_Buf[0]=0x00;
			I2C_Buf[1]=0x33;
			I2C_Buf[2]=i<<3;
			I2C_Buf[3]=0x00;
			CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 4);
			msleep(20);

			I2C_Buf[0]=0x00;

			while(1)
			{
				msleep(1);
				CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			    	msleep(1);
			}
		}
	}
	else if(ts->chip_ID==4)
	{

		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0xF0;
		I2C_Buf[2]=0xAC;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x21;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x99;
		I2C_Buf[2]=0x00;
		I2C_Buf[3]=0x0E;
		I2C_Buf[4]=0x01;
		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 5);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x81;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x99;
		I2C_Buf[2]=0x00;
		I2C_Buf[3]=0x0F;
		I2C_Buf[4]=0x01;
		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 5);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x01;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x00;
		I2C_Buf[2]=0x00;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
		msleep(20);

	    	for(i=0;i<FW_Length/4096;i++)
		{
			Row_Address=(i*16);
			I2C_Buf[2]=0x00;
			I2C_Buf[3]=0x33;
			I2C_Buf[4]=Row_Address;
			CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 3);
			msleep(80);

			I2C_Buf[0]=0x00;

			while(1)
			{
				CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			    	msleep(1);
			}
		}
	}
	dev_info(&client->dev, "Program : erase(0x%2X) success", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to NT11003  
	//----------------------------------------
	dev_info(&client->dev, "Program : write begin, please wait ...");
	if(ts->chip_ID==3)
	{
		for (j = 0 ; j < FW_Length/128 ; j++)	// Write/ Read 208 times 
		{
			Flash_Address = j * 128 ; 						     		 

		   	for (i = 0 ; i < 16 ; i++)	// 128/8 = 16 times for One Row program 
			{
	    		// Step 3 : write binary data to NT11003  
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8
		   
				// Calculate a check sum by Host controller. 
				// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
				//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
				//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1 
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
		    	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
		        	      I2C_Buf[13]) + 1; 
		   		
				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
				CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.
				if( i == 15 ){
					msleep(7);

				// Read NT1100x status
	   			I2C_Buf[0] = 0x00;
				CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

				// if return “AAH” then going next step
				if (I2C_Buf[1] != 0xAA)
				{
					dev_info(&client->dev, "Program : write(j=%d, i=%d, 0x%2X) error", j, i, I2C_Buf[1]);
	      			return;
				}
				}
				Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
			}
		   	
			msleep(15);	// Each Row program --> Need 15ms delay time 
		}
	}
	else if(ts->chip_ID==2)
	{
	    	for(j=0;j<FW_Length/32;j++)
		{
	    		Flash_Address=(j)*32;

		    	for (i = 0 ; i < 4 ; i++, Flash_Address += 8)	// 128/8 = 16 times for One Row program
			{
	    		// Step 3 : write binary data to NT11003
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8

				// Calculate a check sum by Host controller.
				// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
				//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
				//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
	            	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
	                	      I2C_Buf[13]) + 1;

				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer
				CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);

				msleep(2);
			}
			// Read NT1100x status
   			I2C_Buf[0] = 0x00;
			while(1)
			{
				CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			}
		}		
	}
	else if(ts->chip_ID==4)
	{
		for (j = 0 ; j < FW_Length/128 ; j++)
		{
			Flash_Address = j * 128 ; 						     		 

		   	for (i = 0 ; i < 16 ; i++)
			{
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8
		   
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
		    	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
		        	      I2C_Buf[13]) + 1; 
		   		
				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
				CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

				msleep(1);	// Delay 1 ms 

				// Read NT1100x status
	   			I2C_Buf[0] = 0x00;
				CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

				// if return “AAH” then going next step
				if (I2C_Buf[1] != 0xAA)
				{
					dev_info(&client->dev, "Program : write(j=%d, i=%d, 0x%2X) error", j, i, I2C_Buf[1]);
	      			return;
				}
				Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
			}
		   	
			msleep(15);	// Each Row program --> Need 15ms delay time 
		}
	}

	dev_info(&client->dev, "Program : write finish ~~");
	/////////////////////////////////////////////////////////////////////
	
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	
	msleep(500);
	dev_info(&client->dev, "Program : OK");

}
#endif

/*******************************************************
Description:
	Novatek touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
#if ModeB
static unsigned char touch_cunt_old;
#endif
struct timeval TimeOrgin1;
struct timeval TimeNow1;



static void NVTtouch_ts_work_func(struct work_struct *work)
{

	int ret = -1;
	// Support 10 points maximum
	struct NVTtouch_ts_data *ts = container_of(work, struct NVTtouch_ts_data, work);
	uint8_t  point_data[ (MAX_FINGER_NUM*6)+2+1+20]={0};
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0,1,2,3,4,5,6,7,8,9};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned char input_w = 0;
	unsigned char index = 0;
	int count;

	unsigned long int i, j ;
	unsigned int X_Coordination, Y_Coordination;
	unsigned long int X_Temp, Y_Temp;
	unsigned char run_count = 0;
	struct i2c_client *client = ts->client;
	unsigned char Report = 0;
	unsigned char reportCnt = 0;


#if ModeB
	unsigned char lift_count = 0;
	unsigned char touch_cunt_now = 0;
#endif	

	workfinish = 0;

	ret = i2c_read_bytes(ts->client, point_data,  (MAX_FINGER_NUM*6)+2+1+20);
	mPointState.points=0;


	for(i=0;i<10;i++)
	{
		mPointState.Status[i]= STATUS_NONE;
        mPointState.ID[i] = 0xFF;
		mPointState.PosX[i] = 0xFFFF;
		mPointState.PosY[i] = 0xFFFF;
	}

	
	for(i=0;i<10;i++)
	{		
		if ( !((point_data[i*6+2] == 0xFF) && (point_data[i*6+4] == 0xFF)) )
		{			            
			mPointState.Status[mPointState.points] = STATUS_IN;
			mPointState.PosX[mPointState.points] = (point_data[i*6+2]<<4)|(point_data[i*6+4]>>4);
			mPointState.PosY[mPointState.points] = (point_data[i*6+3]<<4)|(point_data[i*6+4]&0x0F);
            mPointState.Area[mPointState.points] = point_data[i*6+5];
            mPointState.Pressure[mPointState.points] = point_data[i*6+6];
            mPointState.points++;
		}
	}

	//parameter parser
	Para_Parser(point_data);
/*
#if DEBUG_MODE
	printk("Interpolation Points = %d\n", mPointState.points);
#endif
*/

	if (mPointState.points == 0)
	{
		NoTouchCnt++;
		if (NoTouchCnt > NO_TOUCH_CNT_MAX)
			NoTouchCnt = NO_TOUCH_CNT_MAX;
	}
	else
		NoTouchCnt = 0;

	TrackingFunc_MinDis(C_TrackThreshold, FW_X_Jitter_Maximun , FW_Y_Jitter_Maximun, FW_X_Jitter_Offset, FW_Y_Jitter_Offset, FW_IIR_Weight, FW_IIR_Start, (FW_IIR_Distance_H*256 + FW_IIR_Distance_L));

	if (NoTouchCnt == NO_TOUCH_CNT_MAX)
	{
		for (i=0; i<MAXID; i++)
		{
			ID_Valid[i] = 0;
			ID_Valid_pre[i] = 0;
			ID_Table[i] = 0;
			Enter_Debounce[i] = 0;
			Break_Debounce[i] = 0;	
			Large_Move_Cnt[i] = 0;								
		}
	}

	//printk("IIR_X = %d   ,     IC_Y = %d \n", ReportPointState.PosX[0],ReportPointState.PosY[0]);

	Boundary(C_X_Resolution, C_Y_Resolution, C_XEdgeDist_H, C_XEdgeDist_L, C_YEdgeDist_H, C_YEdgeDist_L, C_XCorrectionGradient_H, C_XCorrectionGradient_L, C_YCorrectionGradient_H, C_YCorrectionGradient_L);

    Report_Buffer_Before(FW_FinUP_TH);
    
	//printk("Boundary_X = %d   ,     Boundary_Y = %d \n", Boundary_ReportPointState.PosX[0],Boundary_ReportPointState.PosY[0]);

	if(mPointState.points > 0)
	{
		for (index=0; index<mPointState.points; index++)
		{
            if (reportCnt >= MAX_REPORT_NUM)
                break;
                
		    if (ID_Valid[mPointState.ID[index]]==1)
		    {
    		    Report = 1;

  				//X_Temp = (Boundary_ReportPointState.PosX[index]*5)/11;
   				//Y_Temp = (Boundary_ReportPointState.PosY[index]*25)/54;			

		   		if (FW_FinUP_TH == 0)
				{
					X_Temp = ((Boundary_ReportPointState.PosX[index])*5)/11;
					Y_Temp = ((Boundary_ReportPointState.PosY[index])*25)/54;
				}
				else
				{
		            X_Temp = ((Report_Buffer_X[mPointState.ID[index]][0])*5)/11;
		            Y_Temp = ((Report_Buffer_Y[mPointState.ID[index]][0])*25)/54;
		        }
           
    		    X_Coordination = (unsigned int)X_Temp;
    		    Y_Coordination = (unsigned int)Y_Temp;

				if ( (X_Coordination == 0) && (Y_Coordination == 0))
				{
    		    	//printk("Report_X = %d, Report_Y = %d, ID = %d, ID_Table = %d\n", X_Coordination,Y_Coordination, mPointState.ID[index], ID_Table[mPointState.ID[index]]);
					
					ID_Valid[mPointState.ID[index]] = 0;
					ID_Table[mPointState.ID[index]] = 0;
					Enter_Debounce[mPointState.ID[index]] = 0;
					Break_Debounce[mPointState.ID[index]] = 0;	
					Large_Move_Cnt[mPointState.ID[index]] = 0;
					continue;
				}
				else
					reportCnt++;

    	  	 	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, X_Coordination);
    		    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, Y_Coordination);
    		    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, getArea(index));//mPointState.Area[index]);
    		    input_report_abs(ts->input_dev, ABS_PRESSURE, getPressure(index));//mPointState.Pressure[index]);
    		    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, mPointState.ID[index]+1);
    		    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    		    input_mt_sync(ts->input_dev);
		    }
		}
		
		if (Report == 1)
		{
		    input_sync(ts->input_dev);
		}
		else
		{
		   input_report_key(ts->input_dev, BTN_TOUCH, 0);
		   input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	       input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		   input_mt_sync(ts->input_dev);
		   input_sync(ts->input_dev);
		}
 
	}
	else
	{
	    if(prePointState.points>0)
	    {	
    		input_report_key(ts->input_dev, BTN_TOUCH, 0);
    		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
    		input_mt_sync(ts->input_dev);
    		input_sync(ts->input_dev);
	    }

	}    	

    Report_Buffer_After(FW_FinUP_TH);
        
	prePointState.points = mPointState.points;
    	
	for (i=0; i<mPointState.points; i++)
	{
	    prePointState.ID[i] = mPointState.ID[i];	
	    prePointState.Status[i] = mPointState.Status[i];	
	    prePointState.PosX[i] = mPointState.PosX[i];	
	    prePointState.PosY[i] = mPointState.PosY[i];
        prePointState.Area[i] = mPointState.Area[i];
        prePointState.Pressure[i] = mPointState.Pressure[i];
	    preReportPointState.PosX[i] = ReportPointState.PosX[i];	
	    preReportPointState.PosY[i] = ReportPointState.PosY[i];
	    preIIRPointState.PosX[i] = IIRPointState.PosX[i];	
	    preIIRPointState.PosY[i] = IIRPointState.PosY[i];        
	}
    
    for (i=0; i<MAXID; i++)
    {
        ID_Valid_pre[i] = ID_Valid[i];
    }
	
	if(g_charger_mode != usb_state){
		nv_touch_mode(ts , g_charger_mode);
	}
    
	if(ts->use_irq)
		enable_irq(ts->client->irq);
    
	workfinish = 1;
	return;

								
#if defined(INT_PORT)
	if(ts->int_trigger_type > 1)
	{
		msleep(POLL_TIME);
		//goto COORDINATE_POLL;
	}
#endif
	goto END_WORK_FUNC;

//NO_ACTION:	

#ifdef HAVE_TOUCH_KEY
//	dev_info(&client->dev, "HAVE KEY DOWN!0x%x\n",point_data[1]);
	if(point_data[ts->max_touch_num*6+1]==0x1F)	
	{
		for(count = 0; count < MAX_KEY_NUM; count++)
		{
			input_report_key(ts->input_dev, touch_key_array[count], ((point_data[ts->max_touch_num*6+2]>>count)&(0x01)));	
		}
	}	   
#endif

prePointState.points=mPointState.points;	
for(i=0;i<10;i++)
{
    prePointState.ID[i]=mPointState.ID[i];	
    prePointState.Status[i]=mPointState.Status[i];	
    prePointState.PosX[i]=mPointState.PosX[i];	
    prePointState.PosY[i]=mPointState.PosY[i];
    preIIRPointState.PosX[i]=IIRPointState.PosX[i];	
    preIIRPointState.PosY[i]=IIRPointState.PosY[i];
	
}
END_WORK_FUNC:
XFER_ERROR:
	if(ts->use_irq)
		enable_irq(ts->client->irq);

	workfinish = 1;




}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.
	
return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart NVTtouch_ts_timer_func(struct hrtimer *timer)
{
	struct NVTtouch_ts_data *ts = container_of(timer, struct NVTtouch_ts_data, timer);
	queue_work(NVTtouch_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t NVTtouch_ts_irq_handler(int irq, void *dev_id)
{
	struct NVTtouch_ts_data *ts = dev_id;
#if ReportRate
	if(ShowReportRate()==1)
	{
		//z//printk("Report Rate = %d\n", TouchCount);
		TouchCount=0;
	}
	else
	{
		TouchCount++;
	}

#endif

	disable_irq_nosync(ts->client->irq);
	queue_work(NVTtouch_wq, &ts->work);
	
	return IRQ_HANDLED;
}


static int NVTtouch_ts_por(struct NVTtouch_ts_data *ts) {
	int ret = 0;
	if(ts->reset_pin > 0) {
		ret = gpio_request(ts->reset_pin, "NVT reset pin");
		if (ret != 0) {
			gpio_free(ts->reset_pin);
			dev_info(&ts->client->dev, "Request NVT reset pin :%d fail\n", ts->reset_pin);
			return -EIO;
		} 
	}
	if(ts->int_pin > 0) {
		ret = gpio_request(ts->int_pin, "NVT interrupt pin");
		if (ret != 0) {
			gpio_free(ts->int_pin);
			dev_info(&ts->client->dev, "Request NVT interrupt pin :%d fail\n", ts->int_pin);
			return -EIO;
		} 
	}
	
	gpio_direction_input(ts->int_pin);
	//pull up and down
	
	gpio_direction_output(ts->reset_pin, GPIO_HIGH);
	msleep(100);
	gpio_set_value(ts->reset_pin, GPIO_LOW);
	msleep(10);
	gpio_set_value(ts->reset_pin, GPIO_HIGH);
	msleep(100);
	return 0;
}



uint8_t Check_TP_Vendor(struct NVTtouch_ts_data *ts)
{
	int ret = -1;
	struct i2c_client *client = ts->client;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x85;
	ret = CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);
	
	if(ret < 0)
		return ret;
	
	if (I2C_Buf[1] == TP_VENDOR_O_FILM || I2C_Buf[1] == VENDOR_O_FILM){	
		dev_info(&client->dev, "TP Vendor = O-Film");
		return VENDOR_O_FILM;
	}
	else if (I2C_Buf[1] == TP_VENDOR_WINTEK || I2C_Buf[1] == VENDOR_WINTEK){
		dev_info(&client->dev, "TP Vendor = Wintek");
		return VENDOR_WINTEK;
	}
	else{
    		dev_info(&client->dev, "Unknown TP Vendor");    
	return ret;
	}
}


//ASUS_BSP HANS: temporary way to notify usb state +++
void nv_touch_mode(struct NVTtouch_ts_data *ts ,int g_charger_mode)
{

	uint8_t I2C_Buf[16] = {0};

	if(ts==NULL)
		return;

	usb_state = g_charger_mode;

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x77;
    
	if(usb_state == 0){
	        //printk("[Touch_N] Usb Power OFF.\n");
		I2C_Buf[1] = 0xF0;
	}
	else if(usb_state == 1){
	        //printk("[Touch_N] Usb Power ON DC.\n");
	        I2C_Buf[1] = 0xF1;
	}
	else if(usb_state == 2){
	       // printk("[Touch_N] Usb Power ON AC.\n");
	        I2C_Buf[1] = 0xF2;
	}
	else{
		//printk("[Touch_N] Use USB default value %d\n",usb_state);
		usb_state = 0;
		I2C_Buf[1] = 0xF0;
	}
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
}
EXPORT_SYMBOL(nv_touch_mode);
//ASUS_BSP HANS: temporary way to notify usb state ---

/*******************************************************
Description:
	Novatek touchscreen 
 function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int NVTtouch_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	
	struct NVTtouch_ts_data *ts;
	const char irq_table[4] = {IRQ_TYPE_EDGE_RISING,
							   IRQ_TYPE_EDGE_FALLING,
							   IRQ_TYPE_LEVEL_HIGH,
							   IRQ_TYPE_LEVEL_LOW};

	int *pdata = client->dev.platform_data;
	
	dev_info(&client->dev, "Install touch driver.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_info(&client->dev,  "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		return ret;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		dev_info(&client->dev,  "Memory alloc fail for NVTtouch_ts\n");
		return ret;
	}
	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, NVTtouch_ts_work_func);

  	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_info(&client->dev, "Failed to allocate input device\n");
		goto err_NVT;
	}


  	//power on reset
  	ts->reset_pin = *pdata++;
  	ts->int_pin = *pdata++;
	ts->id_pin0 = *pdata++;
	ts->id_pin1 = *pdata;
  	printk("tp reset:%d, int:%d\n", ts->reset_pin, ts->int_pin);

  	//*----------------------------------------------------------------------------------------toby_debug
	if(ts->id_pin0 > 0) {
		ret = gpio_request(ts->id_pin0, "touch id pin");
		printk("\n Toby-> tp_id ret= %d",ret);
		if (ret != 0) {
			gpio_free(ts->id_pin0);
			dev_info(&ts->client->dev, "Request NVT id pin :%d fail\n", ts->id_pin0);
			return -EIO;
		}
		gpio_direction_input(ts->id_pin0);
		msleep(10);
	}
	
	tp_id=gpio_get_value(ts->id_pin0);
	
	printk("\n Toby-> tp_id = %d",tp_id);
	//-----------------------------------------------------------------------------------------*/

	//*----------------------------------------------------------------------------------------toby_debug
	if(ts->id_pin1 > 0) {
		ret = gpio_request(ts->id_pin1, "touch id pin");
		printk("\n Toby-> tp_id1 ret= %d",ret);
		if (ret != 0) {
			gpio_free(ts->id_pin1);
			dev_info(&ts->client->dev, "Request NVT id pin :%d fail\n", ts->id_pin1);
			return -EIO;
		} 
		gpio_direction_input(ts->id_pin1);
		msleep(10);
	}
	
	tp_id1=gpio_get_value(ts->id_pin1);
	
	printk("\n Toby-> tp_id = %d",tp_id1);
	//-----------------------------------------------------------------------------------------*/

if(1) //asus-eko:patch HW reset
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	
	msleep(500);
}

#if UPDATE_FIRMWARE
{
	uint8_t FWversion = 0;
	uint8_t TP_Vender = 0;
	ts->chip_ID = IC;
	//TP_Vender = Check_TP_Vendor(ts);
	

 	if(tp_id == 0 && tp_id1 == 0){
 		if(Check_FW_Ver(ts , &FWversion) > 0){
			if( FWversion != 36 ){		
				
					Update_Firmware(ts,BUFFER_DATA_NOVTEK,28*1024);
			}
		}
		else{
			Update_Firmware(ts,BUFFER_DATA_NOVTEK,28*1024);	
		}
		
 	}
	else if(tp_id == 1 && tp_id1 == 0){
		if(Check_FW_Ver(ts , &FWversion) > 0){
			if( FWversion != 116 ){
			 	Update_Firmware(ts,BUFFER_DATA_WINTEK,28*1024);
					
			}
		} 
		else {
			Update_Firmware(ts,BUFFER_DATA_WINTEK,28*1024);
					
		}
	}
	else if(tp_id == 1 && tp_id1 == 1){
		dev_info(&client->dev, "No Tp connected! ");
		return -1;
	}
	else{
		dev_info(&client->dev, "Unknown TP ");
		return -1;
	}
	 
	
}
#endif

#if 1
	ts->bad_data = 0;	
	for(retry=0; retry<3; retry++)
	{
		ret=NVTtouch_init_panel(ts);
		msleep(2);
		if(ret != 0)
			continue;
		else
			break;
	}
	if(ret != 0) {
		ts->bad_data=1;
		goto err_NVT;
	}
#else
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		ts->max_button_num = MAX_KEY_NUM;
		ts->chip_ID = IC;
#endif
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 

#if ModeB
	//input_mt_init_slots(ts->input_dev, ts->max_touch_num);
#endif	
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#ifdef NVTTOUCH_MULTI_TOUCH
//	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num*2, 0, 0);
#endif	

#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);	
	}
#endif
              
	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0306;
	ts->input_dev->id.product = 0xFF2F;

	
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_info(&client->dev, "Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_NVT;
	}


	memset(&prePointState, 0, sizeof(prePointState));
  	
	if (client->irq) {
		dev_info(&client->dev, "ts->int_trigger_type=%d\n",ts->int_trigger_type);
		ret  = request_irq(client->irq, NVTtouch_ts_irq_handler, irq_table[ts->int_trigger_type],
			client->name, ts);
		
		if (ret != 0) {
			dev_info(&client->dev, "Cannot allocate ts INT! ERRNO:%d\n", ret);
			goto err_NVT;
		}
		else {	
			disable_irq(client->irq);
			ts->use_irq = 1;
			dev_info(&client->dev, "Request EIRQ %d success\n", client->irq);
		}
	} else
		{
		goto err_NVT;    
	}
	


	
	
	
	
	if(ts->use_irq)
		enable_irq(client->irq);
	else {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = NVTtouch_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);	
	}	
		
#if NVT_TOUCH_CTRL_DRIVER
		nvt_flash_init(ts);
	#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = NVTtouch_ts_early_suspend;
	ts->early_suspend.resume = NVTtouch_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	dev_info(&client->dev, "Start %s in %s mode\n", 
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	nv_touch_mode(ts , g_charger_mode);
	
	return 0;
		
err_NVT:	
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int NVTtouch_ts_remove(struct i2c_client *client)
{
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts && ts->use_irq) 
	{
		free_irq(client->irq, ts);
	}	
	else if(ts)
		hrtimer_cancel(&ts->timer);
	
	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int NVTtouch_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	int i=0;
	int retry = 0;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);
	//printk("touch suspend start \n");
	
	while(retry < 200){
		
		if( workfinish == 1 )
			break;

		msleep(1);
		retry++;
	}

	//printk("suspend retry = %d \n",retry);
	
	if (ts->use_irq)
		disable_irq(client->irq);
	 else
		hrtimer_cancel(&ts->timer);
	  //ret = cancel_work_sync(&ts->work);
	  //if(ret && ts->use_irq)	
		//enable_irq(client->irq);
	
	if (ts->power) {
		ret = ts->power(ts, 0);
		if (ret < 0)
			dev_info(&client->dev, KERN_ERR "NVTtouch_ts_resume power off failed\n");
	}
	
	memset(&prePointState, 0, sizeof(prePointState));

	

	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x8F;
	I2C_Buf[2]=0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0xAF;
	//I2C_Buf[2]=0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
	msleep(50);
	
	return 0;
}

static int NVTtouch_ts_resume(struct i2c_client *client)
{
	int ret;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			dev_info(&client->dev, KERN_ERR "NVTtouch_ts_resume power on failed\n");
	}
	

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7f, I2C_Buf, 2);	
	msleep(500);
	nv_touch_mode(ts , g_charger_mode);

	memset(&prePointState, 0, sizeof(prePointState));
	if (ts->use_irq)
		enable_irq(client->irq);
	 else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	
	
	
	
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void NVTtouch_ts_early_suspend(struct early_suspend *h)
{
	struct NVTtouch_ts_data *ts;
	ts = container_of(h, struct NVTtouch_ts_data, early_suspend);
	NVTtouch_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void NVTtouch_ts_late_resume(struct early_suspend *h)
{
	struct NVTtouch_ts_data *ts;
	ts = container_of(h, struct NVTtouch_ts_data, early_suspend);
	NVTtouch_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id NVTtouch_ts_id[] = {
	{ NVTTOUCH_I2C_NAME, 0 },
	{ }
};

#if 0  //hhb
static struct i2c_board_info NVTtouch_i2c_boardinfo = {
	I2C_BOARD_INFO("NVT-ts", 0x01), .irq = IRQ_EINT(6)
};
#endif

static struct i2c_driver NVTtouch_ts_driver = {
	.probe		= NVTtouch_ts_probe,
	.remove		= NVTtouch_ts_remove,
	.shutdown   = NVTtouch_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= NVTtouch_ts_suspend,
	.resume		= NVTtouch_ts_resume,
#endif
	.id_table	= NVTtouch_ts_id,
	.driver = {
		.name	= NVTTOUCH_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

int NVT_add_i2c_device(struct i2c_board_info *info)
{
	int err;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	adapter = i2c_get_adapter(NVT_BUS_NUM);
	if(!adapter)
	{
		printk("%s: can't get i2c adapter %d\n", __func__, NVT_BUS_NUM);
		err=-ENODEV;
		goto err_driver;
	}	

	client = i2c_new_device(adapter, info);
	if(!client)
	{
		printk("%s: can't add i2c device at 0x%x\n", __func__, (unsigned int)info->addr);
		err=-ENODEV;
		goto err_driver;
	}	
	i2c_put_adapter(adapter);
	return 0;
err_driver:
	return err;
}


/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit NVTtouch_ts_init(void)
{
	int ret;
#if 0  //hhb	
	NVT_add_i2c_device(&NVTtouch_i2c_boardinfo);
#endif

	NVTtouch_wq = create_workqueue("NVTtouch_wq");		//create a work queue and worker thread
	if (!NVTtouch_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	ret = i2c_add_driver(&NVTtouch_ts_driver);
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit NVTtouch_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&NVTtouch_ts_driver);
	if (NVTtouch_wq)
		destroy_workqueue(NVTtouch_wq);		//release our work queue
}

late_initcall(NVTtouch_ts_init);
module_exit(NVTtouch_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
