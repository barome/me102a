/* drivers/input/sensors/access/kxtj2.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>

//<-- ASUS-Bevis_Chen + -->
#ifdef CONFIG_ASUS_ENGINEER_MODE
#include <linux/workqueue.h>

struct i2c_client * ex_client;

				/* IOCTLs  library */
#define ASUS_GSENSOR_IOCTL_MAGIC			'a'
#define GBUFF_SIZE				12	/* Rx buffer size */

#define ASUS_GSENSOR_IOCTL_CLOSE		        _IO(ASUS_GSENSOR_IOCTL_MAGIC, 0x02)
#define ASUS_GSENSOR_IOCTL_START		        _IO(ASUS_GSENSOR_IOCTL_MAGIC, 0x03)
#define ASUS_GSENSOR_IOCTL_GETDATA               	_IOR(ASUS_GSENSOR_IOCTL_MAGIC, 0x08, char[GBUFF_SIZE+1])
#endif
//<-- ASUS-Bevis_Chen - -->


#define KXTJ2_DEVID	0x09	//chip id
#define KXTJ2_RANGE	2000000

//#define KXTJ2_XOUT_HPF_L                (0x00)	/* 0000 0000 */
//#define KXTJ2_XOUT_HPF_H                (0x01)	/* 0000 0001 */
//#define KXTJ2_YOUT_HPF_L                (0x02)	/* 0000 0010 */
//#define KXTJ2_YOUT_HPF_H                (0x03)	/* 0000 0011 */
//#define KXTJ2_ZOUT_HPF_L                (0x04)	/* 0001 0100 */
//#define KXTJ2_ZOUT_HPF_H                (0x05)	/* 0001 0101 */
#define KXTJ2_XOUT_L                    (0x06)	/* 0000 0110 */
#define KXTJ2_XOUT_H                    (0x07)	/* 0000 0111 */
#define KXTJ2_YOUT_L                    (0x08)	/* 0000 1000 */
#define KXTJ2_YOUT_H                    (0x09)	/* 0000 1001 */
#define KXTJ2_ZOUT_L                    (0x0A)	/* 0001 1010 */
#define KXTJ2_ZOUT_H                    (0x0B)	/* 0001 1011 */
#define KXTJ2_DCST_RESP                 (0x0C)	/* 0000 1100 */

#define KXTJ2_WHO_AM_I                  (0x0F)	/* 0000 1111 */
//#define KXTJ2_TILT_POS_CUR              (0x10)	/* 0001 0000 */
//#define KXTJ2_TILT_POS_PRE              (0x11)	/* 0001 0001 */
#define KXTJ2_INT_SRC_REG1              (0x15)	/* 0001 0101 */
#define KXTJ2_INT_SRC_REG2              (0x16)	/* 0001 0110 */

#define KXTJ2_STATUS_REG                (0x18)	/* 0001 1000 */

#define KXTJ2_INT_REL                   (0x1A)	/* 0001 1010 */
#define KXTJ2_CTRL_REG1                 (0x1B)	/* 0001 1011 */
//#define KXTJ2_CTRL_REG2                 (0x1C)	/* 0001 1100 */
#define KXTJ2_CTRL_REG2                 (0x1D)	/* 0001 1101 */
#define KXTJ2_INT_CTRL_REG1             (0x1E)	/* 0001 1110 */
#define KXTJ2_INT_CTRL_REG2             (0x1F)	/* 0001 1111 */
//#define KXTJ2_INT_CTRL_REG3             (0x20)	/* 0010 0000 */
#define KXTJ2_DATA_CTRL_REG             (0x21)	/* 0010 0001 */
//#define KXTJ2_TILT_TIMER                (0x28)	/* 0010 1000 */
#define KXTJ2_WAKEUP_TIMER              (0x29)	/* 0010 1001 */
//#define KXTJ2_TDT_TIMER                 (0x2B)	/* 0010 1011 */
//#define KXTJ2_TDT_H_THRESH              (0x2C)	/* 0010 1100 */
//#define KXTJ2_TDT_L_THRESH              (0x2D)	/* 0010 1101 */
//#define KXTJ2_TDT_TAP_TIMER             (0x2E)	/* 0010 1110 */
//#define KXTJ2_TDT_TOTAL_TIMER           (0x2F)	/* 0010 1111 */
//#define KXTJ2_TDT_LATENCY_TIMER         (0x30)	/* 0011 0000 */
//#define KXTJ2_TDT_WINDOW_TIMER          (0x31)	/* 0011 0001 */
//#define KXTJ2_WUF_THRESH                (0x5A)	/* 0101 1010 */
//#define KXTJ2_TILT_ANGLE                (0x5C)	/* 0101 1100 */
//#define KXTJ2_HYST_SET                  (0x5F)	/* 0101 1111 */
#define KXTJ2_SELT_TEST                  (0x3A)	/* 0011 1010 */
#define KXTJ2_WAKUP_THRESHOLD            (0x5F)	/* 0110 1010 */

/* CONTROL REGISTER 1 BITS */
#define KXTJ2_DISABLE			0x7F
#define KXTJ2_ENABLE			(1 << 7)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

/* CTRL_REG1: set resolution, g-range, data ready enable */
/* Output resolution: 8-bit valid or 12-bit valid */
#define KXTJ2_RES_8BIT		0
#define KXTJ2_RES_12BIT		(1 << 6)
/* Output g-range: +/-2g, 4g, or 8g */
#define KXTJ2_G_2G		0
#define KXTJ2_G_4G		(1 << 3)
#define KXTJ2_G_8G		(1 << 4)

/* DATA_CTRL_REG: controls the output data rate of the part */
#define KXTJ2_ODR12_5F		0
#define KXTJ2_ODR25F			1
#define KXTJ2_ODR50F			2
#define KXTJ2_ODR100F			3
#define KXTJ2_ODR200F			4
#define KXTJ2_ODR400F			5
#define KXTJ2_ODR800F			6
#define KXTJ2_ODR1600F			7

/* kxtj2 */
#define KXTJ2_PRECISION       12
#define KXTJ2_BOUNDARY        (0x1 << (KXTJ2_PRECISION - 1))
#define KXTJ2_GRAVITY_STEP    KXTJ2_RANGE / KXTJ2_BOUNDARY


/****************operate according to sensor chip:start************/

static int kxtj2_set_data_rate(struct i2c_client *client, unsigned char rate)
{
    return sensor_write_reg(client, KXTJ2_DATA_CTRL_REG , rate);
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int status = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	
	//register setting according to chip datasheet		
	if(enable)
	{	
		if(sensor->ops->ctrl_data & KXTJ2_ENABLE)
		{
			sensor->ops->ctrl_data &= ~KXTJ2_ENABLE;
			result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
			if(result)
			{
				goto active_error;
			}
		}
		if(rate >= 100)
		{
            rate = 100;
			result = kxtj2_set_data_rate(client,KXTJ2_ODR200F);
			if(result)
			{
				goto active_error;
			}
		}
		else if (rate >= 50)
		{
			result = kxtj2_set_data_rate(client,KXTJ2_ODR100F);
			if(result)
			{
				goto active_error;
			}
		}
		else
		{
			result = kxtj2_set_data_rate(client,KXTJ2_ODR50F);
			if(result)
			{
				goto active_error;
			}
		}
		if (rate != 0)
		{
			sensor->pdata->poll_delay_ms = (1000 / rate) / 10 * 10;
		}
		status = KXTJ2_ENABLE;	//kxtj2	
		sensor->ops->ctrl_data |= status;
	}
	else
	{
		status = ~KXTJ2_ENABLE;	//kxtj2
		sensor->ops->ctrl_data &= status;
	}

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
active_error:
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	
	return result;

} 

static int sensor_init(struct i2c_client *client)
{	
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	//<-- ASUS-Bevis_Chen + -->
	#ifdef CONFIG_ASUS_ENGINEER_MODE
	ex_client= client;
	#endif
	//<-- ASUS-Bevis_Chen - -->
	
	result = sensor->ops->active(client,0,0);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_OFF;
	
	result = sensor_write_reg(client, KXTJ2_DATA_CTRL_REG, KXTJ2_ODR100F);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	if(sensor->pdata->irq_enable)	//open interrupt
	{
		result = sensor_write_reg(client, KXTJ2_INT_CTRL_REG1, 0x34);//enable int,active high,need read INT_REL
		if(result)
		{
			printk("%s:line=%d,error\n",__func__,__LINE__);
			return result;
		}
	}
	
	sensor->ops->ctrl_data = (KXTJ2_RES_12BIT | KXTJ2_G_2G);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	return result;
}

static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
    s64 result;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	//int precision = sensor->ops->precision;
	switch (sensor->devid) {	
		case KXTJ2_DEVID:		
			result = (((int)high_byte << 8) | ((int)low_byte ))>>4;
			if (result < KXTJ2_BOUNDARY)
       			result = result* KXTJ2_GRAVITY_STEP;
    		else
       			result = ~( ((~result & (0x7fff>>(16-KXTJ2_PRECISION)) ) + 1) 
			   			* KXTJ2_GRAVITY_STEP) + 1;
			break;

		default:
			printk(KERN_ERR "%s: devid wasn't set correctly\n",__func__);
			return -EFAULT;
    }

    return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

#define GSENSOR_MIN  10
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);	
    	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x,y,z;
	struct sensor_axis axis;	
	char buffer[6] = {0};	
	char value = 0;
        struct Gsensor_cached_data *cachedData = (struct Gsensor_cached_data *)sensor->sensor_data;
	
	if(sensor->ops->read_len < 6)	//sensor->ops->read_len = 6
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	memset(buffer, 0, 6);
	
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	//this gsensor need 6 bytes buffer
	x = sensor_convert_data(sensor->client, buffer[1], buffer[0]);	//buffer[1]:high bit 
	y = sensor_convert_data(sensor->client, buffer[3], buffer[2]);
	z = sensor_convert_data(sensor->client, buffer[5], buffer[4]);		

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;	
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	if(cachedData->calibrationFlag == GSENSOR_NEED_CALIBRATION)
	{
        	axis.x -= cachedData->offset[0];
        	axis.y -= cachedData->offset[1];
        	axis.z -= cachedData->offset[2];
	}
	DBG( "%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	//Report event  only while value is changed to save some power
	if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
	{
		gsensor_report_value(client, &axis);

		/* \BB\A5\B3\E2\B5ػ\BA\B4\E6\CA\FD\BE\DD. */
		mutex_lock(&(sensor->data_mutex) );
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex) );
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0))	//read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n",__func__,value);
	}
	
	return ret;
}

struct sensor_operate gsensor_kxtj2_ops = {
	.name				= "kxtj2",
	.type				= SENSOR_TYPE_ACCEL,		//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_KXTJ2,		//i2c id number
	.read_reg			= KXTJ2_XOUT_L,			//read data
	.read_len			= 6,				//data length
	.id_reg				= KXTJ2_WHO_AM_I,		//read device id from this register
	.id_data 			= KXTJ2_DEVID,			//device id
	.precision			= KXTJ2_PRECISION,		//12 bits
	.ctrl_reg 			= KXTJ2_CTRL_REG1,		//enable or disable 
	.int_status_reg 		= KXTJ2_INT_REL,		//intterupt status register
	.range				= {-KXTJ2_RANGE,KXTJ2_RANGE},	//range
	.trig				= IRQF_TRIGGER_LOW|IRQF_ONESHOT,		
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_kxtj2_ops;
}




/*******************ASUS_ENGINEER_MODE PART************************/
//<-- ASUS-Bevis_Chen + -->
#ifdef CONFIG_ASUS_ENGINEER_MODE

static long asus_gsensor_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{

	if(ex_client == NULL){
		printk("%s: ERROR ! ASUS GSENSOR ioctl ex_client is NULL  \n", __func__);
		return -EFAULT;
		}
		
	struct sensor_private_data *sensor =
	   (struct sensor_private_data *) i2c_get_clientdata(ex_client);	

	struct i2c_client *client = sensor->client;
	void __user *argp = (void __user *)arg;
	int result = 0;

	struct sensor_platform_data *pdata;
	int ret = 0;
	int x,y,z;
	struct sensor_axis temp_axis = {0};	
	char buffer[6] = {0};	
	char value = 0;

	switch (cmd) {
	case ASUS_GSENSOR_IOCTL_START:	
		printk("%s:ASUS GSENSOR_IOCTL_START start,status=%d\n", __func__,sensor->status_cur);
		mutex_lock(&sensor->operation_mutex);	
		if(++sensor->start_count == 1)
		{
			printk("%s:ASUS  sensor->ops->active \n", __func__);
			if(sensor->status_cur == SENSOR_OFF)
			{
				atomic_set(&(sensor->data_ready), 0);
				if ( (result = sensor->ops->active(client, 1, 0) ) < 0 ) {
		        		mutex_unlock(&sensor->operation_mutex);
					printk("%s:ASUS fail to active sensor,ret=%d\n",__func__,result);         
					goto error;           
		    		}			
				sensor->status_cur = SENSOR_ON;
			}	
		}
	        mutex_unlock(&sensor->operation_mutex);
	        printk("%s:ASUS GSENSOR_IOCTL_START OK\n", __func__);
	        break;

	case ASUS_GSENSOR_IOCTL_CLOSE:				
	        printk("%s:ASUS GSENSOR_IOCTL_CLOSE start,status=%d\n", __func__,sensor->status_cur);
	        mutex_lock(&sensor->operation_mutex);		
		if(--sensor->start_count == 0)
		{
			printk("%s:ASUS  sensor->ops->unactive \n", __func__);
			if(sensor->status_cur == SENSOR_ON)
			{
				atomic_set(&(sensor->data_ready), 0);
				if ( (result = sensor->ops->active(client, 0, 0) ) < 0 ) {
		                	mutex_unlock(&sensor->operation_mutex);              
					goto error;
		            	}	
				sensor->status_cur = SENSOR_OFF;
		        }
			
			printk("%s:ASUS GSENSOR_IOCTL_CLOSE OK\n", __func__);
		}
		
	        mutex_unlock(&sensor->operation_mutex);	
	        break;
		
	case ASUS_GSENSOR_IOCTL_GETDATA:
		pdata = sensor->pdata;
		mutex_lock(&sensor->data_mutex);
			
			if(sensor->ops->read_len < 6)	//sensor->ops->read_len = 6
			{
				printk("%s:ASUS lenth is error,len=%d\n",__func__,sensor->ops->read_len);
				return -1;
			}
			
			memset(buffer, 0, 6);
			
			/* Data bytes from hardware xL, xH, yL, yH, zL, zH */	
			do {
				*buffer = sensor->ops->read_reg;
				ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
				if (ret < 0)
				return ret;
			} while (0);
		
			//this gsensor need 6 bytes buffer
			x = sensor_convert_data(sensor->client, buffer[1], buffer[0]);	//buffer[1]:high bit 
			y = sensor_convert_data(sensor->client, buffer[3], buffer[2]);
			z = sensor_convert_data(sensor->client, buffer[5], buffer[4]);		
		
			temp_axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
			temp_axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
			temp_axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
		
			printk( "%s:ASUS temp_axis = %d  %d  %d \n", __func__, temp_axis.x, temp_axis.y, temp_axis.z);
		
		//memcpy(&axis, &sensor->axis, sizeof(sensor->axis));	//get data from buffer
		mutex_unlock(&sensor->data_mutex);		

		 if ( copy_to_user(argp, &temp_axis, sizeof(temp_axis) ) ) {
	            printk("%s:ASUS failed to copy sense data to user space.\n",__func__);
				result = -EFAULT;			
				goto error;
	           }		
		printk("%s:ASUS_GSENSOR_IOCTL_GETDATA OK\n", __func__);
		break;
	default:
		result = -ENOTTY;
	goto error;
	}

error:
	return result;
}


struct file_operations asus_gsensor_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_gsensor_ioctl,
};

struct miscdevice asus_gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asus_gsensor",
	.fops = &asus_gsensor_fops,
};

int gsensor_register(void)
{
	int err=-1;
	err = misc_register(&asus_gsensor_device);
	if (err)
		printk(KERN_ERR "%s: asus_gsensor misc register failed\n", __func__);
	return err;
}

#endif
//<-- ASUS-Bevis_Chen - -->



static int __init gsensor_kxtj2_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);

	//<-- ASUS-Bevis_Chen + -->
	#ifdef CONFIG_ASUS_ENGINEER_MODE
	if(gsensor_register())
		return -1;
	#endif
	//<-- ASUS-Bevis_Chen - -->
	
	return result;
}

static void __exit gsensor_kxtj2_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_kxtj2_init);
module_exit(gsensor_kxtj2_exit);

