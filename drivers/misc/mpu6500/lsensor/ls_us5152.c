/*
 * Copyright (C) 2012 UPI semi <Finley_huang@upi-semi.com>. All Rights Reserved.
 * us5152 Light Sensor Driver for Linux 2.6
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
/*===================================================*/
#include <asm/io.h>
#include <mach/board.h>
#include <mach/gpio.h>
/*===================================================*/
#include "ls_us5152.h"

#define us5152_DRV_NAME	"ls_us5152"
#define DRIVER_VERSION		"1.0"
//#define us5152_NUM_CACHABLE_REGS      8

#define LIGHT_SENSOR_START_TIME_DELAY	50000000
/*******************************************************************************/
/* global variable                                                             */
/*******************************************************************************/
static const struct i2c_board_info __initdata i2c_us5152= {I2C_BOARD_INFO("us5152",(0X39))};
static struct platform_driver us5152_als_driver;
static u8 reg_cache[us5152_NUM_CACHABLE_REGS];
/*******************************************************************************/
/* function declation                                                             */
/*******************************************************************************/
static void us5152_light_enable(struct i2c_client *client);
static void us5152_light_disable(struct i2c_client *client);

/* Each client has this additional data */
struct us5152_data {
        struct i2c_client *client;
        struct mutex lock;
        struct input_dev *input;
        struct work_struct work;
        struct workqueue_struct *workqueue;
	struct hrtimer timer;
	ktime_t light_poll_delay;
//        char phys[32];
//        u8 reg_cache[us5152_NUM_CACHABLE_REGS];
//        u8 mode_before_suspend;
  //      u8 mode_before_interrupt;
    //    u16 rext;
};

int us5152_i2c_read(struct i2c_client *client,u8 reg) {
	int val;
        val = i2c_smbus_read_byte_data(client, reg);
        if (val < 0)
                printk("%s %d i2c transfer error\n", __func__, __LINE__);
        return val;
}

int us5152_i2c_write(struct i2c_client *client, u8 reg,u8 mask, u8 shift, int val ) {
        struct us5152_data *data = i2c_get_clientdata(client);
	int err;
        u8 tmp;
        mutex_lock(&data->lock);

	tmp = reg_cache[reg];
        tmp &= ~mask;
        tmp |= val << shift;

        err = i2c_smbus_write_byte_data(client, reg, tmp);
        if (!err)
                reg_cache[reg] = tmp;

        mutex_unlock(&data->lock);
        if (err >= 0) return 0;

        printk("%s %d i2c transfer error\n", __func__, __LINE__);
        return err;
}

/* all int flag */
static int get_int_flag(struct i2c_client *client)
{
        return ((us5152_i2c_read(client,REGS_CR0) & 0x0E) >> 1);
}

static int set_int_flag(struct i2c_client *client, int flag)
{
	//write 0 clear
	return us5152_i2c_write(client, REGS_CR0,
                CR0_ALL_INT_MASK, CR0_ALL_INT_SHIFT, flag);
}


/* INT_A */
static int get_inta(struct i2c_client *client)
{
        return us5152_i2c_read(client,REGS_CR0) & 0x02;
}

static int set_inta(struct i2c_client *client)
{
	//write 0 clear
	return us5152_i2c_write(client, REGS_CR0,
                CR0_INTA_MASK, CR0_INTA_SHIFT, CR0_INTA_CLEAR);
}

/* OneShot */

static int get_oneshotmode(struct i2c_client *client)
{
        return (us5152_i2c_read(client,REGS_CR0)& 0x40) >> 6;
}

static int set_oneshotmode(struct i2c_client *client, int mode)
{
        return us5152_i2c_write(client,REGS_CR0,CR0_ONESHOT_MASK, CR0_ONESHOT_SHIFT, mode);
}

/*OP mode */
static int get_opmode(struct i2c_client *client)
{
        return (us5152_i2c_read(client,REGS_CR0)& 0x30) >> 4;
}

static int set_opmode(struct i2c_client *client, int mode)
{
        return us5152_i2c_write(client,REGS_CR0,CR0_OPMODE_MASK, 
		CR0_OPMODE_SHIFT, mode);
}

/* power_status */
static int set_power_status(struct i2c_client *client, int status)
{
	if(status == CR0_SHUTDOWN_EN )
                return us5152_i2c_write(client,REGS_CR0,CR0_SHUTDOWN_MASK, 
                	CR0_SHUTDOWN_SHIFT,CR0_SHUTDOWN_EN);
	else
		return us5152_i2c_write(client,REGS_CR0,CR0_SHUTDOWN_MASK, 
			CR0_SHUTDOWN_SHIFT, CR0_OPERATION);
}
static int get_power_status(struct i2c_client *client)
{
        u8 data;
	data = us5152_i2c_read(client,REGS_CR0) & 0x80;
	if (data)
        	return 0;/*data = 1, shut down*/
	else
		return 1;/*operation*/
}

/* ambient gain */
static int set_als_gain(struct i2c_client *client, int gain)
{
	
        return us5152_i2c_write(client,REGS_CR1,CR1_ALS_GAIN_MASK, 
              	CR1_ALS_GAIN_SHIFT,gain);
}
static int get_als_gain(struct i2c_client *client)
{
        u8 data;

	data = us5152_i2c_read(client,REGS_CR1) & 0x07;

	return data;
}

/* resolution for als*/
static int get_als_resolution(struct i2c_client *client)
{		
        return (us5152_i2c_read(client,REGS_CR1) & 0x18) >> 3;
}

static int set_als_resolution(struct i2c_client *client, int res)
{
        return us5152_i2c_write(client,REGS_CR1,
                CR1_ALS_RES_MASK, CR1_ALS_RES_SHIFT, res);
}

/* als fault queue depth for interrupt event output*/
static int get_als_fq(struct i2c_client *client)
{
        return (us5152_i2c_read(client,REGS_CR1) & 0xE0) >> 5;
}

static int set_als_fq(struct i2c_client *client, int depth)
{
        return us5152_i2c_write(client,REGS_CR1,
                CR1_ALS_FQ_MASK, CR1_ALS_FQ_SHIFT, depth);
}

/* int type*/
static int set_int_type(struct i2c_client *client, int type)
{
	
        return us5152_i2c_write(client,REGS_CR2,CR2_INT_MASK, 
              	CR2_INT_SHIFT,type);
}
static int get_int_type(struct i2c_client *client)
{
        u8 data;

	data = (us5152_i2c_read(client,REGS_CR2) & 0x20) >> 5;

	return data;
}

/* wait time slot selection */
static int set_wait_sel(struct i2c_client *client, int slot)
{
	
        return us5152_i2c_write(client,REGS_CR3,CR3_WAIT_SEL_MASK, 
              	CR3_WAIT_SEL_SHIFT,slot);
}
static int get_wait_sel(struct i2c_client *client)
{
        u8 data;

	data = (us5152_i2c_read(client,REGS_CR3) & 0xC0) >> 6;

	return data;
}


/* proximity IR_LED drive */
static int set_ps_drive(struct i2c_client *client, int drive)
{
	
        return us5152_i2c_write(client,REGS_CR3,CR3_LEDDR_MASK, 
              	CR3_LEDDR_SHIFT,drive);
}
static int get_ps_drive(struct i2c_client *client)
{
        u8 data;

	data = (us5152_i2c_read(client,REGS_CR3) & 0x30) >> 4;

	return data;
}

/* INT pine source selection */
static int set_int_sel(struct i2c_client *client, int source)
{
	
        return us5152_i2c_write(client,REGS_CR3,CR3_INT_SEL_MASK, 
              	CR3_INT_SEL_SHIFT,source);
}
static int get_int_sel(struct i2c_client *client)
{
        u8 data;

	data = (us5152_i2c_read(client,REGS_CR3) & 0x0C) >> 2;

	return data;
}

/* software reset for register and core */
static int set_software_reset(struct i2c_client *client)
{
	
        return us5152_i2c_write(client,REGS_CR3,CR3_INT_SEL_MASK, 
              	CR3_INT_SEL_SHIFT,CR3_SOFTRST_EN);
}

/*
static int get_software_reset(struct i2c_client *client)
{
        u8 data;

	data = us5152_i2c_read(client,REGS_CR3) & 0x01;

	return data;
}
*/

/*get chip id*/
static int get_chip_id(struct i2c_client *client)
{
        u8 data;
	data = us5152_i2c_read(client,REGS_CHIP_ID);
	if (data)
        	return data;
	else
		return 0;
}

/* 50/60Hz coupling rejection*/
static int get_rej_5060(struct i2c_client *client)
{
        return (us5152_i2c_read(client,REGS_CR10) & 0x01);
}

static int set_rej_5060(struct i2c_client *client, int enable)
{
        return us5152_i2c_write(client,REGS_CR10,
                CR10_REJ_5060_MASK, CR10_REJ_5060_SHIFT, enable);
}

/* modulation frequency of LED driver*/
static int get_freq_sel(struct i2c_client *client)
{
        return ((us5152_i2c_read(client,REGS_CR10) & 0x06) >> 1);
}

static int set_freq_sel(struct i2c_client *client, int clk)
{
        return us5152_i2c_write(client,REGS_CR10,
                CR10_FREQ_MASK, CR10_FREQ_SHIFT, clk);
}

static int get_comp_ledfreq(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);
        int value;

        mutex_lock(&data->lock);
        value = i2c_smbus_read_byte_data(client, REGS_CR11);	
	
	return value;
}

static int set_comp_ledfreq(struct i2c_client *client, int value)
{
        int ret = 0;
        struct us5152_data *data = i2c_get_clientdata(client);

        mutex_lock(&data->lock);
        ret = i2c_smbus_write_byte_data(client,REGS_CR11,
                                        value);
        if (ret < 0) {
                mutex_unlock(&data->lock);
                return ret;
        }

        reg_cache[REGS_CR11] = value;
        mutex_unlock(&data->lock);

        return ret;
}

static int get_als_lt(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);
        int lsb, msb, lt;

        mutex_lock(&data->lock);
        lsb = i2c_smbus_read_byte_data(client, REGS_INT_LSB_TH_LO);

        if (lsb < 0) {
                mutex_unlock(&data->lock);
                return lsb;
        }

        msb = i2c_smbus_read_byte_data(client, REGS_INT_MSB_TH_LO);
        mutex_unlock(&data->lock);

        if (msb < 0)
                return msb;

        lt = ((msb << 8) | lsb);

        return lt;
}

static int set_als_lt(struct i2c_client *client, int lt)
{
        int ret = 0;
        struct us5152_data *data = i2c_get_clientdata(client);

        mutex_lock(&data->lock);
        ret = i2c_smbus_write_byte_data(client, REGS_INT_LSB_TH_LO,
                                        lt & 0xff);
        if (ret < 0) {
                mutex_unlock(&data->lock);
                return ret;
        }

        ret = i2c_smbus_write_byte_data(client, REGS_INT_MSB_TH_LO,
                                        (lt >> 8) & 0xff);
        if (ret < 0) {
                mutex_unlock(&data->lock);
                return ret;
        }

        reg_cache[REGS_INT_MSB_TH_LO] = (lt >> 8) & 0xff;
        reg_cache[REGS_INT_LSB_TH_LO] = lt & 0xff;
        mutex_unlock(&data->lock);

        return ret;
}

static int get_als_ht(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);
        int lsb, msb, ht;

        mutex_lock(&data->lock);
        lsb = i2c_smbus_read_byte_data(client,REGS_INT_LSB_TH_HI);

        if (lsb < 0) {
                mutex_unlock(&data->lock);
                return lsb;
        }

        msb = i2c_smbus_read_byte_data(client, REGS_INT_MSB_TH_HI );
        mutex_unlock(&data->lock);

        if (msb < 0)
                return msb;

        ht = ((msb << 8) | lsb);

        return ht;
}

static int set_als_ht(struct i2c_client *client, int ht)
{
        int ret = 0;
        struct us5152_data *data = i2c_get_clientdata(client);

        mutex_lock(&data->lock);
        ret = i2c_smbus_write_byte_data(client,REGS_INT_LSB_TH_HI,
                                        ht & 0xff);
        if (ret < 0) {
                mutex_unlock(&data->lock);
                return ret;
        }

        ret = i2c_smbus_write_byte_data(client,REGS_INT_MSB_TH_HI,
                                        (ht >> 8) & 0xff);
        if (ret < 0) {
                mutex_unlock(&data->lock);
                return ret;
        }

        reg_cache[REGS_INT_MSB_TH_HI] = (ht >> 8) & 0xff;
        reg_cache[REGS_INT_LSB_TH_HI] = ht & 0xff;
        mutex_unlock(&data->lock);

        return ret;
}

static int get_als_value(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);
        int lsb, msb, bitdepth;

        mutex_lock(&data->lock);
        lsb = i2c_smbus_read_byte_data(client, REGS_LBS_SENSOR);

        if (lsb < 0) {
                mutex_unlock(&data->lock);
                return lsb;
        }

        msb = i2c_smbus_read_byte_data(client, REGS_MBS_SENSOR);
        mutex_unlock(&data->lock);

        if (msb < 0)
                return msb;

	bitdepth = get_als_resolution(client);
	switch(bitdepth){
	case 0:	
		lsb &= 0xF0;
		break;
	case 1:
		lsb &= 0xFC;
		break;
	}	
	
	return ((msb << 8) | lsb);
}

//=============================================================================
/* interrupt flag finley*/
static ssize_t us5152_show_int_flag(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%i\n", get_int_flag(client));
}

static ssize_t us5152_store_int_flag(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7))
                return -EINVAL;

        ret = set_int_flag(client, val);
        if (ret < 0)
                return ret;

        return count;
}
static DEVICE_ATTR(int_flag, S_IWUSR | S_IRUGO,
                   us5152_show_int_flag, us5152_store_int_flag);

/* als interrupt lt */
static ssize_t us5152_show_als_int_lt(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%i\n",get_als_lt(client));
}

static ssize_t us5152_store_als_int_lt(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 16, &val) < 0) || (val > 0xffff))
                return -EINVAL;

        ret = set_als_lt(client, val);
        if (ret < 0)
                return ret;

        return count;
}

static DEVICE_ATTR(als_int_lt, S_IWUSR | S_IRUGO,
                   us5152_show_als_int_lt, us5152_store_als_int_lt);

/* als interrupt ht */
static ssize_t us5152_show_als_int_ht(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%i\n", get_als_ht(client));
}

static ssize_t us5152_store_als_int_ht(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 16, &val) < 0) || (val > 0xffff))
                return -EINVAL;

        ret = set_als_ht(client, val);
        if (ret < 0)
                return ret;

        return count;
}

static DEVICE_ATTR(als_int_ht, S_IWUSR | S_IRUGO,
                   us5152_show_als_int_ht, us5152_store_als_int_ht);


/* als resolution */
static ssize_t us5152_show_als_resolution(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%d\n", get_als_resolution(client));
}

static ssize_t us5152_store_als_resolution(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 16))
                return -EINVAL;

        ret = set_als_resolution(client, val);
        if (ret < 0)
                return ret;

        return count;
}

static DEVICE_ATTR(als_resolution, S_IWUSR | S_IRUGO,
                   us5152_show_als_resolution, us5152_store_als_resolution);

/* one shot mode finley*/
static ssize_t us5152_show_oneshotmode(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%d\n", get_oneshotmode(client));
}

static ssize_t us5152_store_oneshotmode(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

	/*set 0 : continuous , set 1: one shot mode*/
        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
                return -EINVAL;

        ret = set_oneshotmode(client, val);
        if (ret < 0)
                return ret;

        return count;
}

static DEVICE_ATTR(oneshotmode, S_IWUSR | S_IRUGO,
                   us5152_show_oneshotmode, us5152_store_oneshotmode);


/*op mode */
static ssize_t us5152_show_opmode(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%d\n", get_opmode(client));
}

static ssize_t us5152_store_opmode(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7))
                return -EINVAL;

        ret = set_opmode(client, val);
        if (ret < 0)
                return ret;

        return count;
}

static DEVICE_ATTR(opmode, S_IWUSR | S_IRUGO,
                   us5152_show_opmode, us5152_store_opmode);

/* power_status finley*/
static ssize_t us5152_show_power_status(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%d\n", get_power_status(client));
}

static ssize_t us5152_store_power_status(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        int ret;

        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
                return -EINVAL;
        ret = set_power_status(client, val);
        return ret ? ret : count;
}

static DEVICE_ATTR(power_status, S_IWUSR | S_IRUGO,
                   us5152_show_power_status, us5152_store_power_status);

/* Chip_ID  finley*/
static ssize_t us5152_show_chip_id(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        return sprintf(buf, "%x\n", get_chip_id(client));
}


static DEVICE_ATTR(chip_id, S_IRUGO, us5152_show_chip_id, NULL);

/* LUX for als finley*/
static ssize_t us5152_show_als_lux(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);

        /* No LUX data if not operational */
        if (!get_power_status(client))
                return -EBUSY;

        return sprintf(buf, "%d\n",get_als_value(client));
}

static ssize_t us5152_store_als_lux(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val;
        
        if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
                return -EINVAL;
	printk("%s : cancelling poll timer\n", __func__);
	printk("val = %ld.\n", val);

	if(val != 0)
		us5152_light_enable(client);
	else
		us5152_light_disable(client);

	return 0;
}
static DEVICE_ATTR(als_lux, S_IWUSR | S_IRUGO, us5152_show_als_lux, us5152_store_als_lux);

static struct attribute *us5152_attributes[] = {
	&dev_attr_chip_id.attr,
        &dev_attr_als_lux.attr,
	&dev_attr_power_status.attr,
        &dev_attr_opmode.attr,
        &dev_attr_als_resolution.attr,
 	&dev_attr_als_int_lt.attr,
        &dev_attr_als_int_ht.attr,
        &dev_attr_int_flag.attr,

	NULL
};

static const struct attribute_group us5152_attr_group = {
        .attrs = us5152_attributes,
};

static int us5152_init_client(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);
        struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int i ;
	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
                printk(KERN_INFO "byte op is not permited.\n");
                return -EIO;
        }

        /* read all the registers once to fill the cache.
         * if one of the reads fails, we consider the init failed */
        for (i = 0; i < ARRAY_SIZE(reg_cache); i++) {
                int v = i2c_smbus_read_byte_data(client, i);
                if (v < 0)
                        return -ENODEV;
                reg_cache[i] = v;
        }

	/*Set Default*/
	//set_power_status(client,1);
	//set_mode(client,us5152_MODE_ALS_CONTINUOUS);	
	//set_resolution(client,us5152_RES_16);
	//set_range(client,3);
	//set_int_ht(client,0x3E8); //1000 lux
	//set_int_lt(client,0x8); //8 lux
	dev_info(&data->client->dev, "us5152 ver. %s found.\n",DRIVER_VERSION);
	return 0;
}

static void us5152_light_enable(struct i2c_client *client)
{
	struct us5152_data *data = i2c_get_clientdata(client);

	printk("%s : start poll timer, delay time is %lld ns\n", __func__,
		ktime_to_ns(data->light_poll_delay));

	/*push -1 to input subsystem to enable real value to go through next*/
	/*input_report_abs(data->input, ABS_MISC, -1);*/

	hrtimer_start(&data->timer, 
		ktime_set(0, LIGHT_SENSOR_START_TIME_DELAY), HRTIMER_MODE_REL);
}

static void us5152_light_disable(struct i2c_client *client)
{
	struct us5152_data *data = i2c_get_clientdata(client);
	printk("%s : cancelling poll timer\n", __func__);
	
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work);

	
}

static void us5152_work(struct work_struct *work)
{
        struct us5152_data *data =
                        container_of(work, struct us5152_data, work);
        struct i2c_client *client = data->client;
        int lux;

        lux = get_als_value(client);

	//printk("Lux: %d\n",lux);
        msleep(100);
	
	if(lux >= 0){
        	input_report_abs(data->input, ABS_MISC, lux);
        	input_sync(data->input);
	}
}

static enum hrtimer_restart us5152_timer_function(struct hrtimer *timer)
{
        struct us5152_data *data = container_of(timer, struct us5152_data, timer);
	
	queue_work(data->workqueue, &data->work);

	hrtimer_forward_now(&data->timer, data->light_poll_delay);

	return HRTIMER_RESTART;
}

static int __devinit us5152_i2c_probe(struct i2c_client *client,
                                    const struct i2c_device_id *id)
{
        struct us5152_data *data;
        struct input_dev *input_dev;
        int err = 0;
	
	 data = kzalloc(sizeof(struct us5152_data), GFP_KERNEL);
        if (!data) {
                err = -ENOMEM;
                goto exit;
        }
        mutex_init(&data->lock);
        data->client = client;
        i2c_set_clientdata(client, data);

	/* initialize the us5152 chip */
        err = us5152_init_client(client);
        if (err != 0)
              goto exit_free;


	/* register sysfs hooks */
        err = sysfs_create_group(&client->dev.kobj, &us5152_attr_group);
        if (err)
                goto exit_free;

	input_dev = input_allocate_device();
        if (!input_dev) {
                err = -ENOMEM;
                goto exit_free;
        }

        data->input = input_dev;
        input_dev->name = "lightsensor";
        input_dev->id.bustype = BUS_I2C;

        __set_bit(EV_ABS, input_dev->evbit);
        input_set_abs_params(input_dev, ABS_MISC, 0,
                        65535, 0, 0);

        err = input_register_device(input_dev);

	/*hrtimer setting, we poll for lux using a timer*/
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = us5152_timer_function;
	/*record poll value interval*/
	data->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);

        data->workqueue = create_singlethread_workqueue("us5152");
        INIT_WORK(&data->work, us5152_work);
        if (data->workqueue == NULL) {
                dev_err(&client->dev, "couldn't create workqueue\n");
                err = -ENOMEM;
                goto exit_free_interrupt;
        }

	printk("%s:ok\n",__func__);
	
        return 0;

exit_free_interrupt:
        free_irq(client->irq, data);
exit_free:
        printk("Error\n");
        kfree(data);
exit:
        return err;

}

static int __devexit us5152_i2c_remove(struct i2c_client *client)
{
        struct us5152_data *data = i2c_get_clientdata(client);

        set_power_status(client,0);
	destroy_workqueue(data->workqueue);
       // free_irq(IRQ_LIGHT_INT, data);
	input_unregister_device(data->input);
        input_free_device(data->input);
        sysfs_remove_group(&client->dev.kobj, &us5152_attr_group);
        kfree(i2c_get_clientdata(client));
	return 0;
}

#ifdef CONFIG_PM
static int us5152_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int us5152_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
	#define us5152_i2c_suspend        NULL
	#define us5152_i2c_resume         NULL
#endif

static const struct i2c_device_id us5152_i2c_id[] = {
        { us5152_DRV_NAME, 0 },
        {}
};
MODULE_DEVICE_TABLE(i2c, us5152_i2c_id);

static struct i2c_driver us5152_i2c_driver = {
        .driver = {
//                .owner  = THIS_MODULE,
                .name   = us5152_DRV_NAME,
        },
        .id_table = us5152_i2c_id,
        .suspend = us5152_i2c_suspend,
        .resume = us5152_i2c_resume,
        .probe  = us5152_i2c_probe,
        .remove = __devexit_p(us5152_i2c_remove),
};


static int __init sensor_init(void)
{
	pr_info("%s\n", __func__);
	i2c_add_driver(&us5152_i2c_driver);	
	return 0;
}

static void __exit sensor_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&us5152_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

