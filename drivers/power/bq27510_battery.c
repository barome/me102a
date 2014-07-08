/*
 * BQ27510 battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h> 
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>

#define DRIVER_VERSION			"2.1.0"
#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26
#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27500_REG_SOC			0x2c

#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27000_FLAG_CHGS		BIT(8)
#define BQ27500_FLAG_FC			BIT(9)
#define BQ27500_FLAG_OTD		BIT(14)
#define BQ27500_FLAG_OTC		BIT(15)
#define BQ27500_FLAG_BATDET     BIT(3)

/*define for firmware update*/
#define BSP_I2C_MAX_TRANSFER_LEN			128
#define BSP_MAX_ASC_PER_LINE				400
#define BSP_ENTER_ROM_MODE_CMD				0x00
#define BSP_ENTER_ROM_MODE_DATA 			0x0F00
#define BSP_ROM_MODE_I2C_ADDR				0x0B
#define BSP_NORMAL_MODE_I2C_ADDR			0x55
#define BSP_FIRMWARE_FILE_SIZE				(400*1024)

/*define for power detect*/
#define BATTERY_LOW_CAPACITY 2
#define BATTERY_LOW_VOLTAGE 3500000
#define BATTERY_RECHARGER_CAPACITY 97
#define BATTERY_LOW_TEMPRETURE 0
#define BATTERY_HIGH_TEMPRETURE 650

/*globle variable*/
struct i2c_client* g_bq27510_i2c_client = NULL;
static struct i2c_driver bq27510_battery_driver;

enum bq27510_reg {
	REG_FLAGS = 0,
	REG_SOC,
	REG_AI,
	REG_VOLT,
	REG_TEMP,
};
static int bq27510_reg_value[5] = {0};

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

/* If the system has several batteries we need a different name for each
 * of them...
 */
struct bq27510_device_info {
	struct device 		*dev;
	struct power_supply	bat;
	struct power_supply	ac;
	struct power_supply	usb;
	struct delayed_work work;
	struct i2c_client	*client;

	int bat_full;
	int bat_status;
	int bat_capacity;
	int bat_health;
	int bat_present;
	int bat_voltage;
	int bat_current;
	int bat_tempreture;

	unsigned int isInit;
	unsigned int interval;
	unsigned int dc_check_pin;
	unsigned int usb_check_pin;
	unsigned int bat_low_pin;
	//unsigned int chg_ok_pin;
	int wake_irq;
	struct delayed_work wakeup_work;
	struct wake_lock bat_low_lock;
	unsigned int bat_num;
};

/*globle variable*/
static struct bq27510_device_info *bq27510_di;

extern int get_bq24155_status(void);

int  virtual_battery_enable = 0;
static ssize_t battery_proc_write(struct file *file,const char __user *buffer,
			 size_t count, loff_t *ppos)
{
	char c;
	int rc;
	printk("USER:\n");
	printk("echo x >/proc/driver/power\n");
	printk("x=1,means just print log ||x=2,means log and data ||x= other,means close log\n");

	rc = get_user(c,buffer);
	if(rc)
		return rc;
		
	if(c == '1')
		virtual_battery_enable = 1;
	else if(c == '2')
		virtual_battery_enable = 2;
	else if(c == '3')
		virtual_battery_enable = 3;
	else 
		virtual_battery_enable = 0;
	printk("%s,count(%d),virtual_battery_enable(%d)\n",__FUNCTION__,(int)count,virtual_battery_enable);
	return count;
}

static const struct file_operations battery_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= battery_proc_write,
}; 

/*
 * Common code for BQ27510 devices read
 */
#define BQ27510_SPEED			300 * 1000
static DEFINE_MUTEX(battery_mutex);
static int bq27510_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	
	mutex_lock(&battery_mutex);
	ret = i2c_master_reg8_recv(client, reg, buf, len, BQ27510_SPEED);
	mutex_unlock(&battery_mutex);
	
	return ret; 
}

static int bq27510_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret; 

	mutex_lock(&battery_mutex);
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, BQ27510_SPEED);
	mutex_unlock(&battery_mutex);
	
	return ret;
}


/*
*	firmware update codes
*/
static int bq27510_read_and_compare(struct i2c_client *client, u8 reg, u8 *pSrcBuf, u8 *pDstBuf, u16 len)
{
	int i2c_ret;

	i2c_ret = bq27510_read(client, reg, pSrcBuf, len);
	if(i2c_ret < 0)
	{
		printk(KERN_ERR "[%s,%d] bq27510_read failed\n",__FUNCTION__,__LINE__);
		return i2c_ret;
	}

	i2c_ret = strncmp(pDstBuf, pSrcBuf, len);

	return i2c_ret;
}


static int bq27510_atoi(const char *s)
{
	int k = 0;

	k = 0;
	while (*s != '\0' && *s >= '0' && *s <= '9') {
		k = 10 * k + (*s - '0');
		s++;
	}
	return k;
}

static unsigned long bq27510_strtoul(const char *cp, unsigned int base)
{
	unsigned long result = 0,value;

	while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)
		? toupper(*cp) : *cp)-'A'+10) < base) 
	{
		result = result*base + value;
		cp++;
	}

	return result;
}

static int bq27510_firmware_program(struct i2c_client *client, const unsigned char *pgm_data, unsigned int filelen)
{
	unsigned int i = 0, j = 0, ulDelay = 0, ulReadNum = 0;
	unsigned int ulCounter = 0, ulLineLen = 0;
	unsigned char temp = 0; 
	unsigned char *p_cur;
	unsigned char pBuf[BSP_MAX_ASC_PER_LINE] = { 0 };
	unsigned char p_src[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
	unsigned char p_dst[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
	unsigned char ucTmpBuf[16] = { 0 };

bq275x0_firmware_program_begin:
	if(ulCounter > 10)
	{
		return -1;
	}
	
	p_cur = (unsigned char *)pgm_data;		 

	while(1)
	{
		if((p_cur - pgm_data) >= filelen)
		{
			printk("Download success\n");
			break;
		}
			
		while (*p_cur == '\r' || *p_cur == '\n')
		{
			p_cur++;
		}
		
		i = 0;
		ulLineLen = 0;

		memset(p_src, 0x00, sizeof(p_src));
		memset(p_dst, 0x00, sizeof(p_dst));
		memset(pBuf, 0x00, sizeof(pBuf));

		/*获取一行数据，去除空格*/
		while(i < BSP_MAX_ASC_PER_LINE)
		{
			temp = *p_cur++;	  
			i++;
			if(('\r' == temp) || ('\n' == temp))
			{
				break;	
			}
			if(' ' != temp)
			{
				pBuf[ulLineLen++] = temp;
			}
		}

		
		p_src[0] = pBuf[0];
		p_src[1] = pBuf[1];

		if(('W' == p_src[0]) || ('C' == p_src[0]))
		{
			for(i=2,j=0; i<ulLineLen; i+=2,j++)
			{
				memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
				memcpy(ucTmpBuf, pBuf+i, 2);
				p_src[2+j] = bq27510_strtoul(ucTmpBuf, 16);
			}

			temp = (ulLineLen -2)/2;
			ulLineLen = temp + 2;
		}
		else if('X' == p_src[0])
		{
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+2, ulLineLen-2);
			ulDelay = bq27510_atoi(ucTmpBuf);
		}
		else if('R' == p_src[0])
		{
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+2, 2);
			p_src[2] = bq27510_strtoul(ucTmpBuf, 16);
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+4, 2);
			p_src[3] = bq27510_strtoul(ucTmpBuf, 16);
			memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			memcpy(ucTmpBuf, pBuf+6, ulLineLen-6);
			ulReadNum = bq27510_atoi(ucTmpBuf);
		}

		if(':' == p_src[1])
		{
			switch(p_src[0])
			{
				case 'W' :

					#if 0
					printk("W: ");
					for(i=0; i<ulLineLen-4; i++)
					{
						printk("%x ", p_src[4+i]);
					}
					printk(KERN_ERR "\n");
					#endif					  

					if(bq27510_write(client, p_src[3], &p_src[4], ulLineLen-4) < 0)
					{
						 printk(KERN_ERR "[%s,%d] bq27510_write failed len=%d\n",__FUNCTION__,__LINE__,ulLineLen-4);						  
					}

					break;
				
				case 'R' :
					if(bq27510_read(client, p_src[3], p_dst, ulReadNum) < 0)
					{
						printk(KERN_ERR "[%s,%d] bq275x0_i2c_bytes_read failed\n",__FUNCTION__,__LINE__);
					}
					break;
					
				case 'C' :
					if(bq27510_read_and_compare(client, p_src[3], p_dst, &p_src[4], ulLineLen-4))
					{
						ulCounter++;
						printk(KERN_ERR "[%s,%d] bq275x0_i2c_bytes_read_and_compare failed\n",__FUNCTION__,__LINE__);
						goto bq275x0_firmware_program_begin;
					}
					break;
					
				case 'X' :					  
					mdelay(ulDelay);
					break;
				  
				default:
					return 0;
			}
		}
	  
	}

	return 0;
	
}

static int bq27510_firmware_download(struct i2c_client *client, const unsigned char *pgm_data, unsigned int len)
{
	int iRet;
	unsigned char ucTmpBuf[2] = { 0 };

	ucTmpBuf[0] = BSP_ENTER_ROM_MODE_DATA & 0x00ff;
	ucTmpBuf[1] = (BSP_ENTER_ROM_MODE_DATA>>8) & 0x00ff;
	
	/*Enter Rom Mode */
	iRet = bq27510_write(client, BSP_ENTER_ROM_MODE_CMD, &ucTmpBuf[0], 2);
	if(0 > iRet)
	{
		printk(KERN_ERR "[%s,%d] bq27510_write failed\n",__FUNCTION__,__LINE__);
	}
	mdelay(10);

	/*change i2c addr*/
	g_bq27510_i2c_client->addr = BSP_ROM_MODE_I2C_ADDR;

	/*program bqfs*/
	iRet = bq27510_firmware_program(client, pgm_data, len);
	if(0 != iRet)
	{
		printk(KERN_ERR "[%s,%d] bq275x0_firmware_program failed\n",__FUNCTION__,__LINE__);
	}

	/*change i2c addr*/
	g_bq27510_i2c_client->addr = BSP_NORMAL_MODE_I2C_ADDR;

	return iRet;
	
}

static int bq27510_update_firmware(struct i2c_client *client, const char *pFilePath) 
{
	char *buf;
	struct file *filp;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	unsigned int length;
	int ret = 0;

	/* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(pFilePath, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp)) 
	{
		printk(KERN_ERR "[%s,%d] filp_open failed\n",__FUNCTION__,__LINE__);
		set_fs(oldfs);
		return -1;
	}

	if (!filp->f_op) 
	{
		printk(KERN_ERR "[%s,%d] File Operation Method Error\n",__FUNCTION__,__LINE__); 	   
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) 
	{
		printk(KERN_ERR "[%s,%d] Get inode from filp failed\n",__FUNCTION__,__LINE__);			
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	printk("bq27510 firmware image size is %d \n",length);
	if (!( length > 0 && length < BSP_FIRMWARE_FILE_SIZE))
	{
		printk(KERN_ERR "[%s,%d] Get file size error\n",__FUNCTION__,__LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* allocation buff size */
	buf = vmalloc(length+(length%2));		/* buf size if even */
	if (!buf) 
	{
		printk(KERN_ERR "[%s,%d] Alloctation memory failed\n",__FUNCTION__,__LINE__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* read data */
	if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length) 
	{
		printk(KERN_ERR "[%s,%d] File read error\n",__FUNCTION__,__LINE__);
		filp_close(filp, NULL);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		return -1;
	}

	ret = bq27510_firmware_download(client, (const char*)buf, length);

	if(0 == ret)
		ret = 1;

	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);
	
	return ret;
}

static u8 get_child_version(void)
{
	u8 data[32];
	
	data[0] = 0x39;
	if(bq27510_write(g_bq27510_i2c_client, 0x3e, data, 1) < 0)
		return -1;
	mdelay(2);

	data[0] = 0x00;
	if(bq27510_write(g_bq27510_i2c_client, 0x3f, data, 1) < 0)
		return -1;
	mdelay(2);

	data[0] = 0x00;
	if(bq27510_write(g_bq27510_i2c_client, 0x61, data, 1) < 0)
		return -1;
	mdelay(2);

	bq27510_read(g_bq27510_i2c_client, 0x60, data, 1);
	mdelay(2);

	bq27510_read(g_bq27510_i2c_client, 0x40, data, 32);
	
	return data[0];
}

static ssize_t bq27510_attr_store(struct device_driver *driver,const char *buf, size_t count)
{
	int iRet = 0;
	unsigned char path_image[255];

	if(NULL == buf || count >255 || count == 0 || strnchr(buf, count, 0x20))
		return -1;

	memcpy (path_image, buf,  count);
	/* replace '\n' with  '\0'	*/ 
	if((path_image[count-1]) == '\n')
		path_image[count-1] = '\0'; 
	else
		path_image[count] = '\0';		

	/*enter firmware bqfs download*/
	virtual_battery_enable = 1;
	iRet = bq27510_update_firmware(g_bq27510_i2c_client, path_image);		
	msleep(3000);
	pr_err("Update firemware finish, then update battery status...");		
	virtual_battery_enable = 0;

	return iRet;
}

static ssize_t bq27510_attr_show(struct device_driver *driver, char *buf)
{
	u8 ver;
	
	if(NULL == buf)
	{
		return -1;
	}

	ver = get_child_version();

	if(ver < 0)
	{
		return sprintf(buf, "%s", "Coulometer Damaged or Firmware Error");
	}
	else
	{	 
	
		return sprintf(buf, "%x", ver);
	}

}

static DRIVER_ATTR(state, 0664, bq27510_attr_show, bq27510_attr_store);


int bq27510_battery_status_output(void)
{
	if (gpio_get_value(bq27510_di->dc_check_pin) && gpio_get_value(bq27510_di->usb_check_pin))
		return 0;	/*discharging*/
	else
		return 1;	/*charging*/
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27510_battery_tempreture(struct bq27510_device_info *di)
{
	int ret;
	int temp = 0;
	u8 buf[2];

	#if defined (CONFIG_NO_BATTERY_IC)
	return 258;
	#endif

	if(virtual_battery_enable == 1)
		return 125/*258*/;
	ret = bq27510_read(di->client,BQ27x00_REG_TEMP,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}
	temp = get_unaligned_le16(buf);
	bq27510_reg_value[REG_TEMP] = temp;
	temp = temp - 2731;
	DBG("Enter:%s %d--temp = %d\n",__FUNCTION__,__LINE__,temp);
	return temp;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27510_battery_voltage(struct bq27510_device_info *di)
{
	int ret;
	u8 buf[2];
	int volt = 0;

	#if defined (CONFIG_NO_BATTERY_IC)
		return 4000000;
	#endif
	if(virtual_battery_enable == 1)
		return 4000000;

	ret = bq27510_read(di->client,BQ27x00_REG_VOLT,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}
	volt = get_unaligned_le16(buf);
	bq27510_reg_value[REG_VOLT] = volt;
	//bp27510 can only measure one li-lion bat
	if(di->bat_num == 2){
		volt = volt * 1000 * 2;
	}else{
		volt = volt * 1000;
	}

	DBG("Enter:%s %d--volt = %d\n",__FUNCTION__,__LINE__,volt);
	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27510_battery_current(struct bq27510_device_info *di)
{
	int ret;
	int curr = 0;
	u8 buf[2];

	#if defined (CONFIG_NO_BATTERY_IC)
		return 22000;
	#endif
	if(virtual_battery_enable == 1)
		return 11000/*22000*/;
	ret = bq27510_read(di->client,BQ27x00_REG_AI,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = get_unaligned_le16(buf);
	bq27510_reg_value[REG_AI] = curr;
	DBG("curr = %x \n",curr);
	if(curr>0x8000){
		curr = 0xFFFF^(curr-1);
	}
	curr = curr * 1000;
	DBG("Enter:%s %d--curr = %d\n",__FUNCTION__,__LINE__,curr);
	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27510_battery_rsoc(struct bq27510_device_info *di)
{
	int ret;
	int rsoc = 0;
	u8 buf[2];

	#if defined (CONFIG_NO_BATTERY_IC)
		return 100;
	#endif
	
	if(virtual_battery_enable == 1)
		return 50/*100*/;
	
	ret = bq27510_read(di->client,BQ27500_REG_SOC,buf,2); 
	if (ret<0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}
	rsoc = get_unaligned_le16(buf);
	bq27510_reg_value[REG_SOC] = rsoc;
	DBG("Enter:%s %d--rsoc = %d\n",__FUNCTION__,__LINE__,rsoc);
	
	return rsoc;
}

static int bq27510_battery_status(struct bq27510_device_info *di)
{
	u8 buf[2];
	int flags = 0;
	int status;
	int ret;

	#if defined (CONFIG_NO_BATTERY_IC)
	return POWER_SUPPLY_STATUS_FULL;
	#endif

	if(virtual_battery_enable == 1)
	{
		return POWER_SUPPLY_STATUS_FULL;
	}
	ret = bq27510_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	flags = get_unaligned_le16(buf);
	bq27510_reg_value[REG_FLAGS] = flags;
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,flags);
	if (flags & BQ27500_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & BQ27500_FLAG_DSC)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	if (((status==POWER_SUPPLY_STATUS_FULL)||(status==POWER_SUPPLY_STATUS_CHARGING)) 
		&& (!bq27510_battery_status_output()))
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if ((status==POWER_SUPPLY_STATUS_DISCHARGING) && (bq27510_battery_status_output()))
		status = POWER_SUPPLY_STATUS_CHARGING;

	return status;
}

static int bq27510_health_status(struct bq27510_device_info *di)
{
	u8 buf[2];
	int flags = 0;
	int status;
	int ret;
	
	#if defined (CONFIG_NO_BATTERY_IC)
		return POWER_SUPPLY_HEALTH_GOOD;
	#endif

	if(virtual_battery_enable == 1)
	{
		return POWER_SUPPLY_HEALTH_GOOD;
	}
	ret = bq27510_read(di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}
	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,flags);
	if ((flags & BQ27500_FLAG_OTD)||(flags & BQ27500_FLAG_OTC))
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;
}


/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27510_battery_time(struct bq27510_device_info *di, int reg,
				union power_supply_propval *val)
{
	u8 buf[2];
	int tval = 0;
	int ret;
	
	if(virtual_battery_enable == 1) {
		val->intval = 60;
		return 0;
	}

	ret = bq27510_read(di->client,reg,buf,2);
	if (ret<0) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}
	tval = get_unaligned_le16(buf);
	DBG("Enter:%s %d--tval=%d\n",__FUNCTION__,__LINE__,tval);
	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	DBG("Enter:%s %d val->intval = %d\n",__FUNCTION__,__LINE__,val->intval);
	return 0;
}

int bq27510_battery_present(void)
{
	u8 buf[2];
	int flags = 0;
	int ret;
	
	if(virtual_battery_enable == 1) {
		return 1;
	}

	ret = bq27510_read(bq27510_di->client,BQ27x00_REG_FLAGS, buf, 2);
	if (ret < 0) {
		dev_err(bq27510_di->dev, "error reading flags\n");
		return ret;
	}
	flags = get_unaligned_le16(buf);
	DBG("Enter:%s %d--status = %x\n",__FUNCTION__,__LINE__,flags);
	if (flags & BQ27500_FLAG_BATDET) {
		//printk("bq27510 think battery present\n");
		return 1;
	}
	else {
		//printk("bq27510 think no battery present\n");
		return 0;
	}
}

#define to_bq27510_device_info(x) container_of((x), \
				struct bq27510_device_info, bat);

static int bq27510_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27510_device_info *di = to_bq27510_device_info(psy);
	DBG("Enter:%s %d psp= %d\n",__FUNCTION__,__LINE__,psp);
	
	switch (psp) {
	
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->bat_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->bat_voltage;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->bat_present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->bat_current;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		//if (di->bat_voltage <= BATTERY_LOW_VOLTAGE) {
		//	printk(KERN_INFO "battery now voltage is %d, power off voltage is %d\n", di->bat_voltage, BATTERY_LOW_VOLTAGE);
		//	val->intval = 0;
		//}
		//else {
			val->intval = di->bat_capacity;
		//}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->bat_tempreture;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->bat_health;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
static enum power_supply_property bq27510_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
};


static int bq_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	DBG("%s:%d psp = %d\n",__FUNCTION__,__LINE__,psp);
		
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:	
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if ((get_bq24155_status() == POWER_SUPPLY_STATUS_CHARGING )&& (bq27510_battery_present()==1))
				val->intval = 1;	/*charging*/
			else
				val->intval = 0;	/*discharging*/
		}
		DBG("%s:%d val->intval = %d\n",__FUNCTION__,__LINE__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
static enum power_supply_property bq_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};



static int bq_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	DBG("%s:%d psp = %d\n",__FUNCTION__,__LINE__,psp);
		
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:	
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if ((get_bq24155_status() == POWER_SUPPLY_STATUS_CHARGING ) && (bq27510_battery_present()==1))
				val->intval = 1;	/*charging*/
			else
				val->intval = 0;	/*discharging*/
		}
		DBG("%s:%d val->intval = %d\n",__FUNCTION__,__LINE__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
static enum power_supply_property bq_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static void bq27510_powersupply_init(struct bq27510_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27510_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27510_battery_props);
	di->bat.get_property = bq27510_battery_get_property;
	
	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = bq_ac_props;
	di->ac.num_properties = ARRAY_SIZE(bq_ac_props);
	di->ac.get_property = bq_ac_get_property;

	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = bq_usb_props;
	di->usb.num_properties = ARRAY_SIZE(bq_usb_props);
	di->usb.get_property = bq_usb_get_property;
}


static void bq27510_battery_update_status(struct bq27510_device_info *di)
{
	power_supply_changed(&di->bat);
	power_supply_changed(&di->ac);
	power_supply_changed(&di->usb);

}

static void bq27510_get_battery_info(struct bq27510_device_info *di)
{
	di->bat_status = bq27510_battery_status(di);
	di->bat_voltage = bq27510_battery_voltage(di);
	di->bat_present = bq27510_battery_present();
	di->bat_current = bq27510_battery_current(di);
	di->bat_capacity = bq27510_battery_rsoc(di);
	di->bat_tempreture = bq27510_battery_tempreture(di);
	di->bat_health = bq27510_health_status(di);
}

static void bq27510_print_battery_info(struct bq27510_device_info *di)
{
	static char *status_text[] = {
		"Unknown", "Charging", "Discharging", "Not charging", "Full"
	};

	printk("bq27510 regs:\n");
	printk("FLAG      SOC       AI       VOLT      TEMP\n");
	printk("%d        %d        %d       %d        %d\n", bq27510_reg_value[REG_FLAGS], 
															bq27510_reg_value[REG_SOC], bq27510_reg_value[REG_AI], 
															bq27510_reg_value[REG_VOLT], bq27510_reg_value[REG_TEMP]);
	printk("\n");
	printk("POWER_SUPPLY_NAME=bq27510-battery\n");
	printk("POWER_SUPPLY_STATUS=%s\n", status_text[di->bat_status]);
	printk("POWER_SUPPLY_PRESENT=%d\n", di->bat_present);
	printk("POWER_SUPPLY_VOLTAGE_NOW=%d\n", di->bat_voltage);
	printk("POWER_SUPPLY_CURRENT_NOW=%d\n", di->bat_current);
	printk("POWER_SUPPLY_CAPACITY=%d\n", di->bat_capacity);
	printk("POWER_SUPPLY_TEMP=%d\n", di->bat_tempreture);
	printk("\n");
}

#ifdef CONFIG_CHARGER_BQ24161
uint8_t otg_is_host_mode(void);
extern int bq24161_device_deinit(void);
extern int bq24161_device_init(int te_enable, int chg_enable, int otg_lock);
extern void bq24161_print_reg(void);
extern int mhl_vbus_power_on(void);
static void bq27510_for_charging(struct bq27510_device_info *di)
{			
	if (bq27510_battery_status_output() && (di->bat_present == 1)) {//有充电器接入，且电池存在
		//tempreture out of safe range, do not charge
		if ((di->bat_tempreture < BATTERY_LOW_TEMPRETURE) 
			|| (di->bat_tempreture > BATTERY_HIGH_TEMPRETURE)) {
			printk(KERN_INFO "battery tempreture is out of safe range\n");
			di->bat_full = 0;
			bq24161_device_init(1, 0, 0);//disable charging
			return ;
		}

		bq24161_device_init(1, 1, 0);

		if (di->bat_status==POWER_SUPPLY_STATUS_FULL) {//充满
			di->bat_full = 1;
			printk(KERN_INFO "**********charger ok*********\n");
		}
		else if ((di->bat_full==1) && (di->bat_capacity<=BATTERY_RECHARGER_CAPACITY)) {//已充满过，且电量小于95%，需要续充
			bq24161_device_init(1, 0, 0);
			msleep(1000);
			bq24161_device_init(1, 1, 0);
			di->bat_full = 0;
			printk(KERN_INFO "**********recharger*********\n");
		}
	}
	else if (otg_is_host_mode() || mhl_vbus_power_on()) {
		bq24161_device_init(1, 0, 1);
	}
	else {
		di->bat_full = 0;
		bq24161_device_deinit();
	}
}

#endif

static void bq27510_battery_work(struct work_struct *work)
{
	struct bq27510_device_info *di = container_of(work, struct bq27510_device_info, work.work); 
			
	bq27510_get_battery_info(di);
	bq27510_battery_update_status(di);
//	bq27510_print_battery_info(di);
	#ifdef CONFIG_CHARGER_BQ24161
	bq27510_for_charging(di);
//	bq24161_print_reg();
	#endif

	/* reschedule for the next time */
	schedule_delayed_work(&di->work, di->interval);
}

void bq27510_battery_info_update(int sec)
{
	if (bq27510_di && bq27510_di->isInit) {
		cancel_delayed_work_sync(&bq27510_di->work);
		schedule_delayed_work(&bq27510_di->work, sec * HZ);
	}
}

static irqreturn_t bq27510_bat_wakeup(int irq, void *dev_id)
{	
	struct bq27510_device_info *di = (struct bq27510_device_info *)dev_id;

	printk("!!!  bq27510 bat_low irq low vol !!!\n\n\n");
	
	wake_lock_timeout(&di->bat_low_lock, 10 * HZ);

	schedule_delayed_work(&di->wakeup_work, HZ / 10);	
	return IRQ_HANDLED;
}

static void bq27510_battery_wake_work(struct work_struct *work)
{
	cancel_delayed_work_sync(&bq27510_di->work);
	schedule_delayed_work(&bq27510_di->work, 0);
	rk28_send_wakeup_key();
}

static int bq27510_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	cancel_delayed_work_sync(&bq27510_di->work);
	return 0;
}

static int bq27510_battery_resume(struct i2c_client *client)
{
	schedule_delayed_work(&bq27510_di->work, 0);
	return 0;
}

static int bq27510_is_in_rom_mode(void)
{
	int ret = 0;
	unsigned char data = 0x0f;
	
	bq27510_di->client->addr = BSP_ROM_MODE_I2C_ADDR;
	ret = bq27510_write(bq27510_di->client, 0x00, &data, 1);
	bq27510_di->client->addr = BSP_NORMAL_MODE_I2C_ADDR;

	if (ret == 1)
		return 1;
	else 
		return 0;
}

static void bq27510_low_power_detect(struct bq27510_device_info *di)
{
	int retval = 0;
	int timeout = 0;
	int iTestRomMod = 0;
	int charger_status;
	
	
#ifndef CONFIG_NO_BATTERY_IC 

	#ifdef CONFIG_BATTERY_BQ2415x
	//bq24161_device_init(1, 1, 0);
	//bq24161_device_deinit();
	charger_status = get_bq24155_status();
	
	#endif

	while (1) {
        //battery present ?
        retval = bq27510_battery_present();
        if (retval == 1) {
            printk("bq27510 think battery present\n");
        }
        else if (retval == 0) {
            printk("bq27510 think no battery present\n");
            break;
        }
		else if (retval < 0 && iTestRomMod == 0) {// maybe in rom mode, you need updata firmware to exit rom mode
			iTestRomMod = 1;
			if (bq27510_is_in_rom_mode()) {
				printk(KERN_INFO "bq27510 is in rom mode, you need to update firmware to exit rom mode\n");
				break;
			}
		}

		//voltage detect
		retval = bq27510_battery_voltage(di);
		printk("battery now voltage = %d!!!!!!!!\n", retval);
		if (retval < 0) {
			if (!bq27510_battery_status_output()) {//discharging
				printk("!!!!!!!!!!battery low!!!!!!!!!!power off!!!!!!!\n");
				system_state = SYSTEM_POWER_OFF;
				pm_power_off();
			}
			else {//try to precharging
				#if 0
				#ifdef CONFIG_CHARGER_BQ24161
				printk(KERN_INFO "!!!!!!!battery precharging!!!!!!!\n");
				bq24161_device_init(0, 1, 0);
				bq24161_print_reg();
				#endif
				#endif
				
			}
		}
		else if ((retval <= BATTERY_LOW_VOLTAGE) && (timeout == 0)) {
			if (!bq27510_battery_status_output()) {//discharging
				printk("!!!!!!!!!!battery voltage lower to %d, !!!!!!!!!!power off!!!!!!!\n", BATTERY_LOW_VOLTAGE);
				system_state = SYSTEM_POWER_OFF;
				pm_power_off();
			}
			else {
				break;
			}
		}
		else if (timeout > 0){
			system_state = SYSTEM_POWER_OFF;
			pm_power_off();
		}
		else {
			break;
		}

		timeout++;
		if (timeout > 4) {
			printk(KERN_INFO "!!!!!!precharging battery timeout!!!!!!\n");
			#ifdef CONFIG_CHARGER_BQ24161
			bq24161_device_deinit();
			#endif
			break;
		}
		msleep(30*1000);
	}
#endif

	return ;
}

static int bq27510_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27510_device_info *di;
	int retval = 0;
	struct bq27510_platform_data *pdata;

	pdata = client->dev.platform_data;
	
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = "bq27510-battery";
	di->client = client;
	/* 30 seconds between monotor runs interval */
	di->interval = msecs_to_jiffies(30 * 1000);
	di->bat_num = pdata->bat_num;
	di->dc_check_pin = pdata->dc_check_pin;
	di->bat_low_pin = pdata->bat_low_pin;
	//di->chg_ok_pin = pdata->chgok_pin;
	di->usb_check_pin = pdata->usb_check_pin;
		
	//if (pdata->init_dc_check_pin)
      //  	pdata->init_dc_check_pin( );

	if (pdata->io_init)
		pdata->io_init( );
	
	bq27510_di = di;
	di->bat_full = 0;

	bq27510_low_power_detect(di);

	bq27510_get_battery_info(di);
	bq27510_powersupply_init(di);
	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed;
	}
	retval = power_supply_register(&client->dev, &di->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac\n");
		goto batt_failed_1;
	}

	retval = power_supply_register(&client->dev, &di->usb);
	if (retval) {
		dev_err(&client->dev, "failed to register usb\n");
		goto batt_failed_0;
	}

	g_bq27510_i2c_client = client;
		
	retval = driver_create_file(&(bq27510_battery_driver.driver), &driver_attr_state);
	if (0 != retval)
	{
		printk("failed to create sysfs entry(state): %d\n", retval);
		goto batt_failed_2;
	}

	INIT_DELAYED_WORK(&di->work, bq27510_battery_work);
	schedule_delayed_work(&di->work, msecs_to_jiffies(5*1000));
	
	// battery low irq
	di->wake_irq = gpio_to_irq(pdata->bat_low_pin);
	retval = request_irq(di->wake_irq, bq27510_bat_wakeup, IRQF_TRIGGER_FALLING, "bq27510_battery", di);
	if (retval) {
		printk("bq27510: failed to request bat det irq\n");
		goto batt_failed_3;
	}
	
	wake_lock_init(&di->bat_low_lock, WAKE_LOCK_SUSPEND, "bat_low");
	INIT_DELAYED_WORK(&di->wakeup_work, bq27510_battery_wake_work);
	enable_irq_wake(di->wake_irq);

	di->isInit = 1;
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	return 0;
	
batt_failed_3:
	driver_remove_file(&(bq27510_battery_driver.driver), &driver_attr_state);
batt_failed_2:
	power_supply_unregister(&di->ac);
batt_failed_1:
	power_supply_unregister(&di->bat);
batt_failed_0:
	power_supply_unregister(&di->usb);
batt_failed:
	kfree(di);
	return retval;
}

static int bq27510_battery_remove(struct i2c_client *client)
{
	struct bq27510_device_info *di = i2c_get_clientdata(client);

	wake_lock_destroy(&di->bat_low_lock);
	free_irq(di->wake_irq, di);
	driver_remove_file(&(bq27510_battery_driver.driver), &driver_attr_state);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	kfree(di->bat.name);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27510_id[] = {
	{ "bq27510", 0 },
};

static struct i2c_driver bq27510_battery_driver = {
	.driver = {
		.name = "bq27510",
	},
	.probe = bq27510_battery_probe,
	.remove = bq27510_battery_remove,
	.suspend = bq27510_battery_suspend,
	.resume = bq27510_battery_resume,
	.id_table = bq27510_id,
};

static int __init bq27510_battery_init(void)
{
	int ret;
	struct proc_dir_entry * battery_proc_entry;
	
	ret = i2c_add_driver(&bq27510_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27510 driver\n");
	
	battery_proc_entry = proc_create("driver/power",0777,NULL,&battery_proc_fops);
	return ret;
}

module_init(bq27510_battery_init);
//fs_initcall_sync(bq27510_battery_init);
static void __exit bq27510_battery_exit(void)
{
	i2c_del_driver(&bq27510_battery_driver);
}
module_exit(bq27510_battery_exit);

MODULE_AUTHOR("Rockchip");
MODULE_DESCRIPTION("BQ27510 battery monitor driver");
MODULE_LICENSE("GPL");
