/* 
 * Source for : Waltop ASIC5 pen touch controller.
 * drivers/input/tablet/waltop_I2C.c
 * 
 * Copyright (C) 2008-2013	Waltop International Corp. <waltopRD@waltop.com.tw>
 * 
 * History:
 * Copyright (c) 2011	Martin Chen <MartinChen@waltop.com.tw>
 * Copyright (c) 2012	Taylor Chuang <chuang.pochieh@gmail.com>
 * Copyright (c) 2012	Herman Han <HermanHan@waltop.com>
 * * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */

#include <linux/unistd.h>  
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/input/waltop_i2c.h>

#include <mach/iomux.h>

/*****************************************************************************
 * MACRO definitions and structure
 ****************************************************************************/
//#define PEN_ASIC_X6_VERSION	// now X7 version, this is for X6 version
#define REMAP_TO_LCD_SIZE     // mapping to screen resolution
#define IAP_FWUPDATE			// firmware update code include


#ifdef IAP_FWUPDATE
#include <linux/wakelock.h>
#include <linux/wait.h>
#include <linux/sysfs.h>
//
#define FW_IODATA_SIZE		8	// 8 bytes data
#define FW_IOBUFFER_SIZE	16	// 10 bytes I2C packet
#define FW_BLOCK_SIZE		128	// 128 bytes buffer for user ap
#define FW_RESET_DELAY_A2B	30	/* Delay 30 ms from A to B */
#define FW_RESET_DELAY_B2C	50	/* Delay 50 ms from B to C */
#define FW_RESET_DELAY_CP	5	/* Delay 5 ms after C */
#define USE_WAKELOCK
#endif

#ifdef	REMAP_TO_LCD_SIZE
/* Screen characteristics */
#define LCD_SCREEN_MAX_X		   1280	
#define LCD_SCREEN_MAX_Y		   800
#endif

#define I2C_SCL_RATE				100 * 1000

struct waltop_I2C
{
	struct i2c_client	*client;
	struct input_dev	*input;
	struct work_struct	work;
	struct timer_list	timer;

	char	phys[32];
	atomic_t irq_enabled;
	// Minimun value of X,Y,P are 0
	__u16	x_max;      // X maximun value
	__u16	y_max;      // Y maximun value
	__u16	p_max;      // Pressure maximun value
	__u16	p_minTipOn;	// minimun tip on pressure
	__u16	fw_version;	// 110 means 1.10

	__u8	pkt_data[16];// packets data buffer

	// Ensures that only one function can specify the Device Mode at a time
	struct mutex mutex;	// reentrant protection for struct
	unsigned int delaytime;
	//bool disabled;
};



/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/
void waltop_I2C_worker(struct work_struct *work);
static irqreturn_t waltop_I2C_irq(int irq, void *handle);

static int waltop_I2C_read(struct waltop_I2C *tp);

static int __devexit waltop_I2C_remove(struct i2c_client *client);
static int __devinit waltop_I2C_probe(struct i2c_client *client, const struct i2c_device_id *id);



/*****************************************************************************
 * Global Variables
 ****************************************************************************/
static struct workqueue_struct *waltop_I2C_wq;

static const struct i2c_device_id waltop_I2C_idtable[] = {
	{ "waltop_I2C", 0 },
	{ }
};

static struct i2c_driver waltop_I2C_driver = {
	.driver = {
		.name	= "waltop_I2C",
		.owner	= THIS_MODULE,
	},
	.id_table	= waltop_I2C_idtable,
	.probe		= waltop_I2C_probe,
	.remove	= __devexit_p(waltop_I2C_remove),
	
	/* shutdown/suspend/resume methods are not included now */
	.suspend = NULL,
	.resume = NULL,
};

#ifdef IAP_FWUPDATE
static DECLARE_WAIT_QUEUE_HEAD(iap_wait_queue_head);
#ifdef USE_WAKELOCK
struct wake_lock iap_wake_lock;
#endif
static int wait_queue_flag=0;
static int m_loop_write_flag=0;
static int m_request_count=0;	// request bytes counter
static int m_iap_fw_updating=0;	// 1 means updating
static int m_iapIsReadCmd=0;
static int m_iapPageCount=0;
static int m_fw_cmdAckValue=0;	// fw command ack value
static char iap_fw_status[64];	// fw update status string
#endif



/*****************************************************************************
 * I2C read functions
 ****************************************************************************/
static int waltop_I2C_read(struct waltop_I2C *tp)
{
	int ret = -1;
    struct i2c_msg msg;

//  2012/12, Martin, This is only for X6, 
#ifdef PEN_ASIC_X6_VERSION
	unsigned char buf[1];
  	//20120614 Herman : sent COMMAND 0x4F to Pen >>>>>
	buf[0] = 0x4F;
  	//20120614 split to two messages for X6
    msg.addr = tp->client->addr;
    msg.flags = 0; //Write
    msg.len = 1;
    msg.buf = (unsigned char *)buf;
	msg.scl_rate = I2C_SCL_RATE;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
#endif
//
	msg.addr = tp->client->addr;
    msg.flags = I2C_M_RD; //Read
	msg.len = 8;
	msg.buf = tp->pkt_data;
	msg.scl_rate = I2C_SCL_RATE;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = 8;
 	}
 	else {// some error
		printk(KERN_ERR "%s failed?-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}
	return ret;
}

static int waltop_I2C_readDeviceInfo(struct waltop_I2C *tp)
{
	__u8 sum = 0;
	int i, ret = -1;
    struct i2c_msg msg;
	unsigned char buf[1];

  	//sent COMMAND 0x2A to Pen >>>>>
	buf[0] = 0x2A;
    msg.addr = tp->client->addr;
    msg.flags = 0; //Write
    msg.len = 1;
    msg.buf = (unsigned char *)buf;
    msg.scl_rate = I2C_SCL_RATE;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1)	// 1 msg sent OK
 	{
		// Delay 1 ms, wait for f/w device data ready
		mdelay(1);
		//read back device information
		msg.addr = tp->client->addr;
		msg.flags = I2C_M_RD; //Read
		msg.len = 9;
		msg.buf = tp->pkt_data;
		msg.scl_rate = I2C_SCL_RATE;
		ret = i2c_transfer(tp->client->adapter, &msg, 1);

		if( ret == 1) {	// 1 msg sent OK
			// Check checksum
			for(i=1; i<8; i++) // D1 to D7
				sum = sum + tp->pkt_data[i];
			if( sum == tp->pkt_data[8] )
				ret = 9;
			else {
				ret = -2;
				printk(KERN_ERR "%s Checksum error!-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
			}
		}
		else {// some error
			printk(KERN_ERR "%s failed?-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
		}
	}
	return ret;
}

#ifdef IAP_FWUPDATE
/*****************************************************************************
 * Firmware update related finctions
 ****************************************************************************/
static int I2C_read_func(struct waltop_I2C *tp, unsigned char *read_buf, int read_count)
{
	int ret = -1;
    struct i2c_msg msg;

	msg.addr = tp->client->addr;
    msg.flags = I2C_M_RD; //Read
	msg.len = read_count;
	msg.buf = read_buf;
	msg.scl_rate = I2C_SCL_RATE;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = read_count;
 	}
 	else {// some error
		printk(KERN_ERR "%s failed?-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}
	return ret;
}

static int Read_Ack(struct waltop_I2C *tp)
{
	int ret = -1;
	unsigned char tmp_buf[10];
	
	tmp_buf[0]=0;
	tmp_buf[1]=0;
	ret = I2C_read_func(tp, tmp_buf, 2);
	//printk(KERN_INFO "FW Ack Value=%x,%x\n", tmp_buf[0], tmp_buf[1]);
 	if( ret == 2) { // read count as we request
		ret = (tmp_buf[0] << 8 | tmp_buf[1]);
	}
    //printk(KERN_INFO "FW Ack Value=%d\n", ret);
	return ret;
}

static int I2C_write_func(struct waltop_I2C *tp, unsigned char *write_buf, int write_count)
{
	int ret = -1;
	int try_count = 0;
    struct i2c_msg msg;

	if( write_count <= 4 ) {
		printk(KERN_INFO "FW CMD=0x%02x, 0x%02x, 0x%02x, 0x%02x\n", 
           write_buf[0], write_buf[1], write_buf[2], write_buf[3]);
	}
	msg.addr = tp->client->addr;
    msg.flags = 0; //Write
	msg.len = write_count;
	msg.buf = write_buf;
	msg.scl_rate = I2C_SCL_RATE;
	while( ++try_count < 5){
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = write_count;
		break;
 	}
	else if(ret == -11)
	{
		printk("simon, retry count =%d\n", try_count);
		msleep(10);
	}
}
	return ret;
}

static int Write_Data(struct waltop_I2C *tp, unsigned char *tx_buf, int count)
{
	int i;
	unsigned char checkSum;
	unsigned char out_buffer[FW_IOBUFFER_SIZE];

	checkSum = 0;
	if( count>FW_IODATA_SIZE )
		count = FW_IODATA_SIZE;
    for (i=0; i<count; i++)
	{ 
		out_buffer[i] = tx_buf[i];
		checkSum += tx_buf[i];
		//printk("%x,",tx_buf[i]);
	}

	//printk("\n");
	out_buffer[i] = checkSum;
	out_buffer[i+1] = checkSum;
	return I2C_write_func(tp, out_buffer, count+2);
}

static ssize_t fwdata_read(struct file *filp, struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{

	struct device *dev = container_of(kobj, struct device, kobj);
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	unsigned char ACKDataOK[2] = {0x92, 0xE1};
	unsigned char ACKPageOK[2] = {0x92, 0xE2};
	unsigned char rx_buf[FW_BLOCK_SIZE+4];
	int i, ret=0, retCount=count;

	if(count>FW_BLOCK_SIZE) {
		printk(KERN_ERR "read size over buffer size!\n");
		return -1;
	}

	strcpy(iap_fw_status, "reading");
	m_loop_write_flag = 1;

	for(i=0; i<(count-FW_IODATA_SIZE); i=i+FW_IODATA_SIZE)
	{
		ret=I2C_read_func(tp, &rx_buf[i], FW_IODATA_SIZE);
		if(FW_IODATA_SIZE==ret) {
			wait_queue_flag = 0;
			ret=I2C_write_func(tp, ACKDataOK, 2);
			// wait for fw INT
			wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
		}
		else {
			retCount = 0x92E0;
			strcpy(iap_fw_status, "error");
			break;
		}
	}
	if(retCount<0x9200) // send read page OK and return data
	{
		//last read
		ret=I2C_read_func(tp, &rx_buf[count-FW_IODATA_SIZE], FW_IODATA_SIZE);
		if(FW_IODATA_SIZE==ret) {
			wait_queue_flag = 0;
			m_iapPageCount--;
			// fwupdate will send final ack code at last read, so this is only for every page
			if( m_iapPageCount>0 ) {
				ret=I2C_write_func(tp, ACKPageOK, 2);
				// wait for fw INT from f/w
				wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
			}
			memcpy(buf, rx_buf, count);
		}
		else {
			retCount = 0x92E0;
			strcpy(iap_fw_status, "error");
		}
	}
	wait_queue_flag = 0;
	m_loop_write_flag = 0;
	return retCount;
}

static ssize_t fwdata_write(struct file *filp, struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{
	
    //printk(KERN_INFO "request count size=%d\n", count);
    struct device *dev = container_of(kobj, struct device, kobj);
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	unsigned char tx_buf[FW_BLOCK_SIZE+4];
	int i, ret=0, retCount=count;
    
    if(count>FW_BLOCK_SIZE) {
		printk(KERN_ERR "write size over buffer size!\n");
		return -1;
	}
	memcpy(tx_buf, buf, count);

	m_fw_cmdAckValue = 0;
	wait_queue_flag = 0;
	m_loop_write_flag = 1;

	// 2012/12/18, Martin check
	if((count==4)&&(tx_buf[0]==0x84)) //make sure it is fw IAP command
	{
		//printk(KERN_ERR "start to write cmd 84!\n");
		m_iapIsReadCmd=0;
		ret=I2C_write_func(tp, tx_buf, count);
		// wait for fw ACK
		//printk(KERN_ERR "fwdata_write wait for fw ACK \n");
		wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
		wait_queue_flag = 0;
		if(strcmp(iap_fw_status, "error") == 0) {
			retCount = m_fw_cmdAckValue;	// return the error code
		}
		else {
			if(tx_buf[1]==0x02) {	// Start IAP Read
				m_iapIsReadCmd = 1;
				m_iapPageCount = tx_buf[2]*4;
				// wait for fw INT then back
				wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
				wait_queue_flag = 0;
			}
		}
	}else if(count == 2 && tx_buf[0] == 0x92){
	    dev_info(&tp->client->dev, "Write the finish message 0x%02X,0x%02X\n", tx_buf[0], tx_buf[1]);
	    ret=I2C_write_func(tp, tx_buf, count);
	}else // Write data
	{
		strcpy(iap_fw_status, "waiting");
		m_request_count = count;
		for(i=0; i<count; i=i+FW_IODATA_SIZE)
		{
			//m_request_count -= FW_IODATA_SIZE;
			ret=Write_Data(tp, &(tx_buf[i]), FW_IODATA_SIZE);
			// wait for fw ACK
			wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
			wait_queue_flag = 0;
            m_request_count -= FW_IODATA_SIZE;
			//if(strcmp(iap_fw_status, "error") == 0 || strcmp(iap_fw_status, "finish") == 0) {
            if(m_fw_cmdAckValue == 0x9200 || m_fw_cmdAckValue == 0x92E0 || m_fw_cmdAckValue == 0x92EF) { 
				retCount = m_fw_cmdAckValue;	// return the error code 0x9200, 0x92E0, 0x92EF
				break;
			}
		}
	}
	m_loop_write_flag = 0;
    //printk(KERN_INFO "fwdata_write return size=%d\n", retCount); 
	return retCount;
}

static struct bin_attribute waltop_I2C_fwdata_attributes = {
	.attr = {
		.name = "fwdata",
		.mode = S_IRUGO|S_IWUGO, //change this to super user only when release
	},
	.size = 0,	// 0 means no limit, but not over 4KB
	.read = fwdata_read,
	.write = fwdata_write,
};
#endif


/*************************************************************************
 * SYSFS related functions
 ************************************************************************/
static ssize_t waltop_show_irq_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", atomic_read(&tp->irq_enabled));
}

static ssize_t waltop_irq_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
	if (atomic_cmpxchg(&tp->irq_enabled, 1, 0))
	{
		dev_dbg(dev, "%s() - PEN IRQ %u has been DISABLED.\n", __func__, tp->client->irq);
		disable_irq(tp->client->irq);
	}
	else
	{
		atomic_set(&tp->irq_enabled, 1);
		dev_dbg(dev, "%s() - PEN IRQ %u has been ENABLED.\n", __func__, tp->client->irq);
		enable_irq(tp->client->irq);
	}
               	
    return size;
}
static DEVICE_ATTR(irq_enable,		S_IRUGO|S_IWUGO,	waltop_show_irq_status,		waltop_irq_enable);

static ssize_t waltop_show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
    mutex_lock(&tp->mutex);
	disable_irq(tp->client->irq);
#ifdef PEN_GPIO_FW_UPDATE
	gpio_set_value(PEN_GPIO_FW_UPDATE, 0);
#endif
	// Reset Pen, LOW for 10 ms, then HIGH
#ifdef PEN_GPIO_RESET
	gpio_direction_output(PEN_GPIO_RESET, 0);
	mdelay(FW_RESET_DELAY_A2B);
	gpio_direction_output(PEN_GPIO_RESET, 1);
	mdelay(FW_RESET_DELAY_B2C);
#endif
	// enable irq again
	if (tp->client->irq != 0)
	{
		irq_set_irq_type(tp->client->irq, IRQF_TRIGGER_FALLING);
		enable_irq(tp->client->irq);
	}
	mutex_unlock(&tp->mutex);

    return snprintf(buf, PAGE_SIZE, "Reset finished.\n");
}

static ssize_t waltop_show_read_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
    mutex_lock(&tp->mutex);
	waltop_I2C_read(tp);
	mutex_unlock(&tp->mutex);
	
	return snprintf(buf, PAGE_SIZE, "%x, %x, %x, %x,   %x, %x, %x, %x,   %x, %x, %x\n",
		tp->pkt_data[0], tp->pkt_data[1], tp->pkt_data[2], tp->pkt_data[3],
		tp->pkt_data[4], tp->pkt_data[5], tp->pkt_data[6], tp->pkt_data[7],
		tp->pkt_data[8], tp->pkt_data[9], tp->pkt_data[10]);
}

static ssize_t waltop_store_write_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
	__u8 tx_buf[4] = {0};
	int buf_tmp[4] = {0};
	int ret = 0;
	
    mutex_lock(&tp->mutex);
    sscanf(buf, "%x%x%x", &buf_tmp[0], &buf_tmp[1], &buf_tmp[2]);
    //printk(KERN_INFO "%x, %x, %x\n", buf_tmp[0], buf_tmp[1], buf_tmp[2]);
	tx_buf[0] = (__u8) buf_tmp[0];
	tx_buf[1] = (__u8) buf_tmp[1];
	tx_buf[2] = (__u8) buf_tmp[2];

	ret = i2c_master_send(tp->client, tx_buf, 3);
	if( ret<0 ) { // negative is error
		printk(KERN_ERR "i2c_write failed?-%s:%d\n", __FILE__, __LINE__);
		ret = -EINVAL;
	}
	mutex_unlock(&tp->mutex);
	
	return size;
}


static ssize_t waltop_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
		struct waltop_I2C *tp = dev_get_drvdata(dev);

		__u8 tx_buf[4] = {0};
		int buf_tmp[4] = {0};
		int ret = 0;

		sscanf(buf, "%d", &tp->delaytime);
		printk("Set the delay time %d\n", tp->delaytime);
		return size;
	}


static ssize_t waltop_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", tp->fw_version);
}


#ifdef IAP_FWUPDATE
static ssize_t waltop_show_enter_IAP(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
    int ret;
    mutex_lock(&tp->mutex);
    

	m_iap_fw_updating = 1;
#ifdef USE_WAKELOCK
	wake_lock(&iap_wake_lock);
#endif
	strcpy(iap_fw_status, "enterIAP");
	// Enter IAP
	// MUX -Add change your CPU'mux to GPIO here if needed
    // GPIO6 = Low, SDA = HIGH, SCL = LOW
#ifdef PEN_GPIO_FW_UPDATE
	gpio_set_value(PEN_GPIO_FW_UPDATE, 0);
#endif
	 /* Disable IRQ, set IRQ pin to Low */
	disable_irq(tp->client->irq);
	gpio_direction_output(PEN_GPIO_IRQ, 0);
	udelay(5);

	/* Reset Pen, LOW for 30 ms, then HIGH */
	gpio_direction_output(PEN_GPIO_RESET, 1);
	mdelay(FW_RESET_DELAY_A2B);
	gpio_direction_output(PEN_GPIO_RESET, 0);
	mdelay(FW_RESET_DELAY_B2C);

    /* Set IRQ pin to High, re-enable IRQ */
	/* set SCL=HIGH */
	gpio_direction_input(PEN_GPIO_IRQ);
	udelay(5);
	if (tp->client->irq != 0)
		enable_irq(tp->client->irq);

	/* wait for ready */
	mdelay(FW_RESET_DELAY_CP);

	mutex_unlock(&tp->mutex);
    return snprintf(buf, PAGE_SIZE, "Enter firmware update mode.\n");
}

static ssize_t waltop_show_exit_IAP(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);

    mutex_lock(&tp->mutex);
	// GPIO6 = Low, SDA = HIGH, SCL = HIGH
#ifdef PEN_GPIO_FW_UPDATE
	//gpio_set_value(PEN_GPIO_FW_UPDATE, 0);
#endif
	disable_irq(tp->client->irq);
	gpio_direction_input(PEN_GPIO_IRQ);
	udelay(5);
	
	/* Reset Pen, LOW for 30 ms, then HIGH */
	gpio_direction_output(PEN_GPIO_RESET, 1);
	mdelay(FW_RESET_DELAY_A2B);
	gpio_direction_output(PEN_GPIO_RESET, 0);
	mdelay(FW_RESET_DELAY_B2C);
   /* Set IRQ pin to High, re-enable IRQ */
	/* set SCL=HIGH */
	if (tp->client->irq != 0)
		enable_irq(tp->client->irq);

	/* wait for ready */
	mdelay(FW_RESET_DELAY_CP);

	m_iap_fw_updating = 0;
	m_iapIsReadCmd = 0;
	m_iapPageCount = 0;
#ifdef USE_WAKELOCK
	wake_unlock(&iap_wake_lock);
#endif
	strcpy(iap_fw_status, "exitIAP");

	mutex_unlock(&tp->mutex);

    return snprintf(buf, PAGE_SIZE, "Enter normal mode.\n");
}
#endif


// SYSFS : Device Attributes 
static DEVICE_ATTR(reset,			S_IRUGO,	waltop_show_reset,			NULL);
static DEVICE_ATTR(read_data,		S_IRUGO,	waltop_show_read_data,		NULL);
static DEVICE_ATTR(write_command,	S_IWUGO,	NULL,		waltop_store_write_cmd);
static DEVICE_ATTR(fwversion,		S_IRUGO,	waltop_show_fw_version,		NULL);
#ifdef IAP_FWUPDATE
static DEVICE_ATTR(fwupdate_entry,	S_IRUGO,	waltop_show_enter_IAP,		NULL);
static DEVICE_ATTR(fwupdate_exit,	S_IRUGO,	waltop_show_exit_IAP,		NULL);
#endif

static struct attribute *waltop_I2C_attributes[] = {
	&dev_attr_reset.attr,
	&dev_attr_read_data.attr,
	&dev_attr_write_command.attr,
    &dev_attr_fwversion.attr,
#ifdef IAP_FWUPDATE
	&dev_attr_fwupdate_entry.attr,
	&dev_attr_fwupdate_exit.attr,
#endif
	NULL
};

static struct attribute_group waltop_I2C_attribute_group = {
    .attrs = waltop_I2C_attributes
};


/*****************************************************************************
 * Interrupt and Workqueue related finctions
 ****************************************************************************/
void waltop_I2C_worker(struct work_struct *work)
{
	struct waltop_I2C *tp = container_of(work, struct waltop_I2C, work);
	struct input_dev *inp = tp->input;
	unsigned int x, y, ps, dv;
	int btn_up, btn_low, in_range, tip;
	//printk("%s: enter...\n", __func__);
#ifdef IAP_FWUPDATE
	int ret = 0;

	if(m_iap_fw_updating)
	{
		if(m_iapIsReadCmd==0)
		{
			ret = Read_Ack(tp);
	   
			m_fw_cmdAckValue = ret;
			if(ret == 0x9200 || ret == 0x92E0 || ret == 0x92EF)
			{
				if(ret == 0x92EF)
				{
					strcpy(iap_fw_status, "finish");
				}
				else
				{
					strcpy(iap_fw_status, "error");
				}
			#ifdef USE_WAKELOCK
				wake_unlock(&iap_wake_lock);
			#endif
			}
			else if(ret == -EAGAIN)
			{
				strcpy(iap_fw_status, "error");
			}
			else if(m_request_count == 0) {		
				strcpy(iap_fw_status, "continue");
			}
		}
		if(m_loop_write_flag) {
			wait_queue_flag = 1;
			wake_up_interruptible(&iap_wait_queue_head);
		}
		goto i2cReadErr_out;
	}
#endif

	/* do I2C read */
	if( waltop_I2C_read(tp) < 0 ){ // some error
		goto i2cReadErr_out;
	}
	
	// Log read packet data from firmware for debug
	//printk(KERN_INFO "%x, %x, %x, %x,   %x, %x, %x, %x,   %x, %x, %x\n",
	//tp->pkt_data[0], tp->pkt_data[1], tp->pkt_data[2], tp->pkt_data[3],
	//tp->pkt_data[4], tp->pkt_data[5], tp->pkt_data[6], tp->pkt_data[7],
	//tp->pkt_data[8], tp->pkt_data[9], tp->pkt_data[10]);
		
	// do input_sync() here
	// ...
	in_range = tp->pkt_data[6]&0x20;
	//printk(KERN_INFO "in_range=0x%x", in_range);

	/* report BTN_TOOL_PEN event depend on in range */
	input_report_key(inp, BTN_TOOL_PEN, in_range>0 ? 1:0);

	if( in_range )
	{
		x = ((tp->pkt_data[1] << 8) | tp->pkt_data[2]);
		y = ((tp->pkt_data[3] << 8) | tp->pkt_data[4]);
		ps = (((tp->pkt_data[6] & 0x03) << 8 ) | tp->pkt_data[5]);
		dv = (tp->pkt_data[6]&0x40);
	
		tip = (tp->pkt_data[6]&0x04);
		btn_low = (tp->pkt_data[6]&0x08);
		btn_up = (tp->pkt_data[6]&0x10);

		//printk(KERN_INFO "x=%d, y=%d, pressure=%d, in_range=0x%x, tip=0x%x, btn_low=0x%x, btn_up=%x\n",
		//x, y, ps, in_range, tip, btn_low, btn_up);

		// <<<< 2012/11 mirror direction if x or y is opposite >>>>
		x = tp->x_max - x;
		y = tp->y_max - y;
		//printk(KERN_INFO "x2=%d, y2=%d\n",x, y);

#ifdef REMAP_TO_LCD_SIZE
		// <<<< 2012/11 scale the resolution here if Android don't do it >>>>
		x = x * LCD_SCREEN_MAX_X / (tp->x_max);
		y = y * LCD_SCREEN_MAX_Y / (tp->y_max);
		// or
		//x = x * LCD_SCREEN_MAX_Y / (tp->x_max);
		//y = y * LCD_SCREEN_MAX_X / (tp->y_max);
#endif
		// Use standard single touch event
		// Report X, Y Value, <<<< 2012/11 swap x,y here if need >>>>
		input_report_abs(inp, ABS_X, x);
		input_report_abs(inp, ABS_Y, y);
		//input_report_abs(inp, ABS_X, y);
		//input_report_abs(inp, ABS_Y, x);

		// Report pressure and Tip as Down/Up
		if( dv && (ps > tp->p_minTipOn) ) {
			input_report_abs(inp, ABS_PRESSURE, ps);
			input_report_key(inp, BTN_TOUCH, 1);
		}
		else {
			input_report_abs(inp, ABS_PRESSURE, 0);
			input_report_key(inp, BTN_TOUCH, 0);
		}
		// Report side buttons on Pen
		input_report_key(inp, BTN_STYLUS, btn_low);
		input_report_key(inp, BTN_STYLUS2, btn_up);
	}
	input_sync(inp);
	
i2cReadErr_out:
	if (tp->client->irq != 0) {
		enable_irq(tp->client->irq);
	}
	return;
}

static irqreturn_t waltop_I2C_irq(int irq, void *handle)
{
	struct waltop_I2C *tp = (struct waltop_I2C *) handle;	
	dev_dbg(&(tp->client->dev), "%s : got irq!\n",__func__);

	//printk(KERN_INFO "waltop_I2C_irq in irq : %s\n", dev_name(&tp->client->dev));

	/* disable other irq until this irq is finished */
	disable_irq_nosync(tp->client->irq);
		
	/* schedule workqueue */
	queue_work(waltop_I2C_wq, &tp->work);

	return IRQ_HANDLED;
} 
 

/*****************************************************************************
 * Probe and Initialization functions
 ****************************************************************************/
static int waltop_I2C_init_sysfile(struct i2c_client *client, struct waltop_I2C *tp)
{
	int ret = 0;
	
	ret = sysfs_create_group(&client->dev.kobj, &waltop_I2C_attribute_group);
	if (ret) {
		dev_err(&(client->dev), "%s() - ERROR: sysfs_create_group() failed: %d\n", __func__, ret);
	}
	else {
		dev_err(&(client->dev), "%s() - sysfs_create_group() succeeded.\n", __func__);
	}

	ret = device_create_file(&tp->client->dev, &dev_attr_irq_enable);
	if (ret < 0) {
		dev_err(&(client->dev), "%s() - ERROR: File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto error_dev_create_file;
	}
#ifdef IAP_FWUPDATE
	ret = sysfs_create_bin_file(&client->dev.kobj, &waltop_I2C_fwdata_attributes);
	if (ret < 0) {
		dev_err(&(client->dev), "%s() - ERROR: Binary file attributes creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto error_sysfs_create_bin_file;
	}
#ifdef USE_WAKELOCK
	wake_lock_init(&iap_wake_lock, WAKE_LOCK_SUSPEND,"PenIAP_WakeLock");
#endif
	return 0;

error_sysfs_create_bin_file:
	device_remove_file(&tp->client->dev, &dev_attr_irq_enable);
#else
	return 0;
#endif

error_dev_create_file:
	sysfs_remove_group(&client->dev.kobj, &waltop_I2C_attribute_group);
	
	return ret;
}
static int waltop_I2C_initialize(struct i2c_client *client, struct waltop_I2C *tp, struct wtI2C_platform_data *pfdata)
{
	struct input_dev *input_device;
	int ret = 0;
	/* create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device)
	{
		ret = -ENOMEM;
		dev_err(&(client->dev), "%s() - ERROR: Could not allocate input device.\n", __func__);
		goto error_free_device;
	}

	tp->client = client;
	tp->input = input_device;

	// 2013/01/30, Martin add device information
	ret = waltop_I2C_readDeviceInfo(tp);
	if( ret>0 ) { 
		tp->x_max = ((tp->pkt_data[1] << 8) | tp->pkt_data[2]);
		tp->y_max = ((tp->pkt_data[3] << 8) | tp->pkt_data[4]);
		tp->p_max = ((tp->pkt_data[6]&0x80)>>7)|((tp->pkt_data[7]&0x80)>>6);
		tp->p_max = ((tp->p_max << 8) | (tp->pkt_data[5]));
		tp->fw_version = ((tp->pkt_data[6]&0x7F)*100) + (tp->pkt_data[7]&0x7F);
		if( tp->x_max <= 0 )
			tp->x_max = WALTOP_MAX_X;
		if( tp->y_max <= 0 )
			tp->y_max = WALTOP_MAX_Y;
		if( tp->p_max <= 0 )
			tp->p_max = WALTOP_MAX_P;
	}
	else {
		tp->x_max = pfdata->x_max ? pfdata->x_max : WALTOP_MAX_X;
		tp->y_max = pfdata->y_max ? pfdata->y_max : WALTOP_MAX_Y;
		tp->p_max = pfdata->p_max ? pfdata->p_max : WALTOP_MAX_P;
		tp->fw_version = 1;	// 100 means fw 1.00, this is 0.01
	}
    printk(KERN_INFO "waltop_I2C_readDeviceInfo() - firmware version : %d \n", tp->fw_version);
	tp->p_minTipOn = pfdata->p_minTipOn ? pfdata->p_minTipOn : 0;

	/* Prepare worker structure prior to set up the timer/ISR */
	INIT_WORK(&tp->work, waltop_I2C_worker);
	atomic_set(&tp->irq_enabled, 1);
	
	/* set input device information */
	snprintf(tp->phys, sizeof(tp->phys), "%s/input0", dev_name(&client->dev));

	input_device->phys = tp->phys;
	input_device->dev.parent = &client->dev;
	input_device->name = "Waltop";
	input_device->id.bustype = BUS_I2C;

	/* for example, read product name and version from f/w to fill into id information */
	input_device->id.vendor  = 0x172f;
	input_device->id.product = 0x0100;	// Module code xxxx 
	input_device->id.version = tp->fw_version;	// set to fw_version 

	/* set wakeup but disabled */
	/* for example, suspend after timeout, because we are disabled */
	//waltop_I2C_disable(tp);

	input_set_drvdata(input_device, tp);

	// Use standard single touch event
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_SYN, input_device->evbit);
	
	set_bit(ABS_X, input_device->absbit);
    set_bit(ABS_Y, input_device->absbit);
    set_bit(ABS_PRESSURE, input_device->absbit);
	
	set_bit(BTN_TOOL_PEN, input_device->keybit);
	set_bit(BTN_TOUCH, input_device->keybit);	
	set_bit(BTN_STYLUS, input_device->keybit);
	set_bit(BTN_STYLUS2, input_device->keybit);

	// Set ABS_X, ABS_Y as module's resolution
#ifdef REMAP_TO_LCD_SIZE
	// <<<< 2012/11 scaling the resolution here if Android don't do it >>>>
	input_set_abs_params(input_device, ABS_X, 0, LCD_SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, LCD_SCREEN_MAX_Y, 0, 0);
#else
	input_set_abs_params(input_device, ABS_X, 0, tp->x_max, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, tp->y_max, 0, 0);
	// or
#endif
	input_set_abs_params(input_device, ABS_PRESSURE, 0, tp->p_max, 0, 0);

	ret = input_register_device(input_device);
	if (0 != ret) {
		printk(KERN_ERR "input_register_device() - Error:%s:%d\n", __FILE__, __LINE__);
		goto error_free_device;
	}
		
	/* interrupt setup */
	if (tp->client->irq)
	{		
		gpio_to_irq(tp->client->irq);
		/* enable irq */
		
		unsigned int type = gpio_get_value(tp->client->irq) ? IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
		ret = request_irq(tp->client->irq, waltop_I2C_irq, type, input_device->name, tp);
		if (ret)
		{
			dev_err(&(client->dev), "%s() - ERROR: Could not request IRQ: %d\n", __func__, ret);
			goto error_free_irq;
		}
	}

	// Create SYSFS related file
	ret = waltop_I2C_init_sysfile(client, tp);
	if (ret < 0) {
		goto error_free_irq;
	}

	i2c_set_clientdata(client, tp);
	goto succeed;
    
error_free_irq:
	free_irq(tp->client->irq, tp);
error_free_device:
	if (input_device)
	{
		input_free_device(input_device);
		tp->input = NULL;
	}

	kfree(tp);
succeed:

	return ret;
}


static int __devinit waltop_I2C_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct waltop_I2C *tp;
	struct device *dev = &client->dev;
	struct wtI2C_platform_data *pfdata = dev->platform_data;
	int err;	   
	
	printk(KERN_INFO "waltop_I2C_probe-%s:%d\n", __FILE__, __LINE__);
	mdelay(5);
		
	// check platform data
	if (!pfdata) {
		printk(KERN_ERR "no platform data?-%s:%d\n", __FILE__, __LINE__);
		err = -EINVAL;
		goto err_out;
	}
	else {
		printk(KERN_INFO "platform data?-%s:%d\n", __FILE__, __LINE__);
		//printk(KERN_INFO "xmax=%d, ymax =%d\n", pfdata->x_max, pfdata->y_max);
	}
	if(pfdata->ioint)
	{
		pfdata->ioint();
	}
	// check functionality
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

    tp = kzalloc(sizeof(struct waltop_I2C), GFP_KERNEL);
    if (NULL == tp)
    {
        dev_err(&(client->dev), "%s() - ERROR: Could not allocate %d bytes of kernel memory for ft5x06 struct.\n", __func__, sizeof(struct waltop_I2C));
        err = -ENOMEM;
        goto error_devinit0;
    } 

	
	// Need to initialize the SYSFS mutex before creating the SYSFS entries in waltop_I2C_initialize().
    mutex_init(&tp->mutex);
    err = waltop_I2C_initialize(client, tp, pfdata);
    if (0 > err)
    {
        dev_err(&(client->dev), "%s() - ERROR: waltop_I2C could not be initialized.\n", __func__);
        goto error_mutex_destroy;
    }

    goto succeed;

error_mutex_destroy:
    mutex_destroy(&tp->mutex);        
error_devinit0:    	
err_out:
succeed:
	return err;	
}


static int __devexit waltop_I2C_remove(struct i2c_client *client)
{
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	
	dev_info(&(client->dev), "%s() - Driver is unregistering.\n", __func__);

	device_remove_file(&tp->client->dev, &dev_attr_irq_enable);
    if(tp->client->irq != 0)
    {
		free_irq(client->irq, tp);
	}
	input_unregister_device(tp->input);	

    mutex_lock(&tp->mutex);
    /* Remove the SYSFS entries */
#ifdef IAP_FWUPDATE
    sysfs_remove_bin_file(&client->dev.kobj, &waltop_I2C_fwdata_attributes);
#ifdef USE_WAKELOCK
	wake_unlock(&iap_wake_lock);
	wake_lock_destroy(&iap_wake_lock);
#endif
#endif
    sysfs_remove_group(&client->dev.kobj, &waltop_I2C_attribute_group);
    mutex_unlock(&tp->mutex);
    mutex_destroy(&tp->mutex);
    
	kfree(tp);
	
    dev_info(&(client->dev), "%s() - Driver unregistration is complete.\n", __func__);
    
	return 0;
}

static int __init waltop_I2C_init(void)
{
	int ret = 0;
    printk(KERN_INFO "%s() - Waltop I2C Pen Driver (Built %s @ %s)\n", __func__, __DATE__, __TIME__);

    waltop_I2C_wq = create_singlethread_workqueue("waltop_I2C_wq");
    if (NULL == waltop_I2C_wq)
    {
        printk(KERN_ERR "%s() - ERROR: Could not create the Work Queue due to insufficient memory.\n", __func__);
        ret = -ENOMEM;
    }
    else
    {
        ret = i2c_add_driver(&waltop_I2C_driver);        
        printk(KERN_ERR "waltop_I2C_init-%s:%d\n", __FILE__, __LINE__);
		mdelay(5);
    }

    return ret;
}

static void __exit waltop_I2C_exit(void)
{
	if (waltop_I2C_wq)
    {
        destroy_workqueue(waltop_I2C_wq);
    }

    return i2c_del_driver(&waltop_I2C_driver);
}

module_init(waltop_I2C_init);
module_exit(waltop_I2C_exit); 

MODULE_AUTHOR("Waltop");
MODULE_DESCRIPTION("Waltop I2C pen driver");
MODULE_LICENSE("GPL");
