#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/adc.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <mach/board.h> 
#include <linux/platform_device.h>
#include <linux/us5004.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//<ASUS-Bevis_chen +>
#ifdef CONFIG_ASUS_ENGINEER_MODE
#include <linux/proc_fs.h>

static int asus_thermalsensor_procfile_read(char *buffer, char **buffer_location,
							off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	//if (sensor_status == 0)
		ret = sprintf(buffer, "1\n");
	//else
	//	ret = sprintf(buffer, "0\n");
	return ret;
}

int create_asusproc_thermalsensor_status_entry( void )
{   
    struct proc_dir_entry *res;
    res = create_proc_entry("asus_thermalsensor_status", S_IWUGO| S_IRUGO, NULL);
    if (!res)
        return -ENOMEM;
    res->read_proc = asus_thermalsensor_procfile_read;
    return 0;
}

#endif
//<ASUS-Bevis_chen ->


#define DEBUG 	0

#if DEBUG
#define DBG(X...)	printk(KERN_NOTICE X)
#else
#define DBG(X...)
#endif

#define US5004_I2C_RATE 100*1000
//#define poll_delay_ms 5000
static int poll_delay_ms = 5000;
static int cur_temperature = 60;
static int alert = 0;

struct us5004_private_data {
	struct work_struct	work;
	struct delayed_work delaywork;	/*report second event*/
	struct i2c_client *client;
	struct tasklet_struct sirq_work;
	atomic_t data_ready;
	wait_queue_head_t data_ready_wq;		
	struct mutex data_mutex;
	struct mutex operation_mutex;	
	struct mutex sensor_mutex;
	struct mutex i2c_mutex;
	int status;
};

static struct i2c_client *us5004_client;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend us5004_early_suspend;
#endif

static int us5004_i2c_write(struct i2c_adapter *i2c_adap,
			    unsigned char address,
			    unsigned int len, unsigned char const *data)
{
	struct i2c_msg msgs[1];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)data;
	msgs[0].len = len;
	msgs[0].scl_rate = US5004_I2C_RATE;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res == 1)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}

static int us5004_i2c_read(struct i2c_adapter *i2c_adap,
			   unsigned char address, unsigned char reg,
			   unsigned int len, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = address;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;
	msgs[0].scl_rate = US5004_I2C_RATE;
	
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].scl_rate = US5004_I2C_RATE;	

	res = i2c_transfer(i2c_adap, msgs, 2);
	if (res == 2)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;

}


int us5004_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = us5004_i2c_read(client->adapter, client->addr, reg, length, rxData);	
	return ret;
}

int us5004_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	ret = us5004_i2c_write(client->adapter, client->addr, length, txData);
	return ret;

}

int us5004_write_reg(struct i2c_client *client, int addr, int value)
{
	char buffer[2];
	int ret = 0;
	struct us5004_private_data* sensor = 
		(struct us5004_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	buffer[0] = addr;
	buffer[1] = value;
	ret = us5004_tx_data(client, &buffer[0], 2);	
	mutex_unlock(&sensor->i2c_mutex);	
	return ret;
}

int us5004_read_reg(struct i2c_client *client, int addr)
{
	char tmp[1] = {0};
	int ret = 0;	
	struct us5004_private_data* sensor = 
		(struct us5004_private_data *)i2c_get_clientdata(client);
	
	mutex_lock(&sensor->i2c_mutex);	
	tmp[0] = addr;
	ret = us5004_rx_data(client, tmp, 1);
	mutex_unlock(&sensor->i2c_mutex);
	
	return tmp[0];
}

int report_cur_temperature(void)
{
	return cur_temperature;
}

EXPORT_SYMBOL_GPL(report_cur_temperature);


#ifdef CONFIG_HAS_EARLYSUSPEND
static void us5004_suspend(struct early_suspend *h)
{
	int result = 0;
	char data = 0;
	//printk("xxx go into us5004_suspend\n");
	result = us5004_write_reg(us5004_client,US5004_CONFIG_W, US5004_CONFIG_SUSPEND);
	if (result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
	}else
	{
		data=us5004_read_reg(us5004_client,US5004_CONFIG_R);
		if(data != US5004_CONFIG_SUSPEND)
		{
			printk("%s:line=%d,CONFIG=%d,error\n",__func__,__LINE__,data);
		}
		//printk("temperature Sensor us5004 enter suspend us5004->status\n");
	}
}

static void us5004_resume(struct early_suspend *h)
{
	int result = 0;
	char data = 0;
	//printk("xxx go into us5004_resume\n");
	result = us5004_write_reg(us5004_client,US5004_CONFIG_W, US5004_CONFIG_DEFAULT);
	if (result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
	}else
	{
		data=us5004_read_reg(us5004_client,US5004_CONFIG_R);
		if(data != US5004_CONFIG_DEFAULT)
		{
			printk("%s:line=%d,CONFIG=%d,error\n",__func__,__LINE__,data);
		}
		//printk("temperature Sensor us5004 enter resume us5004->status \n");
	}
}
#endif

static int sensor_initial(struct i2c_client *client)
{	
	int result = 0;
	char data=0;

	us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT | ENABLE_W_CONFIG);//enable write critical temperature
	result=us5004_write_reg(client,RT_CRITICAL_TEMPERATURE,RT_CRITICAL_TEMPERATURE_def);
	if(result != 0)
	{
		printk("can not write remote critical temperature\n");
	}
	us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT & (~ENABLE_W_CONFIG));

	result = us5004_write_reg(client, CONVERSION_RATE_w, CONVERSION_RATE_DATA);// set conversion rate
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	data=us5004_read_reg(client,CONVERSION_RATE_R);
	if(data != CONVERSION_RATE_DATA)
	{
		printk("%s:line=%d,rate=%d,error\n",__func__,__LINE__,data);
		return data;
	}
	result = us5004_write_reg(client, RTLSB_OFFSET_TEMPERATURE, TEMPERATURE_OFFSET);// set offest 
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	data=us5004_read_reg(client,RTLSB_OFFSET_TEMPERATURE);
	if(data != TEMPERATURE_OFFSET)
	{
		printk("%s:line=%d,rate=%d,error\n",__func__,__LINE__,data);
		return data;
	}
	result = us5004_write_reg(client,RTMSB_H_ALERT_W, RTMSB_H_ALERT_TEMPERATURE);// set remote high alert temperature 
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	data=us5004_read_reg(client,RTMSB_H_ALERT_R);
	if(data != RTMSB_H_ALERT_TEMPERATURE)
	{
		printk("%s:line=%d,remote high alert temperature=%d,error\n",__func__,__LINE__,data);
		return data;
	}	
	
	DBG("%s:ctrl_data=0x%x\n",__func__,sensor->ops->ctrl_data);	
	return result;
}
static int get_chip_id(struct i2c_client *client)
{	
	int result = 0;
	char temp = US5004_DID;
	int i = 0;

	for(i=0; i<3; i++)
	{
		result = us5004_rx_data(client, &temp, 1);
		if(!result)
		break;
	}

	if(result)
		return result;

	if(temp != US5004_DID_DATA )
	{
		printk("%s:id=0x%x is not 0x%x\n",__func__,temp, US5004_DID_DATA);
		result = -1;
	}
			
	DBG("%s:devid=0x%x\n",__func__,temp);
	
	return result;
}

static int temperature_chip_init(struct i2c_client *client)
{
	int result = 0;
	
	result = get_chip_id(client);//get id
	if(result < 0)
	{	
		printk("fail to read chip devid\n");	
		goto error;
	}

	result = sensor_initial(client);	//init sensor
	if(result < 0)
	{	
		printk("%s:fail to init sensor\n",__func__);		
		goto error;
	}

	return 0;

error:
	
	return result;
}

static void  us5004_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct us5004_private_data *sensor = container_of(delaywork, struct us5004_private_data, delaywork);
	struct i2c_client *client = sensor->client;
	u8 data=0;	
	
	mutex_lock(&sensor->sensor_mutex);

	//us5004_read_reg(client,US5004_IRQ_STATUS);

	cur_temperature = us5004_read_reg(client,RTMSB_TEMPERATURE);
	printk("cur_temperature = %d,alert = %d\n",cur_temperature,alert);
	if(cur_temperature > 0x55)
	{
		us5004_write_reg(client,CONVERSION_RATE_w,0x04);
		poll_delay_ms=1000;
	}else if(cur_temperature < 0x4b)
	{
		us5004_write_reg(client, CONVERSION_RATE_w, CONVERSION_RATE_DATA);
		poll_delay_ms=5000;
	}
	
	if((cur_temperature < RTMSB_H_ALERT_TEMPERATURE)&&alert)//
	{
		us5004_read_reg(client,US5004_IRQ_STATUS);
		us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT);
		data=us5004_read_reg(client,US5004_CONFIG_R);
		if(data == US5004_CONFIG_DEFAULT)
		{
			printk("restart temperature irq !\n");
		}else
		{
			printk("restart temperature irq failed !\n");
		}
		alert = 0;
	}else
	{
		//schedule_delayed_work(&sensor->delaywork, msecs_to_jiffies(poll_delay_ms));
	}
	mutex_unlock(&sensor->sensor_mutex);
	schedule_delayed_work(&sensor->delaywork, msecs_to_jiffies(poll_delay_ms));
	
	DBG("%s:%s\n",__func__,sensor->i2c_id->name);
}

static void thermal_tasklet_func (unsigned long data)
{
	struct us5004_private_data *sensor =
	    (struct us5004_private_data *) i2c_get_clientdata(us5004_client);
	mutex_lock(&sensor->sensor_mutex);
	cur_temperature = us5004_read_reg(us5004_client,RTMSB_TEMPERATURE);
	mutex_unlock(&sensor->sensor_mutex);
	printk("go into thermal_tasklet_func cur_temperature = %x\n",cur_temperature);	

	return;
}

static irqreturn_t us5004_interrupt(int irq, void *dev_id)
{
	printk("go into us5004_interrupt\n");
	struct us5004_private_data *sensor = (struct us5004_private_data *)dev_id;

	alert = 1;
	tasklet_schedule(&sensor->sirq_work);
	//mutex_lock(&sensor->sensor_mutex);
	//cur_temperature = us5004_read_reg(sensor->client,RTMSB_TEMPERATURE);
	//mutex_unlock(&sensor->sensor_mutex);
	//printk("go into us5004_interrupt cur_temperature = %x\n",cur_temperature);	

	return IRQ_HANDLED;
}

static int us5004_irq_init(struct i2c_client *client)
{
	struct us5004_private_data *sensor =
	    (struct us5004_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int irq;

		result = gpio_request(client->irq, "us5004");
		if (result)
		{
			printk("%s:fail to request gpio :%d\n",__func__,client->irq);
		}
		
		gpio_pull_updown(client->irq, PullEnable);
		irq = gpio_to_irq(client->irq);
		result = request_threaded_irq(irq, NULL, us5004_interrupt, IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "us5004", sensor);
		if (result) {
			printk(KERN_ERR "%s:fail to request irq = %d, ret = 0x%x\n",__func__, irq, result);	       
			goto error;	       
		}
		client->irq = irq;
		printk("%s:use irq=%d\n",__func__,irq);
		

error:	
	return result;
}

static ssize_t ltemperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,LOCAL_TEMPERATURE);
	
	return sprintf(buf, "%d\n",data);
}
static DEVICE_ATTR(ltemperature, S_IRUGO, ltemperature_show, NULL);//show local temperature

static ssize_t rtemperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,RTMSB_TEMPERATURE);
	
	return sprintf(buf, "%d\n",data);
}
static DEVICE_ATTR(rtemperature, S_IRUGO, rtemperature_show, NULL);//show remote temperature

static ssize_t l_high_alert_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,LT_H_ALERT_R);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t l_high_alert_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);
	//ret=sscanf(buf, "%u", &data);
	//if (ret != 1)
	//	return -EINVAL;
	
	ret=us5004_write_reg(client,LT_H_ALERT_W,data);
	
	return count;
}
static DEVICE_ATTR(l_high_alert_temperature, S_IRUGO | S_IWUSR, l_high_alert_temperature_show, l_high_alert_temperature_store);//show or store local high alert temperature

static ssize_t l_low_alert_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,LT_L_ALERT_R);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t l_low_alert_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);
	
	ret=us5004_write_reg(client,LT_L_ALERT_W,data);
	
	return count;
}
static DEVICE_ATTR(l_low_alert_temperature, S_IRUGO | S_IWUSR, l_low_alert_temperature_show, l_low_alert_temperature_store);//show or store local low alert temperature

static ssize_t r_high_alert_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,RTMSB_H_ALERT_R);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t r_high_alert_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);
	
	ret=us5004_write_reg(client,RTMSB_H_ALERT_W,data);
	
	return count;
}
static DEVICE_ATTR(r_high_alert_temperature, S_IRUGO | S_IWUSR, r_high_alert_temperature_show, r_high_alert_temperature_store);//show or store remote high alert temperature

static ssize_t r_low_alert_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,RTMSB_L_ALERT_R);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t r_low_alert_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);
	
	ret=us5004_write_reg(client,RTMSB_L_ALERT_W,data);
	
	return count;
}
static DEVICE_ATTR(r_low_alert_temperature, S_IRUGO | S_IWUSR, r_low_alert_temperature_show, r_low_alert_temperature_store);//show or store remote low alert temperature

static ssize_t l_critical_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,LT_CRITICAL_TEMPERATURE);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t l_critical_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);
	ret=us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT | ENABLE_W_CONFIG);//enable write critical temperature
	if(ret !=0 )
	{
		printk("can not enable to write critical temperature\n");
	}
	ret=us5004_write_reg(client,LT_CRITICAL_TEMPERATURE,data);
		if(ret != 0)
	{
		printk("can not write local critical temperature\n");
	}
	us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT & (~ENABLE_W_CONFIG));
	
	return count;
}
static DEVICE_ATTR(l_critical_temperature, S_IRUGO | S_IWUSR, l_critical_temperature_show, l_critical_temperature_store);//show or store local critical temperature

static ssize_t r_critical_temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	u8 data=0;
	data=us5004_read_reg(client,RT_CRITICAL_TEMPERATURE);
	
	return sprintf(buf, "%d\n",data);
}
static ssize_t r_critical_temperature_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev,struct i2c_client,dev);
	int data=0;
	int ret=0;
	data=simple_strtol(buf,NULL,10);	
	us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT | ENABLE_W_CONFIG);//enable write critical temperature
	ret=us5004_write_reg(client,RT_CRITICAL_TEMPERATURE,data);
	if(ret != 0)
	{
		printk("can not write remote critical temperature\n");
	}
	us5004_write_reg(client,US5004_CONFIG_W,US5004_CONFIG_DEFAULT & (~ENABLE_W_CONFIG));
	return count;
}
static DEVICE_ATTR(r_critical_temperature, S_IRUGO | S_IWUSR, r_critical_temperature_show, r_critical_temperature_store);//show or store remote critical temperature

static struct attribute *us5004_attributes[] = {
	&dev_attr_ltemperature.attr,
	&dev_attr_rtemperature.attr,
	&dev_attr_l_high_alert_temperature.attr,
	&dev_attr_l_low_alert_temperature.attr,
	&dev_attr_r_high_alert_temperature.attr,
	&dev_attr_r_low_alert_temperature.attr,
	&dev_attr_l_critical_temperature.attr,
	&dev_attr_r_critical_temperature.attr,
	NULL
};
static const struct attribute_group us5004_group = {
	.attrs = us5004_attributes,
};

static int us5004_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct us5004_private_data *sensor = (struct us5004_private_data *) i2c_get_clientdata(client);
	struct us5004_platform_data *pdata=NULL;
	int result = 0;
	int type = 0;
	dev_info(&client->adapter->dev, "%s: %s,0x%x\n", __func__, devid->name,(unsigned int)client);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->adapter->dev,
			"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_no_free;
	}
	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		result = -ENOMEM;
		goto out_no_free;
	}
	type= pdata->type;	
	if(type != 11)
	{	
		dev_err(&client->adapter->dev, "sensor type is error %d\n", pdata->type);
		result = -EFAULT;
		goto out_no_free;	
	}
	if((int)devid->driver_data != 55)
	{	
		dev_err(&client->adapter->dev, "sensor id is error %d\n", (int)devid->driver_data);
		result = -EFAULT;
		goto out_no_free;	
	}
	i2c_set_clientdata(client, sensor);
	us5004_client = client;
	sensor->client = client;	
	if (pdata->init_platform_hw) {
		result = pdata->init_platform_hw();
		if (result < 0)
			goto out_free_memory;
	}
	atomic_set(&(sensor->data_ready), 0);
	init_waitqueue_head(&(sensor->data_ready_wq));
	INIT_DELAYED_WORK(&sensor->delaywork, us5004_delaywork_func);
	tasklet_init(&sensor->sirq_work, thermal_tasklet_func, 0);
	mutex_init(&sensor->data_mutex);	
	mutex_init(&sensor->operation_mutex);	
	mutex_init(&sensor->sensor_mutex);
	mutex_init(&sensor->i2c_mutex);

	result = temperature_chip_init(sensor->client);
	if(result < 0)
		goto out_free_memory;

	result = us5004_irq_init(sensor->client);
	if (result) {
		dev_err(&client->dev,
			"fail to init sensor irq,ret=%d\n",result);
		goto out_unreg_irq;
	}
	result =sysfs_create_group(&client->dev.kobj, &us5004_group);
	if(result < 0) {  
        	printk(KERN_ALERT"Failed to create attribute us5004.");                  
        	goto out_unreg_irq;  
    	}

#ifdef CONFIG_HAS_EARLYSUSPEND
		us5004_early_suspend.suspend = us5004_suspend;
		us5004_early_suspend.resume = us5004_resume;
		us5004_early_suspend.level = 0x2;
		register_early_suspend(&us5004_early_suspend);
#endif

	schedule_delayed_work(&sensor->delaywork, msecs_to_jiffies(poll_delay_ms));
	printk("%s:initialized ok,sensor name:%s,type:%d,id=%d\n\n",__func__,pdata->name,type,(int)devid->driver_data);

	//<-- ASUS-Bevis_chen + -->
	#ifdef CONFIG_ASUS_ENGINEER_MODE
	if(create_asusproc_thermalsensor_status_entry( ))
		printk("%s: ERROR to create thermalsensor proc entry\n",__func__);
	#endif
	//<-- ASUS-Bevis_chen - -->

	return result;
	
out_unreg_irq:
	free_irq(client->irq, devid);	
out_free_memory:
	kfree(sensor);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return result;

}

static void us5004_shutdown(struct i2c_client *client)
{
	struct us5004_private_data *us5004 = (struct us5004_private_data *) i2c_get_clientdata(client);
	mutex_lock(&us5004->sensor_mutex);
	cancel_delayed_work_sync(&us5004->delaywork);
	tasklet_kill(&us5004->sirq_work);
	free_irq(client->irq, us5004);
	mutex_unlock(&us5004->sensor_mutex);
}


static __devexit int  us5004_remove(struct i2c_client *client)
{
	struct us5004_private_data *us5004 = (struct us5004_private_data *) i2c_get_clientdata(client);

	tasklet_kill(&us5004->sirq_work);
	cancel_delayed_work_sync(&us5004->delaywork);
	free_irq(client->irq, us5004);
	kfree(us5004);

#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&us5004_early_suspend);
		us5004_client = NULL;
#endif 
	return 0;
}

static const struct i2c_device_id us5004_i2c_id[] = {
	{ "us5004_temperature", 55 },
	{ }
};

static struct i2c_driver us5004_driver = {
	.probe = us5004_probe,
	.remove = __devexit_p(us5004_remove),
	.shutdown = us5004_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = "temperature-sensor",
	},
	.id_table = us5004_i2c_id,
};

static int __init us5004_init(void)
{
	int res = i2c_add_driver(&us5004_driver);
	pr_info("%s:Probe name %s\n",__func__,us5004_driver.driver.name);
	if(res)
		pr_err("%s failed \n",__func__);
	return res;
}

static void __exit us5004_exit(void)
{
	pr_info("%s\n",__func__);
	i2c_del_driver(&us5004_driver);
}

module_init(us5004_init);
module_exit(us5004_exit);
