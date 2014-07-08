/* /kernel/drivers/input/sensors/hall/hall_sensor.c
 *
 * ivan_shi@asus.com
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

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include "hall_sensor.h"
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <linux/slab.h>
#include <plat/key.h>

//<ASUS-Bevis_chen +>
#ifdef CONFIG_ASUS_ENGINEER_MODE
#include <linux/proc_fs.h>

static int asus_hallsensor_procfile_read(char *buffer, char **buffer_location,
							off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	//if (sensor_status == 0)
		ret = sprintf(buffer, "1\n");
	//else
	//	ret = sprintf(buffer, "0\n");
	return ret;
}

int create_asusproc_hallsensor_status_entry( void )
{   
    struct proc_dir_entry *res;
    res = create_proc_entry("asus_hallsensor_status", S_IWUGO| S_IRUGO, NULL);
    if (!res)
        return -ENOMEM;
    res->read_proc = asus_hallsensor_procfile_read;
    return 0;
}

#endif
//<ASUS-Bevis_chen ->
static struct wake_lock hall_sensor_suspend_lock;

#if 0
#define LOG_TAG "hall_sensor "
#define DBG(x...) printk(LOG_TAG x)
#else
#define DBG(x...) do { } while (0)
#endif

#define HALL_CLOSE 0
#define HALL_FAR 1

struct hall_sensor_priv {
	struct input_dev *input_dev;
	struct hall_sensor_pdata *pdata;
	unsigned int hall_sensor_status:1;	
	int cur_hall_sensor_status; 
	
	unsigned int irq;
	unsigned int irq_type;
	struct delayed_work h_delayed_work;
	struct switch_dev sdev;
	struct mutex mutex_lock;	
	struct timer_list hall_senosr_timer;
	unsigned char *keycodes;
};

static struct hall_sensor_priv *hall_sensor_info;

static int read_gpio(int gpio)
{
	int i,level;
	for(i=0; i<3; i++)
	{
		level = gpio_get_value(gpio);
		if(level < 0)
		{
			DBG("%s:get pin level again,pin=%d,i=%d\n",__FUNCTION__,gpio,i);
			msleep(1);
			continue;
		}
		else
		break;
	}
	if(level < 0)
		DBG("%s:get pin level  err!\n",__FUNCTION__);

	return level;
}

static irqreturn_t hall_sensor_interrupt(int irq, void *dev_id)
{
	DBG("---hall_sensor_interrupt---\n");	
	schedule_delayed_work(&hall_sensor_info->h_delayed_work, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static void hall_sensor_observe_work(struct work_struct *work)
{
	DBG("hall sensor hall_sensor_observe_work\n");
	
	int level = 0;
	struct hall_sensor_pdata *pdata = hall_sensor_info->pdata;
	static unsigned int old_status = 0;
	DBG("%s----%d\n",__FUNCTION__,__LINE__);
	mutex_lock(&hall_sensor_info->mutex_lock);
	wake_lock(&hall_sensor_suspend_lock);
	level = read_gpio(pdata->hall_sensor_gpio);
	DBG("hall sensor gpio level = %d\n",level);
	if(level < 0)
		goto out;
		
	old_status = hall_sensor_info->hall_sensor_status;
	//if(pdata->hall_sensor_in_type == HALL_SENSOR_IN_HIGH)
		hall_sensor_info->hall_sensor_status = level?HALL_FAR:HALL_CLOSE;
	//else
		//hall_sensor_info->hall_sensor_status = level?HEADSET_OUT:HEADSET_IN;

	if(old_status == hall_sensor_info->hall_sensor_status)	{
		goto out;
	}
	
	DBG("(hall sensor in is %s)hall sensor status is %s\n",
		pdata->hall_sensor_in_type?"high level":"low level",
		hall_sensor_info->hall_sensor_status?"far":"close");
		
	if(hall_sensor_info->hall_sensor_status == HALL_FAR)
	{
		DBG("hall sensor far\n");
		hall_sensor_info->cur_hall_sensor_status = HALL_FAR;
		irq_set_irq_type(hall_sensor_info->irq,IRQF_TRIGGER_FALLING);
	}
	else if(hall_sensor_info->hall_sensor_status == HALL_CLOSE)
	{
		DBG("hall sensor close\n");
		hall_sensor_info->cur_hall_sensor_status = HALL_CLOSE;
		irq_set_irq_type(hall_sensor_info->irq,IRQF_TRIGGER_RISING);
	}
	
	switch_set_state(&hall_sensor_info->sdev, hall_sensor_info->cur_hall_sensor_status);
	msleep(1000);
	DBG("hall_sensor_info->cur_hall_sensor_status = %d\n",hall_sensor_info->cur_hall_sensor_status);
		
out:
	wake_unlock(&hall_sensor_suspend_lock);
	mutex_unlock(&hall_sensor_info->mutex_lock);	
	
}
static void hall_senosr_timer_callback(unsigned long arg)
{
	DBG("hall sensor hall_senosr_timer_callback\n");
}

static int rk_Hskey_open(struct input_dev *dev)
{
	//struct rk28_adckey *adckey = input_get_drvdata(dev);
//	DBG("===========rk_Hskey_open===========\n");
	return 0;
}

static void rk_Hskey_close(struct input_dev *dev)
{
//	DBG("===========rk_Hskey_close===========\n");
//	struct rk28_adckey *adckey = input_get_drvdata(dev);

}



static int hall_sensor_probe(struct platform_device *pdev)
{
	DBG("hall sensor probe start\n");
	
	int ret;
	struct hall_sensor_priv *hall_sensor;
	struct hall_sensor_pdata *pdata;
	DBG("%s----%d\n",__FUNCTION__,__LINE__);
	hall_sensor = kzalloc(sizeof(struct hall_sensor_priv), GFP_KERNEL);
	if (hall_sensor == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}	
	hall_sensor->pdata = pdev->dev.platform_data;
	pdata = hall_sensor->pdata;
	//hall_sensor->hall_sensor_status = HEADSET_OUT;
	//hall_sensor->hook_status = HOOK_UP;
	//hall_sensor->isHook_irq = disable;
	hall_sensor->cur_hall_sensor_status = 1;
	hall_sensor->sdev.name = "hall";
	//hall_sensor->sdev.print_name = hall_print_name;
	ret = switch_dev_register(&hall_sensor->sdev);
	DBG("register sdev %d \n",ret);
	if (ret < 0)
		goto failed_free;
		
	mutex_init(&hall_sensor->mutex_lock);
	wake_lock_init(&hall_sensor_suspend_lock, WAKE_LOCK_SUSPEND, "hall_sensor_suspend_lock");
	INIT_DELAYED_WORK(&hall_sensor->h_delayed_work, hall_sensor_observe_work);	

	setup_timer(&hall_sensor->hall_senosr_timer, hall_senosr_timer_callback, (unsigned long)hall_sensor);
	
	
//------------------------------------------------------------------	
	// Create and register the input driver. 
	hall_sensor->input_dev = input_allocate_device();
	if (!hall_sensor->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed_free;
	}	
	hall_sensor->input_dev->name = pdev->name;
	hall_sensor->input_dev->open = rk_Hskey_open;
	hall_sensor->input_dev->close = rk_Hskey_close;
	hall_sensor->input_dev->dev.parent = &pdev->dev;
	//input_dev->phys = KEY_PHYS_NAME;
	hall_sensor->input_dev->id.vendor = 0x0001;
	hall_sensor->input_dev->id.product = 0x0001;
	hall_sensor->input_dev->id.version = 0x0100;
	// Register the input device 
	ret = input_register_device(hall_sensor->input_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_dev;
	}
	input_set_capability(hall_sensor->input_dev, EV_KEY, pdata->hook_key_code);
	
//------------------------------------------------------------------
	if (pdata->hall_sensor_gpio) {
		ret = pdata->hall_sensor_io_init(pdata->hall_sensor_gpio, pdata->hall_sensor_gpio_info.iomux_name, pdata->hall_sensor_gpio_info.iomux_mode);
		if (ret) 
			goto failed_free;	

		hall_sensor->irq = gpio_to_irq(pdata->hall_sensor_gpio);

		//if(pdata->hall_sensor_in_type == HALL_SENSOR_IN_HIGH)
			//hall_sensor->irq_type = IRQF_TRIGGER_RISING;
		//else
			hall_sensor->irq_type = IRQF_TRIGGER_FALLING;
		ret = request_irq(hall_sensor->irq, hall_sensor_interrupt, hall_sensor->irq_type, "hall_sensor", NULL);
		if (ret) 
			goto failed_free_dev;
		enable_irq_wake(hall_sensor->irq);
	}
	else
		goto failed_free_dev;

//------------------------------------------------------------------	
	hall_sensor_info = hall_sensor;
	schedule_delayed_work(&hall_sensor->h_delayed_work, msecs_to_jiffies(500));		

	//<-- ASUS-Bevis_chen + -->
	#ifdef CONFIG_ASUS_ENGINEER_MODE
	if(create_asusproc_hallsensor_status_entry( ))
		printk("%s: ERROR to create hall_sensor proc entry\n",__func__);
	#endif
	//<-- ASUS-Bevis_chen - -->

	return 0;	
	
failed_free_dev:
	platform_set_drvdata(pdev, NULL);
	input_free_device(hall_sensor->input_dev);

failed_free:
	dev_err(&pdev->dev, "failed to hall_sensor probe\n");
	kfree(hall_sensor);
	return ret;
}



static struct platform_driver hall_sensor_driver = {
	.probe	= hall_sensor_probe,
	.driver	= {
		.name	= "hall_sensor",
		.owner	= THIS_MODULE,
	},
};

static int __init hall_sensor_init(void)
{
	platform_driver_register(&hall_sensor_driver);
	return 0;
}

late_initcall(hall_sensor_init);
MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL");
