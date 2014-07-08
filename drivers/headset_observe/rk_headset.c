/* arch/arm/mach-rockchip/rk28_headset.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
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
#include "rk_headset.h"
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <linux/slab.h>
#include <mach/iomux.h>

#include <linux/sched.h>   
#include <linux/kthread.h>

//terry_tao@asus.com++ audio debug mode
#ifdef  CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#endif
//terry_tao@asus.com-- audio debug mode

/* Debug */
#if 0
#define LOG_TAG "Jack_detect "
#define DBG(x...) printk(LOG_TAG x)
#else
#define DBG(x...) do { } while (0)
#endif

#define BIT_HEADSET             (1 << 0)
#define BIT_HEADSET_NO_MIC      (1 << 1)

#define HEADSET 0
#define HOOK 1

#define HEADSET_IN 1
#define HEADSET_OUT 0
#define HOOK_DOWN 1
#define HOOK_UP 0
#define enable 1
#define disable 0

#ifdef CONFIG_SND_RK_SOC_RK2928
extern void rk2928_codec_set_spk(bool on);
#endif
#ifdef CONFIG_SND_SOC_WM8994
extern int wm8994_set_status(void);
#endif

static int hook_irq_balance = 1;
static int jack_irq_balance = 0;
static int hs_event_state;
int g_bDebugMode = 0;
EXPORT_SYMBOL(g_bDebugMode);

#ifdef CONFIG_SND_RK29_SOC_RT5639
#define MAX_MIC_READ_TIMES 50 //5 seconds
static int read_times = 0;
extern int rt5639_set_micbias1_on(bool);
extern int rt5639_test_micbias1(void);
#endif


/* headset private data */
struct headset_priv {
	struct input_dev *input_dev;
	struct rk_headset_pdata *pdata;
	unsigned int headset_status:1;
	unsigned int hook_status:1;
	unsigned int isMic:1;
	unsigned int isHook_irq:1;
	int cur_headset_status; 
	
	unsigned int irq[2];
	unsigned int irq_type[2];
	struct delayed_work h_delayed_work[2];
	struct switch_dev sdev;
	struct mutex mutex_lock[2];	
	unsigned char *keycodes;
};
static struct headset_priv *headset_info;
static struct task_struct * headset_thread = NULL;
static int headset_thread_fun(void *data);

int Headset_isMic(void)
{
	return headset_info->isMic;
}
EXPORT_SYMBOL_GPL(Headset_isMic);

int Headset_status(void)
{
	if(headset_info->cur_headset_status == BIT_HEADSET_NO_MIC ||
		headset_info->cur_headset_status == BIT_HEADSET )
		return HEADSET_IN;
	else
		return HEADSET_OUT;
}
EXPORT_SYMBOL_GPL(Headset_status);

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

static irqreturn_t headset_interrupt(int irq, void *dev_id)
{
	DBG("---headset_interrupt---\n");	
	schedule_delayed_work(&headset_info->h_delayed_work[HEADSET], msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static irqreturn_t Hook_interrupt(int irq, void *dev_id)
{
	DBG("---Hook_interrupt---\n");	
//	disable_irq_nosync(headset_info->irq[HOOK]);
	schedule_delayed_work(&headset_info->h_delayed_work[HOOK], msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static void headsetobserve_work(struct work_struct *work)
{
	int level = 0;
	struct rk_headset_pdata *pdata = headset_info->pdata;
	static unsigned int old_status = 0;
	DBG("%s----%d\n",__FUNCTION__,__LINE__);
	mutex_lock(&headset_info->mutex_lock[HEADSET]);

#ifdef  CONFIG_PROC_FS
	if(g_bDebugMode == 1)
		goto out;
#endif
	level = read_gpio(pdata->Headset_gpio);
	DBG("headset gpio level = %d\n",level);
	if(level < 0)
		goto out;
		
	old_status = headset_info->headset_status;
	if(pdata->headset_in_type == HEADSET_IN_HIGH)
		headset_info->headset_status = level?HEADSET_IN:HEADSET_OUT;
	else
		headset_info->headset_status = level?HEADSET_OUT:HEADSET_IN;

	if(old_status == headset_info->headset_status)	{
		goto out;
	}
	
	DBG("(headset in is %s)headset status is %s\n",
		pdata->headset_in_type?"high level":"low level",
		headset_info->headset_status?"in":"out");
		
	if(headset_info->headset_status == HEADSET_IN)
	{
		headset_info->cur_headset_status = BIT_HEADSET_NO_MIC;
		if(pdata->headset_in_type == HEADSET_IN_HIGH)
			irq_set_irq_type(headset_info->irq[HEADSET],IRQF_TRIGGER_FALLING);
		else
			irq_set_irq_type(headset_info->irq[HEADSET],IRQF_TRIGGER_RISING);
		if (pdata->Hook_gpio) {
			#ifdef CONFIG_SND_RK29_SOC_RT5639
			rt5639_set_micbias1_on(true);
			#endif
			//check hook status
			msleep(50);
			level = read_gpio(pdata->Hook_gpio);
			DBG("Hook gpio level = %d\n",level);
			if(level < 0)
				goto out;

			if((level > 0 && pdata->Hook_down_type == HOOK_DOWN_LOW)
				|| (level == 0 && pdata->Hook_down_type == HOOK_DOWN_HIGH))
			{
				headset_info->isMic = 1;//have mic
			}
			else{
				headset_info->isMic= 0;//No microphone
				headset_thread = kthread_run(headset_thread_fun,NULL,"headset_check_mic");
			}	
			DBG("headset_info->isMic = %d\n",headset_info->isMic);	
			headset_info->cur_headset_status = headset_info->isMic ? BIT_HEADSET:BIT_HEADSET_NO_MIC;
			//enable_irq(headset_info->irq[HOOK]);
			//headset_info->isHook_irq = enable;			
		}
	}
	else if(headset_info->headset_status == HEADSET_OUT)
	{	
		headset_info->hook_status = HOOK_UP;
		if(headset_info->isHook_irq == enable)
		{
			DBG("disable headset_hook irq\n");
			headset_info->isHook_irq = disable;
			disable_irq(headset_info->irq[HOOK]);
		}
		//rt5639_set_micbias1_on(false);
		headset_info->cur_headset_status = 0;//~(BIT_HEADSET|BIT_HEADSET_NO_MIC);					
		if(pdata->headset_in_type == HEADSET_IN_HIGH)
			irq_set_irq_type(headset_info->irq[HEADSET],IRQF_TRIGGER_RISING);
		else
			irq_set_irq_type(headset_info->irq[HEADSET],IRQF_TRIGGER_FALLING);
	}
	rk28_send_wakeup_key();
	switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);	
	DBG("headset_info->cur_headset_status = %d\n",headset_info->cur_headset_status);

out:
	mutex_unlock(&headset_info->mutex_lock[HEADSET]);	
}

static void Hook_work(struct work_struct *work)
{
	int level = 0;
	struct rk_headset_pdata *pdata = headset_info->pdata;
	static unsigned int old_status = HOOK_UP;
	DBG("%s----%d\n",__FUNCTION__,__LINE__);

	mutex_lock(&headset_info->mutex_lock[HOOK]);
	
	if(headset_info->headset_status == HEADSET_OUT){
		DBG("Headset is out\n");
		goto RE_ERROR;
	}
	
	#ifdef CONFIG_SND_SOC_WM8994
	if(wm8994_set_status() != 0)	{
		DBG("wm8994 is not set on heatset channel or suspend\n");
		goto RE_ERROR;
	}
	#endif
	#ifdef CONFIG_SND_RK29_SOC_RT5639
	if(rt5639_test_micbias1() != 0)
	{
		DBG("rt5639 is not set on heatset channel or suspend\n");
		goto RE_ERROR;
	}
	#endif
	
	level = read_gpio(pdata->Hook_gpio);
	DBG("Hook_work -- level = %d\n",level);
	if(level < 0)
		goto RE_ERROR;
	
	old_status = headset_info->hook_status;
//	DBG("Hook_work -- level = %d\n",level);
	
	if(level == 0)
		headset_info->hook_status = pdata->Hook_down_type == HOOK_DOWN_HIGH?HOOK_UP:HOOK_DOWN;
	else if(level > 0)	
		headset_info->hook_status = pdata->Hook_down_type == HOOK_DOWN_HIGH?HOOK_DOWN:HOOK_UP;
		
	if(old_status == headset_info->hook_status)
	{
		DBG("old_status == headset_info->hook_status\n");
		goto RE_ERROR;
	}
	DBG("Hook_work -- level = %d  hook status is %s\n",level,headset_info->hook_status?"key down":"key up");	
	if(headset_info->hook_status == HOOK_DOWN)
	{
		if(pdata->Hook_down_type == HOOK_DOWN_HIGH)
			irq_set_irq_type(headset_info->irq[HOOK],IRQF_TRIGGER_FALLING);
		else
			irq_set_irq_type(headset_info->irq[HOOK],IRQF_TRIGGER_RISING);		
	}
	else
	{
		if(pdata->Hook_down_type == HOOK_DOWN_HIGH)
			irq_set_irq_type(headset_info->irq[HOOK],IRQF_TRIGGER_RISING);
		else
			irq_set_irq_type(headset_info->irq[HOOK],IRQF_TRIGGER_FALLING);
	}
	input_report_key(headset_info->input_dev,pdata->hook_key_code,headset_info->hook_status);
	input_sync(headset_info->input_dev);	
RE_ERROR:
	mutex_unlock(&headset_info->mutex_lock[HOOK]);
}

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void headset_early_resume(struct early_suspend *h)
{
	schedule_delayed_work(&headset_info->h_delayed_work[HEADSET], msecs_to_jiffies(10));
	//DBG(">>>>>headset_early_resume\n");
}

static struct early_suspend hs_early_suspend;
#endif

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

//terry_tao@asus.com++ Audio debug mode
#ifdef  CONFIG_PROC_FS
#define Audio_debug_PROC_FILE  "driver/audio_debug"
static struct proc_dir_entry *audio_debug_proc_file;

static mm_segment_t oldfs;
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}

static ssize_t audio_debug_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];
    struct rk_headset_pdata *pdata = headset_info->pdata;
    memset(messages, 0, sizeof(messages));

    printk("[Audio Debug] audio_debug_proc_write\n");
    if (len > 256)
    {
        len = 256;
    }
    if (copy_from_user(messages, buff, len))
    {
        return -EFAULT;
    }
    
    initKernelEnv();

    if(strncmp(messages, "1", 1) == 0)
    {
        disable_irq(headset_info->irq[HEADSET]);
        jack_irq_balance +=1;
        
        //disable_irq(headset_info->irq[HOOK]);
        //hook_irq_balance += 1;
        headset_info->headset_status = HEADSET_OUT;
        switch_set_state(&headset_info->sdev, 0);
        
        //gpio_direction_output(PM8921_GPIO_PM_TO_SYS(19), 0);
        //gpio_direction_output(PM8921_GPIO_PM_TO_SYS(20), 0);//enable uart log, disable audio
        if(pdata->audio_debug_pin_type == AUDIO_DEBUG_HIGH)
			gpio_set_value(pdata->audio_debug_gpio,GPIO_HIGH);
		else
			gpio_set_value(pdata->audio_debug_gpio,GPIO_LOW);
		gpio_request(RK30_PIN1_PB1, NULL);
		iomux_set(UART2_SOUT);
        g_bDebugMode = 1;
        printk("Audio Debug Mode!!!\n");
    }
    else if(strncmp(messages, "0", 1) == 0)
    {
		while(jack_irq_balance){
			enable_irq(headset_info->irq[HEADSET]);
			jack_irq_balance -=1;
		}
		
		if(pdata->audio_debug_pin_type == AUDIO_DEBUG_HIGH)
			gpio_set_value(pdata->audio_debug_gpio,GPIO_LOW);
		else
			gpio_set_value(pdata->audio_debug_gpio,GPIO_HIGH);
		schedule_delayed_work(&headset_info->h_delayed_work[HEADSET], msecs_to_jiffies(50));

        printk("Audio Headset Normal Mode!!!\n");
		gpio_request(RK30_PIN1_PB1, NULL);
		iomux_set(GPIO1_B1);
		gpio_direction_output(RK30_PIN1_PB1, GPIO_LOW);
        g_bDebugMode = 0;
    }

    deinitKernelEnv(); 
    return len;
}

static struct file_operations audio_debug_proc_ops = {
    //.read = audio_debug_proc_read,
    .write = audio_debug_proc_write,
};

static void create_audio_debug_proc_file(void)
{
    
    printk("[Audio] create_audio_debug_proc_file\n");
    audio_debug_proc_file = create_proc_entry(Audio_debug_PROC_FILE, 0666, NULL);

    if (audio_debug_proc_file) {
        audio_debug_proc_file->proc_fops = &audio_debug_proc_ops;
    } 
}

static void remove_audio_debug_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk("[Audio] remove_audio_debug_proc_file\n");   
    remove_proc_entry(Audio_debug_PROC_FILE, &proc_root);
}
#endif //#ifdef CONFIG_PROC_FS
//terry_tao@asus.com-- Audio debug mode

static int headset_thread_fun(void *data)
{
	struct rk_headset_pdata *pdata;
	int level;
	int check_count = 0;
	while(!kthread_should_stop())
	{
		msleep(1000);

		if(headset_info != NULL && check_count < 5){
			pdata = headset_info->pdata;
			if(headset_info->cur_headset_status == BIT_HEADSET_NO_MIC){
				mutex_lock(&headset_info->mutex_lock[HEADSET]);
			//check mic again
				#ifdef CONFIG_SND_RK29_SOC_RT5639
				rt5639_set_micbias1_on(true);
				#endif
				msleep(50);
				//check hook status
				level = read_gpio(pdata->Hook_gpio);
				DBG("Hook gpio level = %d\n",level);
				if(level < 0)
					goto out;

				if((level > 0 && pdata->Hook_down_type == HOOK_DOWN_LOW)
					|| (level == 0 && pdata->Hook_down_type == HOOK_DOWN_HIGH))
				{
					headset_info->isMic = 1;//have mic
				}
				else
					headset_info->isMic= 0;//No microphone
				DBG("headset_thread_fun headset_info->isMic = %d\n",headset_info->isMic);	
				headset_info->cur_headset_status = headset_info->isMic ? BIT_HEADSET:BIT_HEADSET_NO_MIC;	
				
				check_count++;
				
				if(headset_info->isMic){
					switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);	
					check_count = 0;
				}
				
				if(check_count == 5 || headset_info->isMic)
				{
					DBG("check_count == 5 || headset_info->isMic\n");
					mutex_unlock(&headset_info->mutex_lock[HEADSET]);
					break;
				}
				DBG("headset_info->cur_headset_status = %d\n",headset_info->cur_headset_status);
				out:
				mutex_unlock(&headset_info->mutex_lock[HEADSET]);
			}
			else
			{
				check_count = 0;
				break;
			}
		}

	}
	DBG("thread stopped\n");
	return 0;
}

static int rockchip_headsetobserve_probe(struct platform_device *pdev)
{
	int ret;
	struct headset_priv *headset;
	struct rk_headset_pdata *pdata;
	DBG("%s----%d\n",__FUNCTION__,__LINE__);
	headset = kzalloc(sizeof(struct headset_priv), GFP_KERNEL);
	if (headset == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}	
	headset->pdata = pdev->dev.platform_data;
	pdata = headset->pdata;
	headset->headset_status = HEADSET_OUT;
	headset->hook_status = HOOK_UP;
	headset->isHook_irq = disable;
	headset->cur_headset_status = 0;
	headset->sdev.name = "h2w";
	headset->sdev.print_name = h2w_print_name;
	ret = switch_dev_register(&headset->sdev);
	if (ret < 0)
		goto failed_free;
	
	mutex_init(&headset->mutex_lock[HEADSET]);
	mutex_init(&headset->mutex_lock[HOOK]);
	
	INIT_DELAYED_WORK(&headset->h_delayed_work[HEADSET], headsetobserve_work);
	INIT_DELAYED_WORK(&headset->h_delayed_work[HOOK], Hook_work);

	headset->isMic = 0;
	//setup_timer(&headset->headset_timer, headset_timer_callback, (unsigned long)headset);
//------------------------------------------------------------------	
	// Create and register the input driver. 
	headset->input_dev = input_allocate_device();
	if (!headset->input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed_free;
	}	
	headset->input_dev->name = pdev->name;
	headset->input_dev->open = rk_Hskey_open;
	headset->input_dev->close = rk_Hskey_close;
	headset->input_dev->dev.parent = &pdev->dev;
	//input_dev->phys = KEY_PHYS_NAME;
	headset->input_dev->id.vendor = 0x0001;
	headset->input_dev->id.product = 0x0001;
	headset->input_dev->id.version = 0x0100;
	// Register the input device 
	ret = input_register_device(headset->input_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_dev;
	}
	input_set_capability(headset->input_dev, EV_KEY, pdata->hook_key_code);

#ifdef CONFIG_HAS_EARLYSUSPEND
	hs_early_suspend.suspend = NULL;
	hs_early_suspend.resume = headset_early_resume;
	hs_early_suspend.level = ~0x0;
	register_early_suspend(&hs_early_suspend);
#endif
	//------------------------------------------------------------------
	if (pdata->Headset_gpio) {
		ret = pdata->headset_io_init(pdata->Headset_gpio, pdata->headset_gpio_info.iomux_name, pdata->headset_gpio_info.iomux_mode);
		if (ret) 
			goto failed_free;	

		headset->irq[HEADSET] = gpio_to_irq(pdata->Headset_gpio);

		if(pdata->headset_in_type == HEADSET_IN_HIGH)
			headset->irq_type[HEADSET] = IRQF_TRIGGER_RISING;
		else
			headset->irq_type[HEADSET] = IRQF_TRIGGER_FALLING;
		ret = request_irq(headset->irq[HEADSET], headset_interrupt, headset->irq_type[HEADSET], "headset_input", NULL);
		if (ret) 
			goto failed_free_dev;
		enable_irq_wake(headset->irq[HEADSET]);
	}
	else
		goto failed_free_dev;
//------------------------------------------------------------------
	if (pdata->Hook_gpio) {
		ret = pdata->hook_io_init(pdata->Hook_gpio, pdata->hook_gpio_info.iomux_name, pdata->hook_gpio_info.iomux_mode);
		if (ret) 
			goto failed_free;
		headset->irq[HOOK] = gpio_to_irq(pdata->Hook_gpio);
		headset->irq_type[HOOK] = pdata->Hook_down_type == HOOK_DOWN_HIGH ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
			
		ret = request_irq(headset->irq[HOOK], Hook_interrupt, headset->irq_type[HOOK] , "headset_hook", NULL);
		if (ret) 
			goto failed_free_dev;
		disable_irq(headset->irq[HOOK]);
	}
//------------------------------------------------------------------	
	headset_info = headset;
	schedule_delayed_work(&headset->h_delayed_work[HEADSET], msecs_to_jiffies(500));
	
#ifdef  CONFIG_PROC_FS
	if (pdata->audio_debug_gpio) {
		ret = pdata->audio_debug_io_init(pdata->audio_debug_gpio, pdata->audio_debug_gpio_info.iomux_name, pdata->audio_debug_gpio_info.iomux_mode);
		if (ret) 
			goto failed_free;
		}
	//gpio_request(RK30_PIN3_PD7, NULL);
	//gpio_direction_output(RK30_PIN3_PD7, GPIO_HIGH);
    create_audio_debug_proc_file();
#endif
	
	return 0;	
	
failed_free_dev:
	platform_set_drvdata(pdev, NULL);
	input_free_device(headset->input_dev);
failed_free:
	dev_err(&pdev->dev, "failed to headset probe\n");
	kfree(headset);
	return ret;
}

static int rockchip_headsetobserve_suspend(struct platform_device *pdev, pm_message_t state)
{
	DBG("%s----%d\n",__FUNCTION__,__LINE__);
	disable_irq(headset_info->irq[HEADSET]);
	disable_irq(headset_info->irq[HOOK]);

	return 0;
}

static int rockchip_headsetobserve_resume(struct platform_device *pdev)
{
	DBG("%s----%d\n",__FUNCTION__,__LINE__);	
	enable_irq(headset_info->irq[HEADSET]);
	enable_irq(headset_info->irq[HOOK]);
	
	return 0;
}

static struct platform_driver rockchip_headsetobserve_driver = {
	.probe	= rockchip_headsetobserve_probe,
//	.resume = 	rockchip_headsetobserve_resume,	
//	.suspend = 	rockchip_headsetobserve_suspend,	
	.driver	= {
		.name	= "rk_headsetdet",
		.owner	= THIS_MODULE,
	},
};

static int __init rockchip_headsetobserve_init(void)
{
	platform_driver_register(&rockchip_headsetobserve_driver);
	return 0;
}
late_initcall(rockchip_headsetobserve_init);
MODULE_DESCRIPTION("Rockchip Headset Driver");
MODULE_LICENSE("GPL");
