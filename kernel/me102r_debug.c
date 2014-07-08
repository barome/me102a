#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/rtc.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/act8846.h>

static int me102r_debug_open(struct inode * inode, struct file * file)
{
    return 0;
}

static int me102r_debug_release(struct inode * inode, struct file * file)
{
    return 0;
}

static ssize_t me102r_debug_read(struct file *file, char __user *buf,
             size_t count, loff_t *ppos)
{
	return 0;
}

static void hw_reset_rtl8723()
{
	struct regulator *dcdc;
	dcdc = regulator_get(NULL, "act_ldo4"); 
	if(IS_ERR(dcdc))
		printk("can not acquire act_ldo4\n");
	regulator_set_mode(dcdc, REGULATOR_MODE_STANDBY);
	udelay(100);
		
	dcdc = regulator_get(NULL, "act_ldo5"); 
	if(IS_ERR(dcdc))
		printk("can not acquire act_ldo5\n");
	regulator_set_mode(dcdc, REGULATOR_MODE_STANDBY);
	msleep(500);
	
	dcdc = regulator_get(NULL, "act_ldo5"); 
	if(IS_ERR(dcdc))
		printk("can not acquire act_ldo5\n");
	regulator_set_mode(dcdc, REGULATOR_MODE_NORMAL);
	udelay(100);
	
	dcdc = regulator_get(NULL, "act_ldo4"); 
	if(IS_ERR(dcdc))
		printk("can not acquire act_ldo4\n");
	regulator_set_mode(dcdc, REGULATOR_MODE_NORMAL);
	udelay(100);
}

char msg[256];
static ssize_t me102r_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    if (count > 256)
        count = 256;
    if (copy_from_user(msg, buf, count)){
		printk("%s:copy_from_user failed\n", __func__);
        return -EFAULT;
	}
	
	printk("%s:msg=%s\n", __func__, msg);
	
    if(strncmp(msg, "hw_reset_rtl8723", 16) == 0)
    {
        printk("%s:hw_reset_rtl8723\n", __func__);
		hw_reset_rtl8723();
    }

	return count;
}

static const struct file_operations proc_me102r_debug_operations = {
    .read       = me102r_debug_read,
    .write      = me102r_debug_write,
    .open       = me102r_debug_open,
    .release    = me102r_debug_release,
};

static int __init proc_me102r_debug_init(void)
{
    proc_create("me102r_debug", S_IRWXUGO, NULL, &proc_me102r_debug_operations);
    return 0;
}
module_init(proc_me102r_debug_init);


