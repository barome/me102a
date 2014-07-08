/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/reboot.h>	
#include <linux/notifier.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>

#include "ug31xx_gauge.h"
#include "uG31xx_API.h"

/* Functions Declaration */
static int ug31xx_battery_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);
static int ug31xx_power_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);
static int ug31xx_update_psp(enum power_supply_property psp,
		union power_supply_propval *val);
static int ug31xx_debug_update_psp(enum power_supply_property psp,
		union power_supply_propval *val);
/*charger read voltage by ward_du*/
_meas_u16_ bq2415x_battery_voltage;
_meas_u16_ bq2415x_update_battery_voltage()
{
	//printk("ward_du in the function:%s,the battery voltage is :%d\n",__FUNCTION__,bq2415x_battery_voltage);//ward_du
	return bq2415x_battery_voltage;
}
EXPORT_SYMBOL(bq2415x_update_battery_voltage);

/* Extern Function */
extern char FactoryGGBXFile[];
/* Global Variables */
static struct ug31xx_gauge *ug31 = NULL;
static char *pGGB;
static GG_DEVICE_INFO		gauge_dev_info;
static GG_CAPACITY		gauge_dev_capacity;
static GG_FETCH_DEBUG_DATA_TYPE gauge_dev_debug;
static struct workqueue_struct *ug31xx_gauge_wq = NULL;
static char *chg_status[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
unsigned cur_cable_status = NO_CABLE;
drv_status_t drv_status = DRV_NOT_READY;

static enum power_supply_property ug31xx_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_CHARGE_FULL,
        /// [AT-PM] : Properties for UPI ; 01/30/2013
	POWER_SUPPLY_PROP_UG31XX_DEBUG,
};

static enum power_supply_property ug31xx_pwr_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
	"ac",
	"usb",
};

static struct power_supply ug31xx_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= ug31xx_batt_props,
		.num_properties = ARRAY_SIZE(ug31xx_batt_props),
		.get_property	= ug31xx_battery_get_property,
	},
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = ug31xx_pwr_props,
		.num_properties = ARRAY_SIZE(ug31xx_pwr_props),
		.get_property	= ug31xx_power_get_property,
	},
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = ug31xx_pwr_props,
		.num_properties = ARRAY_SIZE(ug31xx_pwr_props),
		.get_property = ug31xx_power_get_property,
	},
};

static int ug31xx_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    int ret = 0;

    switch (psp) {
    case POWER_SUPPLY_PROP_HEALTH:
            val->intval = POWER_SUPPLY_HEALTH_GOOD;
            break;
    case POWER_SUPPLY_PROP_PRESENT:
            val->intval = 1;
            break;
    case POWER_SUPPLY_PROP_STATUS:
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
    case POWER_SUPPLY_PROP_CURRENT_NOW:
    case POWER_SUPPLY_PROP_CAPACITY:
    case POWER_SUPPLY_PROP_TEMP:
    case POWER_SUPPLY_PROP_CHARGE_NOW:
    case POWER_SUPPLY_PROP_CHARGE_FULL:
            if (ug31xx_update_psp(psp, val))
		return -EINVAL;
            break;
    /// [AT-PM] : Properties for UPI ; 01/30/2013
    case POWER_SUPPLY_PROP_UG31XX_DEBUG:
	    if (ug31xx_debug_update_psp(psp, val))
		return -EINVAL;
            break;
    default:
            return -EINVAL;
    }
    return ret;
}

static int ug31xx_power_get_property(struct power_supply *psy,
							enum power_supply_property psp,
							union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:			
		if(psy->type == POWER_SUPPLY_TYPE_MAINS &&
			cur_cable_status == AC_ADAPTER_CABLE)
			val->intval = 1;
		else if (psy->type == POWER_SUPPLY_TYPE_USB &&
			cur_cable_status == USB_PC_CABLE)
			val->intval = 1;
		else
			val->intval = 0;
			break;
	default:
			return -EINVAL;
	}
	return ret;
}

static int ug31xx_update_psp(enum power_supply_property psp,
		union power_supply_propval *val)
{

	if (drv_status != DRV_INIT_OK) {
		GAUGE_err("Gauge driver not init finish\n");
		return -EINVAL;
	}

	if (psp == POWER_SUPPLY_PROP_TEMP)
	{
		val->intval = ug31->batt_temp = gauge_dev_info.IT;
		GAUGE_notice("Temperature=%d\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_CAPACITY)
	{
		if( gauge_dev_info.voltage_mV > 3400) val->intval = ug31->batt_capacity = gauge_dev_capacity.RSOC;
		else 
		{
			printk("ward_du:the voltage is below 3.4v!\n");
			val->intval =0;//ward_du
		}
		GAUGE_notice("Capacity=%d %%\n", val->intval);
	}	
	if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW)
	{
		val->intval = ug31->batt_volt = gauge_dev_info.voltage_mV;
		GAUGE_notice("Voltage=%d mV\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_CURRENT_NOW)
	{
		val->intval = ug31->batt_current = gauge_dev_info.AveCurrent_mA;
		GAUGE_notice("Current=%d mA\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_STATUS)
	{
		if (cur_cable_status) {
			if (ug31->batt_capacity == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		GAUGE_notice("Status=%s\n", chg_status[val->intval]);
	}
	if (psp == POWER_SUPPLY_PROP_CHARGE_NOW)
	{
		val->intval = ug31->batt_charge_now = gauge_dev_capacity.NAC;
		GAUGE_notice("Charge_Now=%d mAh\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_CHARGE_FULL)
	{
		val->intval =ug31->batt_charge_full = gauge_dev_capacity.LMD;
		GAUGE_notice("Charge_Full=%d mAh\n", val->intval);
	}

	return 0;
}

static int ug31xx_debug_update_psp(enum power_supply_property psp,
                union power_supply_propval *val)
{
	int idx;
	int size;
	char *ptr;

        if (drv_status != DRV_INIT_OK) {
                GAUGE_err("Gauge driver not init finish\n");
                return -EINVAL;
        }

        /// [AT-PM] : Properties for UPI ; 01/30/2013
	if (psp == POWER_SUPPLY_PROP_UG31XX_DEBUG)
	{
		idx = 0;
		size = sizeof(gauge_dev_debug);
		if(size > UPI_DEBUG_STRING)
		{
			size = UPI_DEBUG_STRING;
			GAUGE_notice("UPI debug string exceeds maximum length.\n");
		}
		ptr = (char *)&gauge_dev_debug;
		memset(ug31->gauge_debug, 0, UPI_DEBUG_STRING);
		while(idx < size)
		{
			sprintf(ug31->gauge_debug, "%s %02x", ug31->gauge_debug, ptr[idx]);
			idx = idx + 1;
		}
		GAUGE_notice("Version=%d.%08x.%06x\n", 
				gauge_dev_debug.versionMain, 
				gauge_dev_debug.versionOtp, 
				gauge_dev_debug.versionSub);
		val->strval = ug31->gauge_debug;
	}

	return 0;
}

static int ug31xx_powersupply_init(struct i2c_client *client)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(ug31xx_supply); i++) {
		ret = power_supply_register(&client->dev, &ug31xx_supply[i]);
		if (ret) {
			GAUGE_err("Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&ug31xx_supply[i]);
			return ret;
		}
	}
	return 0;
}

static void batt_info_update_work_func(struct work_struct *work)
{
	int gg_status;
	int delay_time;
	struct ug31xx_gauge *ug31_dev;
	ug31_dev = container_of(work, struct ug31xx_gauge, batt_info_update_work.work);

	gg_status = 0;
	delay_time = 10;
	#ifdef	UG31XX_DYNAMIC_POLLING
		/// [AT-PM] : Update gauge information ; 02/04/2013
		GAUGE_notice("Update gauge info!!\n");

		mutex_lock(&ug31->info_update_lock);
		gg_status = upiGG_ReadDeviceInfo(pGGB, &gauge_dev_info);
		if(gg_status != UG_READ_DEVICE_INFO_SUCCESS)
		{
			if(gg_status == UG_MEAS_FAIL_BATTERY_REMOVED)
			{
				gauge_dev_capacity.NAC = 0;
				gauge_dev_capacity.LMD = 0;
				gauge_dev_capacity.RSOC = 0;
			}
			GAUGE_err("Read device info fail. gg_status = %d\n", gg_status);
			goto update_data;
		}
		upiGG_ReadCapacity(pGGB, &gauge_dev_capacity);
		upiGG_FetchDebugData(pGGB, &gauge_dev_debug);

		if(gauge_dev_info.vBat1Average_mV < 3250)
		{
			delay_time = 1;
		}
		else if(gauge_dev_info.vBat1Average_mV < 3300)
		{
			delay_time = 5;
		}
		else if(gauge_dev_info.vBat1Average_mV < 3350)
		{
			delay_time = 10;
		}
		else if(gauge_dev_info.vBat1Average_mV < 3400)
		{
			delay_time = 20;
		}
		else if(gauge_dev_info.vBat1Average_mV < 3450)
		{
			delay_time = 40;
		}
		else
		{
			delay_time = 60;
		}
		GAUGE_notice("Gauge info updated!!\n");

update_data:
		mutex_unlock(&ug31->info_update_lock);
	#endif	///< end of UG31XX_DYNAMIC_POLLING

	power_supply_changed(&ug31xx_supply[PWR_SUPPLY_BATTERY]);

	queue_delayed_work(ug31xx_gauge_wq, &ug31_dev->batt_info_update_work, delay_time*HZ);
}

static void ug31_gauge_info_work_func(struct work_struct *work)
{
	int gg_status = 0, retry = 3;
	struct ug31xx_gauge *ug31_dev;
	ug31_dev = container_of(work, struct ug31xx_gauge, ug31_gauge_info_work.work);	

	#ifdef	UG31XX_DYNAMIC_POLLING
		return;
	#endif	///< end of UG31XX_DYNAMIC_POLLING

	GAUGE_notice("Update gauge info!!\n");

	mutex_lock(&ug31->info_update_lock);
	while (retry-- > 0) {
		gg_status = upiGG_ReadDeviceInfo(pGGB,&gauge_dev_info);
		if (gg_status == UG_READ_DEVICE_INFO_SUCCESS)
			goto read_dev_info_ok;
	}
	GAUGE_err("Read device info fail. gg_status=%d\n", gg_status);
	if(gg_status == UG_MEAS_FAIL_BATTERY_REMOVED)
	{
		gauge_dev_capacity.NAC = 0;
		gauge_dev_capacity.LMD = 0;
		gauge_dev_capacity.RSOC = 0;
	}
	goto read_dev_info_fail;

read_dev_info_ok:
	bq2415x_battery_voltage=gauge_dev_info.voltage_mV;//ward_du
	upiGG_ReadCapacity(pGGB,&gauge_dev_capacity);
	upiGG_FetchDebugData(pGGB,&gauge_dev_debug);

	GAUGE_notice("Gauge info updated !!\n");

read_dev_info_fail:
	mutex_unlock(&ug31->info_update_lock);

	if(gauge_dev_capacity.Ready == UG_CAP_DATA_READY)
	{
		queue_delayed_work(ug31xx_gauge_wq, &ug31_dev->ug31_gauge_info_work, 5*HZ);
	}
	else
	{
		queue_delayed_work(ug31xx_gauge_wq, &ug31_dev->ug31_gauge_info_work, 1*HZ);
	}
}

int ug31xx_cable_callback(unsigned usb_cable_state)
{
	int old_cable_status;

	if(drv_status != DRV_INIT_OK) {
		GAUGE_err("Gauge driver not init finish\n");
		cur_cable_status = usb_cable_state;
		return -EPERM;
	}
	printk("========================================================\n");
	printk("%s  usb_cable_state = %x\n", __func__, usb_cable_state) ;
 	printk("========================================================\n");

	old_cable_status = cur_cable_status;
	cur_cable_status = usb_cable_state;

	if (old_cable_status != cur_cable_status) {
		wake_lock_timeout(&ug31->cable_wake_lock, 5*HZ);
	}

	if (cur_cable_status == NO_CABLE) {
		if (old_cable_status == AC_ADAPTER_CABLE) {
			power_supply_changed(&ug31xx_supply[PWR_SUPPLY_AC]);
		} else if (old_cable_status == USB_PC_CABLE) {
			power_supply_changed(&ug31xx_supply[PWR_SUPPLY_USB]);
		}
	} else if (cur_cable_status == USB_PC_CABLE) {
		power_supply_changed(&ug31xx_supply[PWR_SUPPLY_USB]);
	} else if (cur_cable_status == AC_ADAPTER_CABLE) {
		power_supply_changed(&ug31xx_supply[PWR_SUPPLY_AC]);
	}
	cancel_delayed_work(&ug31->batt_info_update_work);
	queue_delayed_work(ug31xx_gauge_wq, &ug31->batt_info_update_work, 2*HZ);
	printk("ward_du:capacity is:%d%\n",gauge_dev_capacity.RSOC);//ward_du
	printk("ward_du:voltage is:%d mv\n",gauge_dev_info.voltage_mV);//ward_du
	return 0;
}
EXPORT_SYMBOL(ug31xx_cable_callback);

static int __devinit ug31xx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int gg_status;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	ug31 = kzalloc(sizeof(*ug31), GFP_KERNEL);
	if (!ug31)
		return -ENOMEM;

	ug31->client = client;
	ug31->dev = &client->dev;

	i2c_set_clientdata(client, ug31);
	ug31xx_i2c_client_set(ug31->client);

	/* get GGB file */
	gg_status = upiGG_Initial(&pGGB,	(GGBX_FILE_HEADER *)FactoryGGBXFile);
	if (gg_status != UG_INIT_SUCCESS) {
		GAUGE_err("GGB file read and init fail\n");
		goto ggb_init_fail;
	}

	mutex_init(&ug31->info_update_lock);
	wake_lock_init(&ug31->cable_wake_lock, WAKE_LOCK_SUSPEND, "cable_state_changed");

	ug31xx_gauge_wq = create_singlethread_workqueue("ug31xx_gauge_work_queue");
	INIT_DELAYED_WORK(&ug31->batt_info_update_work, batt_info_update_work_func);
	INIT_DELAYED_WORK(&ug31->ug31_gauge_info_work, ug31_gauge_info_work_func);
	queue_delayed_work(ug31xx_gauge_wq, &ug31->ug31_gauge_info_work, 0*HZ);

	/* power supply registration */
	if (ug31xx_powersupply_init(client))
		goto pwr_supply_fail;

	drv_status = DRV_INIT_OK;
	if (cur_cable_status)
		ug31xx_cable_callback(cur_cable_status);

	queue_delayed_work(ug31xx_gauge_wq, &ug31->batt_info_update_work, 15*HZ);

	GAUGE_notice(" Driver %s registered done\n", client->name);
	return 0;

pwr_supply_fail:
	kfree(ug31);
ggb_init_fail:
	if(!pGGB) {
		upiGG_UnInitial(&pGGB);
	}
	return gg_status;
}

static int __devexit ug31xx_i2c_remove(struct i2c_client *client)
{
	struct ug31xx_gauge *ug31_dev;
	int i = 0, gg_status;
	for (i = 0; i < ARRAY_SIZE(ug31xx_supply); i++) {
		power_supply_unregister(&ug31xx_supply[i]);
	}

	gg_status = upiGG_UnInitial(&pGGB);
	GAUGE_notice("Driver remove. gg_status=0x%02x\n", gg_status);

	ug31_dev = i2c_get_clientdata(client);
	if (ug31_dev) {
		kfree(ug31_dev);
	}
	return 0;
}

static int ug31xx_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int gg_status;

	cancel_delayed_work_sync(&ug31->batt_info_update_work);
	cancel_delayed_work_sync(&ug31->ug31_gauge_info_work);
	flush_workqueue(ug31xx_gauge_wq);

	mutex_lock(&ug31->info_update_lock);
	gg_status = upiGG_PreSuspend(pGGB);
	if (gg_status != UG_READ_DEVICE_INFO_SUCCESS) {
		GAUGE_err("Fail in suspend. gg_status=0x%02x\n", gg_status);
	} else
  		GAUGE_notice("Driver suspend. gg_status=0x%02x\n", gg_status);
	mutex_unlock(&ug31->info_update_lock);

	return 0;
}

static int ug31xx_i2c_resume(struct i2c_client *client)
{
	int gg_status;
	
	mutex_lock(&ug31->info_update_lock);
	gg_status = upiGG_ReadDeviceInfo(pGGB,&gauge_dev_info);
	if (gg_status == UG_READ_DEVICE_INFO_FAIL) {
		GAUGE_notice("Driver resume read fail. gg_status=0x%02x\n", gg_status);
	}
	gg_status = upiGG_Wakeup(pGGB);
	if (gg_status != UG_READ_DEVICE_INFO_SUCCESS) {
		GAUGE_err("Fail in resume. gg_status=0x%02x\n", gg_status);
		if(gg_status == UG_MEAS_FAIL_BATTERY_REMOVED)
		{
			gauge_dev_capacity.NAC = 0;
			gauge_dev_capacity.LMD = 0;
			gauge_dev_capacity.RSOC = 0;
		}
	} else {
  		GAUGE_notice("Driver resume. gg_status=0x%02x\n", gg_status);
	}
	mutex_unlock(&ug31->info_update_lock);

	cancel_delayed_work(&ug31->ug31_gauge_info_work);
	queue_delayed_work(ug31xx_gauge_wq, &ug31->ug31_gauge_info_work, 0*HZ);
	cancel_delayed_work(&ug31->batt_info_update_work);
	queue_delayed_work(ug31xx_gauge_wq,&ug31->batt_info_update_work, 5*HZ);

	return 0;
}

void ug31xx_i2c_shutdown(struct i2c_client *client)
{
	int gg_status;
	
	cancel_delayed_work(&ug31->ug31_gauge_info_work);
	cancel_delayed_work(&ug31->batt_info_update_work);
	mutex_lock(&ug31->info_update_lock);
	gg_status = upiGG_PrePowerOff(pGGB);
	mutex_unlock(&ug31->info_update_lock);
	GAUGE_notice("Driver shutdown. gg_status=0x%02x\n", gg_status);
}
			
static const struct i2c_device_id ug31xx_i2c_id[] = {
	{ UG31XX_DEV_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ug31xx_i2c_id);

static struct i2c_driver ug31xx_i2c_driver = {
  .driver    = {
          .name  = UG31XX_DEV_NAME,
          .owner = THIS_MODULE,
  },
  .probe     = ug31xx_i2c_probe,
  .remove    = __devexit_p(ug31xx_i2c_remove),
  .suspend   = ug31xx_i2c_suspend,
  .resume    = ug31xx_i2c_resume,
  .shutdown  = ug31xx_i2c_shutdown,
  .id_table  = ug31xx_i2c_id,
};

static int __init ug31xx_i2c_init(void)
{
	return i2c_add_driver(&ug31xx_i2c_driver);
}
subsys_initcall(ug31xx_i2c_init);

static void __exit ug31xx_i2c_exit(void)
{
	i2c_del_driver(&ug31xx_i2c_driver);
}
module_exit(ug31xx_i2c_exit);

MODULE_DESCRIPTION("ug31xx gauge driver");
MODULE_LICENSE("GPL");


