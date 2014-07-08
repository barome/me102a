#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/sensor-dev.h>
#include "AsusSMT_sensorstatus.h"



static int asus_sensor_procfile_read(char *buffer, char **buffer_location,
							off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;

	//if (sensor_status == 0)
		ret = sprintf(buffer, "1\n");
	//else
	//	ret = sprintf(buffer, "0\n");

	return ret;
}

int setup_asusproc_sensorstatus_entry( int type )
{   
	struct proc_dir_entry *res;
    	char *  nodename ;
		
	/*enum sensor_type {
	SENSOR_TYPE_NULL,
	SENSOR_TYPE_ACCEL,
	SENSOR_TYPE_COMPASS,	
	SENSOR_TYPE_GYROSCOPE,	
	SENSOR_TYPE_LIGHT,	
	SENSOR_TYPE_PROXIMITY,
	SENSOR_TYPE_TEMPERATURE,	
	SENSOR_TYPE_PRESSURE,
	SENSOR_NUM_TYPES
	};  //copy from sensor-dev.h file , just for view*/

	switch(type)
	{
		case SENSOR_TYPE_ACCEL:	
			nodename="asus_gsensor_status";
			break;		
		case SENSOR_TYPE_COMPASS:		
			nodename="asus_compass_status";
			break;		
		case SENSOR_TYPE_GYROSCOPE:
			nodename="asus_gyro_status";
			break;
		case SENSOR_TYPE_LIGHT:
			nodename="asus_lightsensor_status";
			break;
		case SENSOR_TYPE_PROXIMITY:
			nodename="asus_proximity_status";
			break;
		case SENSOR_TYPE_TEMPERATURE:				
			nodename="asus_temperature_status";
			break;
		case SENSOR_TYPE_PRESSURE:				
			nodename="asus_pressure_status";
			break;
		default:
			printk("%s:error !unknow sensor type=%d\n",__func__,type);
			return  -EINVAL;
			break;

	}

    res = create_proc_entry(nodename, S_IWUGO| S_IRUGO, NULL);
    if (!res)
        return -ENOMEM;
    res->read_proc = asus_sensor_procfile_read;

    return 0;
}

