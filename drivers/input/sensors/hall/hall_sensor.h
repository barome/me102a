#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#define HALL_SENSOR_IN_HIGH 0x00000001
#define HALL_SENSOR_IN_LOW  0x00000000

#define HOOK_DOWN_HIGH 0x00000001
#define HOOK_DOWN_LOW  0x00000000

struct hall_io_info{
	char	iomux_name[50];
	int		iomux_mode;	
};


struct hall_sensor_pdata{
	unsigned int Hook_gpio;//Detection Headset--Must be set
	unsigned int Hook_adc_chn; //adc channel
	unsigned int Hook_down_type; //Hook key down status   
	int	hook_key_code;
	unsigned int hall_sensor_gpio;//Detection Headset--Must be set
	unsigned int hall_sensor_in_type;//	Headphones into the state level--Must be set	
	struct hall_io_info hall_sensor_gpio_info;
	struct hall_io_info hook_gpio_info;
	int (*hall_sensor_io_init)(int, char *, int);
	int (*hook_io_init)(int, char *, int);
};

#endif
