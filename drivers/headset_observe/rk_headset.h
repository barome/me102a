#ifndef RK_HEADSET_H
#define RK_HEADSET_H

#define HEADSET_IN_HIGH 0x00000001
#define HEADSET_IN_LOW  0x00000000

#define HOOK_DOWN_HIGH 0x00000001
#define HOOK_DOWN_LOW  0x00000000

//terry_tao@asus.com++ Audio debug mode
#ifdef  CONFIG_PROC_FS
#define AUDIO_DEBUG_HIGH	0x00000001
#define AUDIO_DEBUG_LOW		0x00000000
#endif
//terry_tao@asus.com-- Audio debug mode

struct io_info{
	char	iomux_name[50];
	int		iomux_mode;	
};


struct rk_headset_pdata{
	unsigned int Hook_gpio;//Detection Headset--Must be set
	unsigned int Hook_adc_chn; //adc channel
	unsigned int Hook_down_type; //Hook key down status   
	int	hook_key_code;
	unsigned int Headset_gpio;//Detection Headset--Must be set
	unsigned int headset_in_type;//	Headphones into the state level--Must be set	
	struct io_info headset_gpio_info;
	struct io_info hook_gpio_info;
//terry_tao@asus.com++ Audio debug mode
#ifdef  CONFIG_PROC_FS
	unsigned int audio_debug_gpio;//audio debug pin--Must be set
	unsigned int audio_debug_pin_type;//audio debug pin type, low in debug mode or high in debug mode--Must be set
	struct io_info audio_debug_gpio_info;
	int (*audio_debug_io_init)(int, char *, int);
#endif
//terry_tao@asus.com-- Audio debug mode
	int (*headset_io_init)(int, char *, int);
	int (*hook_io_init)(int, char *, int);
};

#endif
