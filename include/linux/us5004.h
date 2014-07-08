#ifndef __US5004_H__
#define __US5004_H__

#include <linux/ioctl.h>

/* us5004 temperature registers */
#define LOCAL_TEMPERATURE		0x00
#define RTMSB_TEMPERATURE		0x01
#define US5004_IRQ_STATUS		0x02
#define US5004_CONFIG_R 		0x03
#define CONVERSION_RATE_R		0x04
#define LT_H_ALERT_R			0x05
#define LT_L_ALERT_R			0x06
#define RTMSB_H_ALERT_R			0x07
#define RTMSB_L_ALERT_R			0x08
#define US5004_CONFIG_W			0x09	/*bit7:ALERT_interrupt_EN bit6:auto-convert/standby mode bit6:ADDR_EN bit5:Tcirt2_EN 
						bit2:monitor_EN bit1:TC_WrEn bit0:ALERT fault queue mode*/
#define CONVERSION_RATE_w		0x0a
#define LT_H_ALERT_W			0x0b
#define LT_L_ALERT_W			0x0c
#define RTMSB_H_ALERT_W			0x0d
#define RTMSB_L_ALERT_W			0x0e
#define ONE_SHOT_CONVERSION		0x0f
#define RTLSB_TEMPERATURE		0x10
#define RTMSB_OFFSET_TEMPERATURE	0x11
#define RTLSB_OFFSET_TEMPERATURE	0x12
#define RTLSB_H_ALERT_W			0x13
#define RTLSB_L_ALERT_W			0x14
#define ALERT_MASK			0x16
#define RT_CRITICAL_TEMPERATURE		0x19
#define LT_CRITICAL_TEMPERATURE		0x20
#define CRITICAL_TEMPERATURE_HYSTERESIS	0x21
#define RTFAM				0xbf
#define SMBUS_ADDR			0xfc

#define US5004_DID			0xff
/*  */

#define US5004_DID_DATA 		0x17
#define CONVERSION_RATE_DATA		0x02
#define RTMSB_H_ALERT_TEMPERATURE	0x55
#define US5004_CONFIG_DEFAULT		0x05
#define ENABLE_W_CONFIG			0x02
#define US5004_CONFIG_SUSPEND		0xe1
#define TEMPERATURE_OFFSET		0xe0
#define RT_CRITICAL_TEMPERATURE_def	0x62//0x5a

/* conversion rate */

struct us5004_platform_data{
	char *name;
	int type;
	int (*init_platform_hw)(void);
};


#endif  /* __US5004_H__ */

