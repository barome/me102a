#
#  Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
#
# Only use ANDROID_BUILD_SHELL to wrap around bash.
# DO NOT use other shells such as zsh.
ifdef ANDROID_BUILD_SHELL
SHELL := $(ANDROID_BUILD_SHELL)
else
# Use bash, not whatever shell somebody has installed as /bin/sh
# This is repeated in config.mk, since envsetup.sh runs that file
# directly.
SHELL := /bin/bash
endif

PWD := $(shell pwd)/drivers/power/ug31xx

$(shell cp -f $(PWD)/uG31xx_API_Capacity.x $(PWD)/uG31xx_API_Capacity.o)

obj-$(CONFIG_UPI_BATTERY) += uG31xx_API.o
obj-$(CONFIG_UPI_BATTERY) += ug31xx_i2c.o
obj-$(CONFIG_UPI_BATTERY) += ug31xx_gauge.o
obj-$(CONFIG_UPI_BATTERY) += uG31xx_API_Measurement.o
obj-$(CONFIG_UPI_BATTERY) += uG31xx_API_Otp.o
obj-$(CONFIG_UPI_BATTERY) += uG31xx_API_Capacity.o
obj-$(CONFIG_UPI_BATTERY) += uG31xx_API_System.o
obj-$(CONFIG_UPI_BATTERY) += uG31xx_API_Backup.o


