#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#
#proximity-objs := cm3629.o
#NFC ++
obj-$(CONFIG_SENSORS_NFC_PN544) += pn544.o
obj-$(CONFIG_SENSORS_NFC_PN544) += pn544_htc.o
obj-$(CONFIG_SENSORS_NFC_PN548) += pn548.o
obj-$(CONFIG_SENSORS_NFC_PN548) += pn548_htc.o

#NFC --

