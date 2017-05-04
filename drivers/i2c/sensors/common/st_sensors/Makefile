#
# Makefile for the STMicroelectronics sensor common modules.
#

obj-$(CONFIG_IIO_ST_SENSORS_I2C_HTC) += st_sensors_i2c.o
obj-$(CONFIG_IIO_ST_SENSORS_SPI_HTC) += st_sensors_spi.o
obj-$(CONFIG_IIO_ST_SENSORS_CORE_HTC) += st_sensors.o
st_sensors-y := st_sensors_core.o
st_sensors-$(CONFIG_IIO_BUFFER) += st_sensors_buffer.o
st_sensors-$(CONFIG_IIO_TRIGGER) += st_sensors_trigger.o
