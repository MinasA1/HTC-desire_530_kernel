#
# Makefile for industrial I/O accelerometer drivers
#

obj-$(CONFIG_IIO_ST_ACCEL_3AXIS_HTC) += st_accel.o
st_accel-y := st_accel_core.o
st_accel-$(CONFIG_IIO_BUFFER) += st_accel_buffer.o

obj-$(CONFIG_IIO_ST_ACCEL_I2C_3AXIS_HTC) += st_accel_i2c.o
obj-$(CONFIG_IIO_ST_ACCEL_SPI_3AXIS_HTC) += st_accel_spi.o
