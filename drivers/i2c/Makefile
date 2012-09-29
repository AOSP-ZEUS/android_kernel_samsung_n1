#
# Makefile for the i2c core.
#
<<<<<<< HEAD
=======
GCOV_PROFILE := y
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

obj-$(CONFIG_I2C_BOARDINFO)	+= i2c-boardinfo.o
obj-$(CONFIG_I2C)		+= i2c-core.o
obj-$(CONFIG_I2C_SMBUS)		+= i2c-smbus.o
obj-$(CONFIG_I2C_CHARDEV)	+= i2c-dev.o
obj-$(CONFIG_I2C_MUX)		+= i2c-mux.o
<<<<<<< HEAD
=======
obj-$(CONFIG_I2C_SLAVE)		+= i2c-slave.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
obj-y				+= algos/ busses/ muxes/

ccflags-$(CONFIG_I2C_DEBUG_CORE) := -DDEBUG
CFLAGS_i2c-core.o := -Wno-deprecated-declarations
