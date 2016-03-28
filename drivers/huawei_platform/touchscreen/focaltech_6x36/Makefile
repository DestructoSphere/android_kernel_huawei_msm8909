#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.


#focaltech   
obj-$(CONFIG_FOCALTECH_FT6X06_UPDATA_FW)    += ft6x06_ex_fun.o 
extlibs :=$(PWD)/drivers/input/touchscreen/focaltech/focal_scap_test_lib.a
ldflags-$(CONFIG_FOCALTECH_FT6X06_UPDATA_FW) +=--whole-archive $(extlibs) --no-whole-archive
obj-$(CONFIG_FOCALTECH_FOCALTECH_USERDATA)	+= focaltech_ctl.o
obj-$(CONFIG_FOCALTECH_FT6X06_SYSDEBUG)  	+= ft6x06_sysdebug.o
obj-$(CONFIG_FOCALTECH_FT6X06_TS)    		+= ft6x06_ts.o 

#delete scap test file, use lib to test

