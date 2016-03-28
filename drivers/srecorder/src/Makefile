#
# Makefile for SRecorder.
#

ccflags-y := -Idrivers/srecorder/include

obj-$(CONFIG_SRECORDER) += srecorder_main.o \
	srecorder_symbols.o \
	srecorder_reason_time.o \
	srecorder_ps_info.o \
	srecorder_back_trace.o \
	srecorder_slab_info.o \
	srecorder_sys_info.o \
	srecorder_dmesg.o \
	srecorder_interface.o \
	srecorder_dev.o \
	srecorder_crc32.o \
	srecorder_log.o \
	srecorder_notifier.o \
	srecorder_jprobe.o

ifdef CONFIG_ARM # this is for 32bits
obj-$(CONFIG_SRECORDER) += srecorder_c_trace.o \
	srecorder_stack.o
endif
