#
# Copyright 2017 Advanced Micro Devices, Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#

LGKCD_FILES	:= $(LGKCD_PATH)/kcd_module.o \
		$(LGKCD_PATH)/kcd_device.o \
		$(LGKCD_PATH)/kcd_chardev.o \
		$(LGKCD_PATH)/kcd_topology.o \
		$(LGKCD_PATH)/kcd_pasid.o \
		$(LGKCD_PATH)/kcd_flat_memory.o \
		$(LGKCD_PATH)/kcd_process.o \
		$(LGKCD_PATH)/kcd_queue.o \
		$(LGKCD_PATH)/kcd_mqd_manager.o \
		$(LGKCD_PATH)/kcd_mqd_manager_lg2xx.o \
		$(LGKCD_PATH)/kcd_kernel_queue.o \
		$(LGKCD_PATH)/kcd_packet_manager.o \
		$(LGKCD_PATH)/kcd_packet_manager_lg2xx.o \
		$(LGKCD_PATH)/kcd_process_queue_manager.o \
		$(LGKCD_PATH)/kcd_device_queue_manager.o \
		$(LGKCD_PATH)/kcd_device_queue_manager_lg2xx.o \
		$(LGKCD_PATH)/kcd_interrupt.o \
		$(LGKCD_PATH)/kcd_events.o \
		$(LGKCD_PATH)/lg2xx_event_interrupt.o \
		$(LGKCD_PATH)/kcd_smi_events.o \
		$(LGKCD_PATH)/kcd_crat.o \
		$(LGKCD_PATH)/kcd_debug.o \
		$(LGKCD_PATH)/kcd_ipc.o

ifneq ($(CONFIG_DEBUG_FS),)
LGKCD_FILES += $(LGKCD_PATH)/kcd_debugfs.o
endif

ifneq ($(CONFIG_VDD_LOONGSON_SVM),)
LGKCD_FILES += $(LGKCD_PATH)/kcd_svm.o \
		$(LGKCD_PATH)/kcd_migrate.o
endif
