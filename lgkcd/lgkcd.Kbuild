LGKCD_SOURCES ?=

LGKCD_SOURCES += lgkcd/kcd_ipc.c
LGKCD_SOURCES += lgkcd/kcd_chardev.c
LGKCD_SOURCES += lgkcd/kcd_crat.c
LGKCD_SOURCES += lgkcd/kcd_debug.c
LGKCD_SOURCES += lgkcd/kcd_debugfs.c
LGKCD_SOURCES += lgkcd/kcd_device.c
LGKCD_SOURCES += lgkcd/kcd_device_queue_manager.c
LGKCD_SOURCES += lgkcd/kcd_device_queue_manager_lg2xx.c
LGKCD_SOURCES += lgkcd/kcd_events.c
LGKCD_SOURCES += lgkcd/kcd_flat_memory.c
LGKCD_SOURCES += lgkcd/kcd_interrupt.c
LGKCD_SOURCES += lgkcd/kcd_kernel_queue.c
LGKCD_SOURCES += lgkcd/kcd_module.c
LGKCD_SOURCES += lgkcd/kcd_mqd_manager.c
LGKCD_SOURCES += lgkcd/kcd_mqd_manager_lg2xx.c
LGKCD_SOURCES += lgkcd/kcd_packet_manager.c
LGKCD_SOURCES += lgkcd/kcd_packet_manager_lg2xx.c
LGKCD_SOURCES += lgkcd/kcd_pasid.c
LGKCD_SOURCES += lgkcd/kcd_process.c
LGKCD_SOURCES += lgkcd/kcd_process_queue_manager.c
LGKCD_SOURCES += lgkcd/kcd_queue.c
LGKCD_SOURCES += lgkcd/kcd_smi_events.c
LGKCD_SOURCES += lgkcd/kcd_topology.c
LGKCD_SOURCES += lgkcd/lg2xx_event_interrupt.c
LGKCD_SOURCES += lgkcd/kcd_doorbell.c

ifeq ($(CONFIG_VDD_LOONGSON_SVM),1)
LGKCD_SOURCES += lgkcd/kcd_svm.c
LGKCD_SOURCES += lgkcd/kcd_migrate.c
endif
