#ifndef __GSGPU_DRV_H__
#define __GSGPU_DRV_H__

#include <linux/firmware.h>
#include <linux/platform_device.h>

#include "gsgpu_shared.h"

#define DRIVER_AUTHOR		"Loongson graphics driver team"

#define DRIVER_NAME		"loonggpu"
#define DRIVER_DESC		"Loongson LGxx GPU Driver"
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 14, 0))
#define DRIVER_DATE		"20241219"
#endif

long gsgpu_drm_ioctl(struct file *filp,
		      unsigned int cmd, unsigned long arg);

#endif /* __GSGPU_DRV_H__ */
