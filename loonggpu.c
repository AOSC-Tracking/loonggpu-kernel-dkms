// SPDX-License-Identifier: GPL-2.0+

#ifdef CONFIG_PAGE_SIZE_4KB

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/printk.h>

static int __init gsgpu_init(void)
{
	printk(KERN_WARNING
	       "gsgpu does not work with configuration with 4 KiB page\n");
	return -EINVAL;
}

static void __exit gsgpu_exit(void) {}

module_init(gsgpu_init);
module_exit(gsgpu_exit);
MODULE_AUTHOR("Xi Ruoyao <xry111@xry111.site>");
MODULE_DESCRIPTION("Dummy module to make dkms happy on 4KiB page kernel");
MODULE_LICENSE("GPL");
#endif
