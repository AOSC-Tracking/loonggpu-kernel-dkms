// SPDX-License-Identifier: GPL-2.0+

#ifdef CONFIG_PAGE_SIZE_4KB

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/printk.h>

static int __init loonggpu_init(void)
{
	printk(KERN_WARNING
	       "loonggpu does not work with configuration with 4 KiB page\n");
	return -EINVAL;
}

static void __exit loonggpu_exit(void) {}

module_init(loonggpu_init);
module_exit(loonggpu_exit);
MODULE_AUTHOR("Xi Ruoyao <xry111@xry111.site>");
MODULE_DESCRIPTION("Dummy module to make dkms happy on 4KiB page kernel");
MODULE_LICENSE("GPL");
#endif
