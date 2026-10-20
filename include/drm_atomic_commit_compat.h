/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(7, 2, 0)
#define drm_atomic_commit drm_atomic_state
#endif
