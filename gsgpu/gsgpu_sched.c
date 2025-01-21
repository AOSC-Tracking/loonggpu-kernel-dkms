#include <linux/fdtable.h>
#include <linux/pid.h>
#include <linux/file.h>
#include "gsgpu_drm.h"
#include "gsgpu.h"
#include "gsgpu_sched.h"
#include "gsgpu_scheduler_helper.h"
#include "gsgpu_vm.h"

int gsgpu_to_sched_priority(int gsgpu_priority,
			enum drm_sched_priority *prio)
{
	switch (gsgpu_priority) {
	case GSGPU_CTX_PRIORITY_VERY_HIGH:
		*prio = LG_DRM_SCHED_PRIORITY_HIGH_HW;
		break;
	case GSGPU_CTX_PRIORITY_HIGH:
		*prio = LG_DRM_SCHED_PRIORITY_HIGH_SW;
		break;
	case GSGPU_CTX_PRIORITY_NORMAL:
		*prio = DRM_SCHED_PRIORITY_NORMAL;
		break;
	case GSGPU_CTX_PRIORITY_LOW:
	case GSGPU_CTX_PRIORITY_VERY_LOW:
		*prio = LG_DRM_SCHED_PRIORITY_LOW;
		break;
	case GSGPU_CTX_PRIORITY_UNSET:
		*prio = LG_DRM_SCHED_PRIORITY_UNSET;
		break;
	default:
		WARN(1, "Invalid context priority %d\n", gsgpu_priority);
		*prio = LG_DRM_SCHED_PRIORITY_INVALID;
		return -EINVAL;
	}

	return 0;
}

static int gsgpu_sched_process_priority_override(struct gsgpu_device *adev,
						  int fd,
						  enum drm_sched_priority priority)
{
	struct file *filp = fget(fd);
	struct drm_file *file;
	struct gsgpu_fpriv *fpriv;
	struct gsgpu_ctx *ctx;
	uint32_t id;

	if (!filp)
		return -EINVAL;

	file = filp->private_data;
	fpriv = file->driver_priv;
	idr_for_each_entry(&fpriv->ctx_mgr.ctx_handles, ctx, id)
		gsgpu_ctx_priority_override(ctx, priority);

	fput(filp);

	return 0;
}

int gsgpu_sched_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *filp)
{
	union drm_gsgpu_sched *args = data;
	struct gsgpu_device *adev = dev->dev_private;
	enum drm_sched_priority priority;
	int r;

	r = gsgpu_to_sched_priority(args->in.priority, &priority);
	if (args->in.flags || r == -EINVAL)
		return -EINVAL;

	switch (args->in.op) {
	case GSGPU_SCHED_OP_PROCESS_PRIORITY_OVERRIDE:
		r = gsgpu_sched_process_priority_override(adev,
							   args->in.fd,
							   priority);
		break;
	default:
		DRM_ERROR("Invalid sched op specified: %d\n", args->in.op);
		r = -EINVAL;
		break;
	}

	return r;
}
