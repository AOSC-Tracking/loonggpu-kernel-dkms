#include "loonggpu.h"

#if defined(LG_TTM_BO_DEVICE_INIT_HAS_GLOB_ARG)
/*
 * Global memory.
 */

/**
 * loonggpu_ttm_mem_global_init - Initialize and acquire reference to
 * memory object
 *
 * @ref: Object for initialization.
 *
 * This is called by drm_global_item_ref() when an object is being
 * initialized.
 */
static int loonggpu_ttm_mem_global_init(struct drm_global_reference *ref)
{
	return ttm_mem_global_init(ref->object);
}

/**
 * loonggpu_ttm_mem_global_release - Drop reference to a memory object
 *
 * @ref: Object being removed
 *
 * This is called by drm_global_item_unref() when an object is being
 * released.
 */
static void loonggpu_ttm_mem_global_release(struct drm_global_reference *ref)
{
	ttm_mem_global_release(ref->object);
}

/**
 * loonggpu_ttm_global_init - Initialize global TTM memory reference structures.
 *
 * @adev: LOONGGPU device for which the global structures need to be registered.
 *
 * This is called as part of the LOONGGPU ttm init from loonggpu_ttm_init()
 * during bring up.
 */
int lg_loonggpu_ttm_global_init(struct loonggpu_device *adev)
{
	struct drm_global_reference *global_ref;
	int r;

	/* ensure reference is false in case init fails */
	adev->mman.mem_global_referenced = false;

	global_ref = &adev->mman.mem_global_ref;
	global_ref->global_type = DRM_GLOBAL_TTM_MEM;
	global_ref->size = sizeof(struct ttm_mem_global);
	global_ref->init = &loonggpu_ttm_mem_global_init;
	global_ref->release = &loonggpu_ttm_mem_global_release;
	r = drm_global_item_ref(global_ref);
	if (r) {
		DRM_ERROR("Failed setting up TTM memory accounting "
			  "subsystem.\n");
		goto error_mem;
	}

	adev->mman.bo_global_ref.mem_glob =
		adev->mman.mem_global_ref.object;
	global_ref = &adev->mman.bo_global_ref.ref;
	global_ref->global_type = DRM_GLOBAL_TTM_BO;
	global_ref->size = sizeof(lg_ttm_global_t);
	global_ref->init = &ttm_bo_global_init;
	global_ref->release = &ttm_bo_global_release;
	r = drm_global_item_ref(global_ref);
	if (r) {
		DRM_ERROR("Failed setting up TTM BO subsystem.\n");
		goto error_bo;
	}

	adev->mman.mem_global_referenced = true;

	return 0;

error_bo:
	drm_global_item_unref(&adev->mman.mem_global_ref);
error_mem:
	return r;
}

void lg_loonggpu_ttm_global_fini(struct loonggpu_device *adev)
{
	if (adev->mman.mem_global_referenced) {
		drm_global_item_unref(&adev->mman.bo_global_ref.ref);
		drm_global_item_unref(&adev->mman.mem_global_ref);
		adev->mman.mem_global_referenced = false;
	}
}
#else
int lg_loonggpu_ttm_global_init(struct loonggpu_device *adev)
{
	return 0;
}
void lg_loonggpu_ttm_global_fini(struct loonggpu_device *adev) {}
#endif

