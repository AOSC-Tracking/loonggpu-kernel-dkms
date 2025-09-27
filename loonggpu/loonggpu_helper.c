#include "loonggpu.h"


lg_dma_resv_t *lg_loonggpu_gem_prime_res_obj(struct drm_gem_object *obj)
{
	struct loonggpu_bo *bo = gem_to_loonggpu_bo(obj);

	return to_dma_resv(bo);
}

struct dma_buf *lg_loonggpu_gem_prime_export(
#if defined(LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG)
					  struct drm_device *dev,
#endif
                                          struct drm_gem_object *gobj,
					  int flags)
{
	return loonggpu_gem_prime_export(gobj, flags);
}
