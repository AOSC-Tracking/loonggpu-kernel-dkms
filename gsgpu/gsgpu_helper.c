#include "gsgpu.h"


lg_dma_resv_t *lg_gsgpu_gem_prime_res_obj(struct drm_gem_object *obj)
{
	struct gsgpu_bo *bo = gem_to_gsgpu_bo(obj);

	return to_dma_resv(bo);
}

struct dma_buf *lg_gsgpu_gem_prime_export(
#if defined(LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG)
					  struct drm_device *dev,
#endif
                                          struct drm_gem_object *gobj,
					  int flags)
{
	return gsgpu_gem_prime_export(gobj, flags);
}
