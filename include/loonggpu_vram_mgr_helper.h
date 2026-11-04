#ifndef __LOONGGPU_VRAM_MGR_HELPER_H__
#define __LOONGGPU_VRAM_MGR_HELPER_H__

int loonggpu_vram_mgr_new_buddy(struct ttm_resource_manager *man,
			     struct ttm_buffer_object *tbo,
			     const struct ttm_place *place,
			     struct ttm_resource **res);
void loonggpu_vram_mgr_del_buddy(struct ttm_resource_manager *man,
				struct ttm_resource *res);
void loonggpu_vram_mgr_debug_buddy(struct ttm_resource_manager *man,
				struct drm_printer *printer);
bool loonggpu_vram_mgr_intersects(struct ttm_resource_manager *man,
				struct ttm_resource *res,
				const struct ttm_place *place,
				size_t size);
bool loonggpu_vram_mgr_compatible(struct ttm_resource_manager *man,
				struct ttm_resource *res,
				const struct ttm_place *place,
				size_t size);

static inline struct loonggpu_vram_mgr *lg_man_to_vram_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct loonggpu_vram_mgr, manager);
}

static inline struct loonggpu_device *vram_mgr_to_loonggpu_device(struct loonggpu_vram_mgr *mgr)
{
	return container_of(mgr, struct loonggpu_device, mman.vram_mgr);
	return NULL;
}

static inline int lg_vram_mgr_init(struct loonggpu_device *adev)
{
#if defined(LG_TTM_BO_INIT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_init_mm(&adev->mman.bdev, TTM_PL_VRAM,
			adev->gmc.real_vram_size >> PAGE_SHIFT);
#else
	return loonggpu_vram_mgr_init_buddy(adev);
#endif
}

u64 loonggpu_vram_mgr_vis_size(struct loonggpu_device *adev,
			       struct gpu_buddy_block *block);
#endif
