#if !defined(LOONGGPU_BO_PIN_HELPER_H)
#define LOONGGPU_BO_PIN_HELPER_H

static inline int lg_bo_pin_count(struct loonggpu_bo *bo)
{
#if defined(LG_DRM_TBO_PIN_COUNT)
	return bo->tbo.pin_count;
#else
	return bo->pin_count;
#endif
}

#if defined(LG_DRM_TBO_PIN_COUNT)
#define lg_bo_pin_count_ptr bo->tbo.pin_count
#else
#define lg_bo_pin_count_ptr bo->pin_count
#endif

static inline void lg_ttm_bo_pin(struct loonggpu_bo *bo)
{
#if defined(LG_DRM_TBO_PIN_COUNT)
	ttm_bo_pin(&bo->tbo);
#else
	bo->pin_count++;
#endif
}

static inline void lg_bo_pin(struct loonggpu_bo *bo)
{
#if defined(LG_DRM_TBO_PIN_COUNT)
	ttm_bo_pin(&bo->tbo);
#else
	bo->pin_count = 1;
#endif
}

static inline void lg_ttm_bo_unpin(struct loonggpu_bo *bo)
{
#if defined(LG_DRM_TBO_PIN_COUNT)
	ttm_bo_unpin(&bo->tbo);
#else
	bo->pin_count--;
#endif
}

#endif /* LOONGGPU_BO_PIN_HELPER_H */
