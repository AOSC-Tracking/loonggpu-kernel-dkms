#ifndef __LOONGGPU_SCHEDULER_HELPER_H__
#define __LOONGGPU_SCHEDULER_HELPER_H__

#include <drm/gpu_scheduler.h>

#if defined(LG_DRM_SCHED_PRIORITY_COUNT)
#define LG_DRM_SCHED_PRIORITY_MAX DRM_SCHED_PRIORITY_COUNT
#define LG_DRM_SCHED_PRIORITY_HIGH_HW DRM_SCHED_PRIORITY_HIGH
#define LG_DRM_SCHED_PRIORITY_HIGH_SW DRM_SCHED_PRIORITY_HIGH
#define LG_DRM_SCHED_PRIORITY_INVALID DRM_SCHED_PRIORITY_COUNT
#else
#define LG_DRM_SCHED_PRIORITY_MAX DRM_SCHED_PRIORITY_MAX
#define LG_DRM_SCHED_PRIORITY_HIGH_HW DRM_SCHED_PRIORITY_HIGH_HW
#define LG_DRM_SCHED_PRIORITY_HIGH_SW DRM_SCHED_PRIORITY_HIGH_SW
#define LG_DRM_SCHED_PRIORITY_INVALID DRM_SCHED_PRIORITY_INVALID
#endif

#if defined(LG_DRM_SCHED_PRIORITY_MIN)
#define LG_DRM_SCHED_PRIORITY_LOW DRM_SCHED_PRIORITY_MIN
#else
#define LG_DRM_SCHED_PRIORITY_LOW DRM_SCHED_PRIORITY_LOW
#endif

#if defined(LG_DRM_SCHED_PRIORITY_COUNT_HAS_UNSET)
#define LG_DRM_SCHED_PRIORITY_UNSET DRM_SCHED_PRIORITY_UNSET
#else
#define LG_DRM_SCHED_PRIORITY_UNSET LOONGGPU_CTX_PRIORITY_UNSET
#endif

static inline int lg_drm_sched_entity_init(struct drm_sched_entity *entity,
                                          enum drm_sched_priority priority,
                                          struct drm_gpu_scheduler **sched_list,
                                          unsigned int num_sched_list,
                                          struct drm_sched_rq **rq_list,
                                          unsigned int num_rq_list,
                                          atomic_t *guilty)
{
#if defined(LG_DRM_SCHED_ENTITY_INIT_HAS_PRIORITY_ARG)
       return drm_sched_entity_init(entity, priority, sched_list, num_sched_list, guilty);
#else
       return drm_sched_entity_init(entity, rq_list, num_rq_list, guilty);
#endif
}
#endif /* __LOONGGPU_SCHEDULER_HELPER_H__ */
