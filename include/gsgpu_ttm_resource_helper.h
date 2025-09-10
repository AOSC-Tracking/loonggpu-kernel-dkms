#ifndef __GSGPU_TTM_RESOURCE_HELPER_H__
#define __GSGPU_TTM_RESOURCE_HELPER_H__

#if !defined (LG_DRM_TTM_TTM_BO_H_PRESENT)
#include <drm/ttm/ttm_bo_driver.h>
#endif

#if defined(LG_TTM_RESOURCE_MANAGER)
typedef struct ttm_resource_manager lg_ttm_manager_t;
typedef struct ttm_resource lg_ttm_mem_t;
typedef struct ttm_resource_manager_func lg_ttm_mem_func_t;
#define LG_GSGPU_VRAM_GTT_MGR_COMPAT_MEMBERS struct gsgpu_vram_mgr vram_mgr; \
                                             struct gsgpu_gtt_mgr gtt_mgr;
#define lg_gsgpu_vram_mgr_man struct ttm_resource_manager manager;
#define lg_gsgpu_gtt_mgr_man struct ttm_resource_manager manager;
#else
typedef struct ttm_mem_type_manager lg_ttm_manager_t;
typedef struct ttm_mem_reg lg_ttm_mem_t;
typedef struct ttm_mem_type_manager_func lg_ttm_mem_func_t;
#define LG_GSGPU_VRAM_GTT_MGR_COMPAT_MEMBERS
#define lg_gsgpu_vram_mgr_man
#define lg_gsgpu_gtt_mgr_man
#endif

#endif /* __GSGPU_TTM_RESOURCE_HELPER_H__ */
