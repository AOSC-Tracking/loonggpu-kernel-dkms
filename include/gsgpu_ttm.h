#ifndef __GSGPU_TTM_H__
#define __GSGPU_TTM_H__

#include <drm/gpu_scheduler.h>
#include "gsgpu_ttm_helper.h"

#include "gsgpu_dma_resv_helper.h"
#include "gsgpu_ttm_resource_helper.h"

#define GSGPU_GTT_MAX_TRANSFER_SIZE	512
#define GSGPU_GTT_NUM_TRANSFER_WINDOWS	2
uint64_t gsgpu_ttm_domain_start(struct gsgpu_device *adev, uint32_t type);

struct gsgpu_vram_mgr {
        lg_gsgpu_vram_mgr_man
	#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	struct drm_buddy b_mm;
	#endif
	struct mutex m_lock;
        struct drm_mm mm;
        spinlock_t lock;
        atomic64_t usage;
        atomic64_t vis_usage;
	struct list_head reservations_pending;
	struct list_head reserved_pages;
	u64 default_page_size;
};

struct gsgpu_gtt_mgr {
        lg_gsgpu_gtt_mgr_man
        struct drm_mm mm;
        spinlock_t lock;
        atomic64_t available;
};

struct gsgpu_mman {
#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
	struct ttm_device               bdev;
#else
	struct ttm_bo_device		bdev;
#endif
	bool				mem_global_referenced;
	bool				initialized;
	void __iomem			*aper_base_kaddr;

#if defined(CONFIG_DEBUG_FS)
	struct dentry			*debugfs_entries[8];
#endif

	/* buffer handling */
	const struct gsgpu_buffer_funcs	*buffer_funcs;
	struct gsgpu_ring			*buffer_funcs_ring;
	bool					buffer_funcs_enabled;

	struct mutex				gtt_window_lock;
	/* Scheduler entity for buffer moves */
	struct drm_sched_entity			entity;

	/* members for compatible */
	LG_GSGPU_MMAN_COMPAT_MEMBERS
	LG_GSGPU_VRAM_GTT_MGR_COMPAT_MEMBERS
};

struct gsgpu_copy_mem {
	struct ttm_buffer_object	*bo;
	lg_ttm_mem_t			*mem;
	unsigned long			offset;
};

extern const lg_ttm_mem_func_t gsgpu_gtt_mgr_func;
extern const lg_ttm_mem_func_t gsgpu_vram_mgr_func;

bool gsgpu_gtt_mgr_has_gart_addr(lg_ttm_mem_t *mem);
uint64_t gsgpu_gtt_mgr_usage(lg_ttm_manager_t *man);
int gsgpu_gtt_mgr_recover(lg_ttm_manager_t *man);

u64 gsgpu_vram_mgr_bo_visible_size(struct gsgpu_bo *bo);
uint64_t gsgpu_vram_mgr_usage(lg_ttm_manager_t *man);
uint64_t gsgpu_vram_mgr_vis_usage(lg_ttm_manager_t *man);

int gsgpu_ttm_init(struct gsgpu_device *adev);
void gsgpu_ttm_late_init(struct gsgpu_device *adev);
void gsgpu_ttm_fini(struct gsgpu_device *adev);
void gsgpu_ttm_set_buffer_funcs_status(struct gsgpu_device *adev,
					bool enable);

int gsgpu_copy_buffer(struct gsgpu_ring *ring, uint64_t src_offset,
		       uint64_t dst_offset, uint32_t byte_count,
		       lg_dma_resv_t *resv,
		       struct dma_fence **fence, bool direct_submit,
		       bool vm_needs_flush);
int gsgpu_ttm_copy_mem_to_mem(struct gsgpu_device *adev,
			       struct gsgpu_copy_mem *src,
			       struct gsgpu_copy_mem *dst,
			       uint64_t size,
			       lg_dma_resv_t *resv,
			       struct dma_fence **f);
int gsgpu_fill_buffer(struct gsgpu_bo *bo,
			uint32_t src_data,
			lg_dma_resv_t *resv,
			struct dma_fence **fence);

int gsgpu_mmap(struct file *filp, struct vm_area_struct *vma);
int gsgpu_ttm_alloc_gart(struct ttm_buffer_object *bo);
int gsgpu_ttm_gart_bind(struct gsgpu_device *adev,
			struct ttm_buffer_object *tbo,
			uint64_t flags);
int gsgpu_ttm_recover_gart(struct ttm_buffer_object *tbo);

int gsgpu_ttm_tt_get_user_pages(struct ttm_tt *ttm, struct page **pages);
void gsgpu_ttm_tt_set_user_pages(struct ttm_tt *ttm, struct page **pages);
void gsgpu_ttm_tt_mark_user_pages(struct ttm_tt *ttm);
int gsgpu_ttm_tt_set_userptr(struct ttm_tt *ttm, uint64_t addr,
				     uint32_t flags);
bool gsgpu_ttm_tt_has_userptr(struct ttm_tt *ttm);
struct mm_struct *gsgpu_ttm_tt_get_usermm(struct ttm_tt *ttm);
bool gsgpu_ttm_tt_affect_userptr(struct ttm_tt *ttm, unsigned long start,
				  unsigned long end);
bool gsgpu_ttm_tt_userptr_invalidated(struct ttm_tt *ttm,
				       int *last_invalidated);
bool gsgpu_ttm_tt_userptr_needs_pages(struct ttm_tt *ttm);
bool gsgpu_ttm_tt_is_readonly(struct ttm_tt *ttm);
uint64_t gsgpu_ttm_tt_pte_flags(struct gsgpu_device *adev, struct ttm_tt *ttm,
				 lg_ttm_mem_t *mem);
int gsgpu_vram_mgr_init(lg_ttm_manager_t *man, unsigned long p_size);
int gsgpu_gtt_mgr_init(lg_ttm_manager_t *man, unsigned long p_size);
int gsgpu_vram_mgr_fini(lg_ttm_manager_t *man);
int gsgpu_gtt_mgr_fini(lg_ttm_manager_t *man);
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
int gsgpu_fill_buffer_buddy(struct gsgpu_bo *bo,
			    uint32_t src_data,
			    struct dma_resv *resv,
			    struct dma_fence **f,
			    bool delayed);
int gsgpu_vram_mgr_init_buddy(struct gsgpu_device *adev);
void gsgpu_vram_mgr_fini_buddy(struct gsgpu_device *adev);
#endif
#endif /* __GSGPU_TTM_H__ */
