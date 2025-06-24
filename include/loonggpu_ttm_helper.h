#ifndef __LOONGGPU_TTM_HELPER_H__
#define __LOONGGPU_TTM_HELPER_H__
#include "loonggpu_ttm_resource_helper.h"
#include <drm/ttm/ttm_tt.h>
#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
#include <drm/ttm/ttm_device.h>
#endif
#include "conftest.h"

unsigned long loonggpu_ttm_io_mem_pfn_buddy(struct ttm_buffer_object *bo,
					unsigned long page_offset);

struct loonggpu_vram_mgr_resource {
	lg_ttm_mem_t base;
	struct list_head blocks;
	unsigned long flags;
};

struct loonggpu_res_cursor {
	uint64_t	start;
	uint64_t	size;
	uint64_t	remaining;
	void		*node;
	uint32_t	mem_type;
};

struct ttm_bo_global_ref;
struct drm_global_reference;
struct loonggpu_device;

#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
typedef struct ttm_device lg_ttm_device_t;
#else
typedef struct ttm_bo_device lg_ttm_device_t;
#endif

#if defined(LG_TTM_BACKEND_FUNC)
#define lg_define_ttm_backend_func static struct ttm_backend_func loonggpu_backend_func = { \
							.bind = &loonggpu_ttm_backend_bind, \
							.unbind = &loonggpu_ttm_backend_unbind, \
							.destroy = &loonggpu_ttm_backend_destroy, \
							};
#define lg_ttm_backend_func_arg struct ttm_tt *ttm
#define lg_ttm_backend_func_pass_param ttm
#define lg_ttm_backend_func_get_bdev ttm->bdev
#define lg_ttm_backend_unbind_ret int
#define lg_ttm_backend_unbind_return(ret) return ret
#define lg_ttm_set_backend_func gtt->ttm.ttm.func = &loonggpu_backend_func;
#define lg_loonggpu_ttm_backend_unbind
#else
#define lg_define_ttm_backend_func
#define lg_ttm_backend_func_arg lg_ttm_device_t *bdev, struct ttm_tt *ttm
#define lg_ttm_backend_func_pass_param bdev, ttm
#define lg_ttm_backend_func_get_bdev bdev
#define lg_ttm_backend_unbind_ret void
#define lg_ttm_backend_unbind_return(ret) return
#define lg_ttm_set_backend_func
#define lg_loonggpu_ttm_backend_unbind loonggpu_ttm_backend_unbind(bdev, ttm);
#endif

static inline void lg_ttm_tt_destroy_common(lg_ttm_device_t *bdev, struct ttm_tt *ttm)
{
#if defined(LG_TTM_TT_DESTROY_COMMON)
	ttm_tt_destroy_common(bdev, ttm);
#endif
}

static inline void lg_ttm_dma_tt_fini(
		#if defined(LG_TTM_DMA_TT)
			struct ttm_dma_tt *ttm
		#else
			struct ttm_tt *ttm
		#endif
			)
{
#if defined(LG_TTM_DMA_TT_FINI)
	ttm_dma_tt_fini(ttm);
#else
	ttm_tt_fini(ttm);
#endif
}

int loonggpu_ttm_backend_bind(lg_ttm_backend_func_arg, lg_ttm_mem_t *bo_mem);

#if defined(LG_TTM_BO_DEVICE_INIT_HAS_GLOB_ARG)
#define LG_LOONGGPU_MMAN_COMPAT_MEMBERS \
        struct ttm_bo_global_ref        bo_global_ref; \
        struct drm_global_reference     mem_global_ref;
#else
#define LG_LOONGGPU_MMAN_COMPAT_MEMBERS
#endif

/*
 * fix case 1/10/13
 */
#define DRM_FILE_PAGE_OFFSET (0x100000000ULL >> PAGE_SHIFT)

extern int lg_loonggpu_ttm_global_init(struct loonggpu_device *adev);
extern void lg_loonggpu_ttm_global_fini(struct loonggpu_device *adev);

static inline void lg_init_mem_type_system(lg_ttm_manager_t *man)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	/* System memory */
	man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
	man->available_caching = TTM_PL_MASK_CACHING;
	man->default_caching = TTM_PL_FLAG_CACHED;
#endif
}

static inline void lg_init_mem_type_gtt(lg_ttm_manager_t *man,
				const lg_ttm_mem_func_t *func,
				u64 gart_start)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	/* GTT memory */
	man->func = func;
	man->gpu_offset = gart_start;
	man->available_caching = TTM_PL_MASK_CACHING;
	man->default_caching = TTM_PL_FLAG_CACHED;
	man->flags = TTM_MEMTYPE_FLAG_MAPPABLE | TTM_MEMTYPE_FLAG_CMA;
#endif
}

static inline void lg_init_mem_type_vram(lg_ttm_manager_t *man,
					const lg_ttm_mem_func_t *func,
					u64 vram_start)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	man->func = func;
	man->gpu_offset = vram_start;
	man->flags = TTM_MEMTYPE_FLAG_FIXED | TTM_MEMTYPE_FLAG_MAPPABLE;
	man->available_caching = TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
	man->default_caching = TTM_PL_FLAG_WC;
#endif
}

static inline int lg_ttm_bo_pipeline_move(struct ttm_buffer_object *bo,
					  struct dma_fence *fence,
					  bool evict, lg_ttm_mem_t *new_mem)
{
	int r;

#if defined(LG_TTM_BO_PIPELINE_MOVE) && defined(LG_DRM_TTM_TTM_BO_DRIVER_H_PRESENT)
	r = ttm_bo_pipeline_move(bo, fence, evict, new_mem);
#else
	if (bo->type == ttm_bo_type_kernel)
		r = ttm_bo_move_accel_cleanup(bo, fence, true, false, new_mem);
	else
		r = ttm_bo_move_accel_cleanup(bo, fence, evict, true, new_mem);
#endif
	return r;
}

static inline void lg_ttm_bo_mem_put(struct ttm_buffer_object *bo, lg_ttm_mem_t *mem)
{
#if defined(LG_TTM_BO_MEM_PUT) && defined(LG_DRM_TTM_TTM_BO_DRIVER_H_PRESENT)
	ttm_bo_mem_put(bo, mem);
#elif defined(LG_TTM_RESOURCE_FREE_DOUBLE_PTR)
	ttm_resource_free(bo, &mem);
#else
	ttm_resource_free(bo, mem);
#endif
}

static inline void lg_set_bus_placement_base(lg_ttm_mem_t *mem, phys_addr_t base)
{
#if defined(LG_TTM_BUS_PLACEMENT_HAS_SIZE)
	mem->bus.base = base;
#endif
}

static inline void lg_set_bus_placement_size(lg_ttm_mem_t *mem, unsigned long size)
{
#if defined(LG_TTM_BUS_PLACEMENT_HAS_SIZE)
	mem->bus.size = size;
#endif
}

static inline void lg_add_bus_placement_offset(lg_ttm_mem_t *mem, phys_addr_t offset)
{
#if !defined(LG_TTM_BUS_PLACEMENT_HAS_SIZE)
	mem->bus.offset += offset;
#endif
}

static inline long lg_get_user_pages_remote(struct task_struct *tsk, struct mm_struct *mm,
					unsigned long start, unsigned long nr_pages,
					unsigned int gup_flags, struct page **pages,
					struct vm_area_struct **vmas, int *locked)
{
#if defined(LG_GET_USER_PAGES_REMOTE_HAS_TSK)
	return get_user_pages_remote(tsk, mm, start, nr_pages, gup_flags, pages, vmas, locked);
#elif defined(LG_GET_USER_PAGES_REMOTE_HAS_VMAS)
	return get_user_pages_remote(mm, start, nr_pages, gup_flags, pages, vmas, locked);
#else
	return get_user_pages_remote(mm, start, nr_pages, gup_flags, pages, locked);
#endif
}

static inline void lg_ttm_tt_set_populated(struct ttm_tt *ttm)
{
#if defined(LG_TTM_TT_SET_POPULATED)
	ttm_tt_set_populated(ttm);
#endif
}

#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT) || defined(LG_TTM_TT_POPULATE_HAS_BDEV)
#define lg_ttm_tt_populate_arg lg_ttm_device_t *bdev, struct ttm_tt *ttm
#else
#define lg_ttm_tt_populate_arg struct ttm_tt *ttm
#endif

static inline void lg_ttm_set_state_unbound(struct ttm_tt *ttm)
{
#if defined(LG_TTM_TT_HAS_STATE)
       ttm->state = tt_unbound;
#endif
}

#if defined(LG_TTM_BO_DRIVER_HAS_INVALIDATE)
#define lg_ttm_bo_driver_set_invalidate_caches  .invalidate_caches = &loonggpu_invalidate_caches,
#define lg_ttm_bo_driver_set_init_mem_type      .init_mem_type = &loonggpu_init_mem_type,
#else
#define lg_ttm_bo_driver_set_invalidate_caches
#define lg_ttm_bo_driver_set_init_mem_type
#endif

#if defined(LG_TTM_BO_DRIVER_HAS_TTM_TT_BIND)
#define lg_ttm_bo_driver_set_bind_unbind_destroy	.ttm_tt_bind = &loonggpu_ttm_backend_bind,     \
							.ttm_tt_unbind = &loonggpu_ttm_backend_unbind, \
							.ttm_tt_destroy = &loonggpu_ttm_backend_destroy,
#else
#define lg_ttm_bo_driver_set_bind_unbind_destroy
#endif

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)

#define lg_set_io_mem_pfn .io_mem_pfn = loonggpu_ttm_io_mem_pfn_buddy,
#define lg_fill_buffer(bo, src_data, resv, f, delayed) loonggpu_fill_buffer_buddy(bo, src_data, resv, f, delayed);

void loonggpu_res_first(struct ttm_resource *res,
		     uint64_t start, uint64_t size,
		     struct loonggpu_res_cursor *cur);
void loonggpu_res_next(struct loonggpu_res_cursor *cur, uint64_t size);

static inline u64
loonggpu_vram_mgr_block_start(struct drm_buddy_block *block)
{
	return drm_buddy_block_offset(block);
}

static inline u64
loonggpu_vram_mgr_block_size(struct drm_buddy_block *block)
{
	return (u64)PAGE_SIZE << drm_buddy_block_order(block);
}

static inline struct loonggpu_vram_mgr_resource *
to_loonggpu_vram_mgr_resource(struct ttm_resource *res)
{
	return container_of(res, struct loonggpu_vram_mgr_resource, base);
}
#else
#define lg_set_io_mem_pfn .io_mem_pfn = loonggpu_ttm_io_mem_pfn,
#define lg_fill_buffer(bo, src_data, resv, f, delayed) loonggpu_fill_buffer(bo, src_data, resv, f);
#endif

static inline int lg_ttm_sg_tt_init(
			#if defined(LG_TTM_DMA_TT)
				    struct ttm_dma_tt *ttm,
			#else
				    struct ttm_tt *ttm,
			#endif
				    u64	abo_flags,
				    struct ttm_buffer_object *bo,
				    uint32_t page_flags)
{
#if defined(LG_TTM_SG_TT_INIT_HAS_CACHING)
	enum ttm_caching caching;
	if (abo_flags & LOONGGPU_GEM_CREATE_CPU_GTT_USWC)
		caching = ttm_write_combined;
	else
		caching = ttm_cached;

	/* allocate space for the uninitialized page entries */
	return ttm_sg_tt_init(ttm, bo, page_flags, caching);
#else
	return ttm_sg_tt_init(ttm, bo, page_flags);
#endif
}

#if defined(LG_DRM_PRIME_SG_TO_DMA_ADDR_ARRAY)
#define lg_drm_prime_sg_to_addr_array	drm_prime_sg_to_dma_addr_array(ttm->sg, \
					gtt->ttm.dma_address, \
					ttm->num_pages)
#else
#define	lg_drm_prime_sg_to_addr_array	drm_prime_sg_to_page_addr_arrays(ttm->sg, \
					ttm->pages, \
					gtt->ttm.dma_address, ttm->num_pages)
#endif

static inline int lg_ttm_bo_evict_mm(lg_ttm_device_t *bdev, int type)
{
#if defined(LG_TTM_BO_EVICT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_evict_mm(bdev, type);
#else
	return ttm_resource_manager_evict_all(bdev, ttm_manager_type(bdev, type));
#endif
}

static inline int lg_ttm_bo_mem_space(struct ttm_buffer_object *bo, struct ttm_placement *placement,
				     lg_ttm_mem_t *mem, struct ttm_operation_ctx *ctx)
{
#if defined(LG_TTM_BO_MEM_SPACE_MEM_ARG_DOUBLE_PTR)
	return ttm_bo_mem_space(bo, placement, &mem, ctx);
#else
	return ttm_bo_mem_space(bo, placement, mem, ctx);
#endif
}

static inline void lg_bdev_set_no_retry(lg_ttm_device_t *bdev, bool flag)
{
#if !defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
	bdev->no_retry = flag;
#endif
}

#if defined(LG_TTM_DMA_TT)
#define lg_loonggputtm_to_dma_ttm(gtt) &gtt->ttm
#else
#define lg_loonggputtm_to_dma_ttm(gtt) NULL
#endif

#endif /* __LOONGGPU_TTM_HELPER_H__ */
