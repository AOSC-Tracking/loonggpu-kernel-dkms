#include "loonggpu.h"
#include "loonggpu_vram_mgr_helper.h"

/**
 * loonggpu_vram_mgr_init - init VRAM manager and DRM MM
 *
 * @man: TTM memory type manager
 * @p_size: maximum size of VRAM
 *
 * Allocate and initialize the VRAM manager.
 */
int loonggpu_vram_mgr_init(lg_ttm_manager_t *man,
				unsigned long p_size)
{
	int r;
	struct loonggpu_vram_mgr *mgr;
	struct loonggpu_device *adev;

	lg_vram_mgr_init_mgr(man, mgr);
	adev = lg_vram_mgr_man_to_loonggpu(man, mgr);

	lg_vram_mgr_init_man_func(man, adev, p_size, &loonggpu_vram_mgr_func);
	lg_vram_init_mgr(mgr);
	r = lg_vram_init_mgr_mm(mgr, 0, p_size);
	if (r)
		return r;
	lg_vram_mgr_init_drv_ptr_used(man, mgr);
	return 0;
}

/**
 * loonggpu_vram_mgr_fini - free and destroy VRAM manager
 *
 * @man: TTM memory type manager
 *
 * Destroy and free the VRAM manager, returns -EBUSY if ranges are still
 * allocated inside it.
 */
int loonggpu_vram_mgr_fini(lg_ttm_manager_t *man)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);
	int ret;

	ret = lg_vram_mgr_fini_clean(man, mgr);
	if (ret)
		return ret;

	lg_vram_mm_fini(mgr);
	lg_vram_mgr_fini_free(man, mgr);
	return 0;
}

/**
 * loonggpu_vram_mgr_vis_size - Calculate visible node size
 *
 * @adev: loonggpu device structure
 * @node: MM node structure
 *
 * Calculate how many bytes of the MM node are inside visible VRAM
 */
u64 loonggpu_vram_mgr_vis_size(struct loonggpu_device *adev,
				   lg_vram_mgr_vis_size_arg)
{
	uint64_t start = lg_vram_mgr_vis_size_start;
	uint64_t end = lg_vram_mgr_vis_size_end;

	if (start >= adev->gmc.visible_vram_size)
		return 0;

	return (end > adev->gmc.visible_vram_size ?
		adev->gmc.visible_vram_size : end) - start;
}

/**
 * loonggpu_vram_mgr_bo_visible_size - CPU visible BO size
 *
 * @bo: &loonggpu_bo buffer object (must be in VRAM)
 *
 * Returns:
 * How much of the given &loonggpu_bo buffer object lies in CPU visible VRAM.
 */
u64 loonggpu_vram_mgr_bo_visible_size(struct loonggpu_bo *bo)
{
	struct loonggpu_device *adev = loonggpu_ttm_adev(bo->tbo.bdev);
	lg_ttm_mem_t *mem = lg_bo_to_mem;
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	struct loonggpu_vram_mgr_resource *vres = to_loonggpu_vram_mgr_resource(mem);
	struct drm_buddy_block *block;
#else
	struct drm_mm_node *nodes = mem->mm_node;
	unsigned pages = mem->num_pages;
#endif
	u64 usage;

	if (loonggpu_gmc_vram_full_visible(&adev->gmc))
		return loonggpu_bo_size(bo);

	if (mem->start >= adev->gmc.visible_vram_size >> PAGE_SHIFT)
		return 0;

	lg_vram_mgr_get_usage_size
	return usage;
}
#if !defined(LG_DRM_DRM_BUDDY_H_PRESENT)
/**
 * loonggpu_vram_mgr_new - allocate new ranges
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @mem: the resulting mem object
 *
 * Allocate VRAM for the given BO.
 */
static int loonggpu_vram_mgr_new(lg_ttm_manager_t *man,
			       struct ttm_buffer_object *tbo,
			       const struct ttm_place *place,
			       lg_ttm_mem_t *mem)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);
	struct loonggpu_device *adev = lg_vram_mgr_man_to_loonggpu(man, mgr);
	struct drm_mm *mm = &mgr->mm;
	struct drm_mm_node *nodes;
	enum drm_mm_insert_mode mode;
	unsigned long lpfn, num_nodes, pages_per_node, pages_left;
	uint64_t usage = 0, vis_usage = 0;
	unsigned i;
	int r;

	lpfn = place->lpfn;
	if (!lpfn)
		lpfn = man->size;

	if (place->flags & TTM_PL_FLAG_CONTIGUOUS ||
	    loonggpu_vram_page_split == -1) {
		pages_per_node = ~0ul;
		num_nodes = 1;
	} else {
		pages_per_node = max((uint32_t)loonggpu_vram_page_split,
				     mem->page_alignment);
		num_nodes = DIV_ROUND_UP(mem->num_pages, pages_per_node);
	}

	nodes = kvmalloc_array(num_nodes, sizeof(*nodes),
			       GFP_KERNEL | __GFP_ZERO);
	if (!nodes)
		return -ENOMEM;

	mode = DRM_MM_INSERT_BEST;
	if (place->flags & TTM_PL_FLAG_TOPDOWN)
		mode = DRM_MM_INSERT_HIGH;

	mem->start = 0;
	pages_left = mem->num_pages;

	spin_lock(&mgr->lock);
	for (i = 0; i < num_nodes; ++i) {
		unsigned long pages = min(pages_left, pages_per_node);
		uint32_t alignment = mem->page_alignment;
		unsigned long start;

		if (pages == pages_per_node)
			alignment = pages_per_node;

		r = drm_mm_insert_node_in_range(mm, &nodes[i],
						pages, alignment, 0,
						place->fpfn, lpfn,
						mode);
		if (unlikely(r))
			goto error;

		usage += nodes[i].size << PAGE_SHIFT;
		vis_usage += loonggpu_vram_mgr_vis_size(adev, &nodes[i]);

		/* Calculate a virtual BO start address to easily check if
		 * everything is CPU accessible.
		 */
		start = nodes[i].start + nodes[i].size;
		if (start > mem->num_pages)
			start -= mem->num_pages;
		else
			start = 0;
		mem->start = max(mem->start, start);
		pages_left -= pages;
	}
	spin_unlock(&mgr->lock);

	atomic64_add(usage, &mgr->usage);
	atomic64_add(vis_usage, &mgr->vis_usage);

	mem->mm_node = nodes;

	return 0;

error:
	while (i--)
		drm_mm_remove_node(&nodes[i]);
	spin_unlock(&mgr->lock);

	kvfree(nodes);
	return r == -ENOSPC ? 0 : r;
}

/**
 * loonggpu_vram_mgr_del - free ranges
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @mem: TTM memory object
 *
 * Free the allocated VRAM again.
 */
static void loonggpu_vram_mgr_del(lg_ttm_manager_t *man,
				lg_ttm_mem_t *mem)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);
	struct loonggpu_device *adev = lg_vram_mgr_man_to_loonggpu(man, mgr);
	struct drm_mm_node *nodes = mem->mm_node;
	uint64_t usage = 0, vis_usage = 0;
	unsigned pages = mem->num_pages;

	if (!mem->mm_node)
		return;

	spin_lock(&mgr->lock);
	while (pages) {
		pages -= nodes->size;
		drm_mm_remove_node(nodes);
		usage += nodes->size << PAGE_SHIFT;
		vis_usage += loonggpu_vram_mgr_vis_size(adev, nodes);
		++nodes;
	}
	spin_unlock(&mgr->lock);

	atomic64_sub(usage, &mgr->usage);
	atomic64_sub(vis_usage, &mgr->vis_usage);

	kvfree(mem->mm_node);
	mem->mm_node = NULL;
}
#endif
/**
 * loonggpu_vram_mgr_usage - how many bytes are used in this domain
 *
 * @man: TTM memory type manager
 *
 * Returns how many bytes are used in this domain.
 */
uint64_t loonggpu_vram_mgr_usage(lg_ttm_manager_t *man)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	return ttm_resource_manager_usage(man);
#else
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);

	return atomic64_read(&mgr->usage);
#endif
}

/**
 * loonggpu_vram_mgr_vis_usage - how many bytes are used in the visible part
 *
 * @man: TTM memory type manager
 *
 * Returns how many bytes are used in the visible part of VRAM
 */
uint64_t loonggpu_vram_mgr_vis_usage(lg_ttm_manager_t *man)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);

	return atomic64_read(&mgr->vis_usage);
}

#if !defined(LG_DRM_DRM_BUDDY_H_PRESENT)
/**
 * loonggpu_vram_mgr_debug - dump VRAM table
 *
 * @man: TTM memory type manager
 * @printer: DRM printer to use
 *
 * Dump the table content using printk.
 */
static void loonggpu_vram_mgr_debug(lg_ttm_manager_t *man,
				  struct drm_printer *printer)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);

	spin_lock(&mgr->lock);
	drm_mm_print(&mgr->mm, printer);
	spin_unlock(&mgr->lock);

	drm_printf(printer, "man size:%llu pages, ram usage:%lluMB, vis usage:%lluMB\n",
		   man->size, loonggpu_vram_mgr_usage(man) >> 20,
		   loonggpu_vram_mgr_vis_usage(man) >> 20);
}
#endif
const lg_ttm_mem_func_t loonggpu_vram_mgr_func = {
	lg_vram_mgr_func_setting
};
