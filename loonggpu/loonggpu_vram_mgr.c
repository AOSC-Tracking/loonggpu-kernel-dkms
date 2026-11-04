#include "loonggpu.h"
#include "loonggpu_vram_mgr_helper.h"

/**
 * loonggpu_vram_mgr_vis_size - Calculate visible node size
 *
 * @adev: loonggpu device structure
 * @node: MM node structure
 *
 * Calculate how many bytes of the MM node are inside visible VRAM
 */
u64 loonggpu_vram_mgr_vis_size(struct loonggpu_device *adev,
			       struct gpu_buddy_block *block)
{
	uint64_t start = loonggpu_vram_mgr_block_start(block);
	uint64_t end = start + loonggpu_vram_mgr_block_size(block);

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
	struct ttm_resource *mem = bo->tbo.resource;
	struct loonggpu_vram_mgr_resource *vres = to_loonggpu_vram_mgr_resource(mem);
	struct gpu_buddy_block *block;
	u64 usage;

	if (loonggpu_gmc_vram_full_visible(&adev->gmc))
		return loonggpu_bo_size(bo);

	if (mem->start >= adev->gmc.visible_vram_size >> PAGE_SHIFT)
		return 0;

	list_for_each_entry(block, &vres->blocks, link)
		usage += loonggpu_vram_mgr_vis_size(adev, block);

	return usage;
}
/**
 * loonggpu_vram_mgr_usage - how many bytes are used in this domain
 *
 * @man: TTM memory type manager
 *
 * Returns how many bytes are used in this domain.
 */
uint64_t loonggpu_vram_mgr_usage(struct ttm_resource_manager *man)
{
	return ttm_resource_manager_usage(man);
}

/**
 * loonggpu_vram_mgr_vis_usage - how many bytes are used in the visible part
 *
 * @man: TTM memory type manager
 *
 * Returns how many bytes are used in the visible part of VRAM
 */
uint64_t loonggpu_vram_mgr_vis_usage(struct ttm_resource_manager *man)
{
	struct loonggpu_vram_mgr *mgr = lg_man_to_vram_mgr(man);

	return atomic64_read(&mgr->vis_usage);
}

const struct ttm_resource_manager_func loonggpu_vram_mgr_func = {
	.alloc = loonggpu_vram_mgr_new_buddy,
	.free  = loonggpu_vram_mgr_del_buddy,
	.intersects = loonggpu_vram_mgr_intersects,
	.compatible = loonggpu_vram_mgr_compatible,
	.debug = loonggpu_vram_mgr_debug_buddy,
};
