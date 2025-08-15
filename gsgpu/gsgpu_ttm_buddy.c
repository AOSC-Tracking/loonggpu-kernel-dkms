#include "gsgpu.h"
#include "gsgpu_ttm_helper.h"
#include "gsgpu_gtt_mgr_helper.h"
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#include <drm/ttm/ttm_bo.h>
#endif

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
void gsgpu_res_first(struct ttm_resource *res,
		     u64 start, u64 size,
		     struct gsgpu_res_cursor *cur)
{
	struct drm_buddy_block *block;
	struct list_head *head, *next;
	struct drm_mm_node *node;

	if (!res)
		goto fallback;

	BUG_ON(start + size > res->size);

	cur->mem_type = res->mem_type;

	switch (cur->mem_type) {
	case TTM_PL_VRAM:
		head = &to_gsgpu_vram_mgr_resource(res)->blocks;

		block = list_first_entry_or_null(head,
						 struct drm_buddy_block,
						 link);
		if (!block)
			goto fallback;

		while (start >= gsgpu_vram_mgr_block_size(block)) {
			start -= gsgpu_vram_mgr_block_size(block);

			next = block->link.next;
			if (next != head)
				block = list_entry(next, struct drm_buddy_block, link);
		}

		cur->start = gsgpu_vram_mgr_block_start(block) + start;
		cur->size = min(gsgpu_vram_mgr_block_size(block) - start, size);
		cur->remaining = size;
		cur->node = block;
		break;
	case TTM_PL_TT:
		node = to_ttm_range_mgr_node(res)->mm_nodes;
		while (start >= node->size << PAGE_SHIFT)
			start -= node++->size << PAGE_SHIFT;

		cur->start = (node->start << PAGE_SHIFT) + start;
		cur->size = min((node->size << PAGE_SHIFT) - start, size);
		cur->remaining = size;
		cur->node = node;
		break;
	default:
		goto fallback;
	}

	return;

fallback:
	cur->start = start;
	cur->size = size;
	cur->remaining = size;
	cur->node = NULL;
	WARN_ON(res && start + size > res->size);
	return;
}

void gsgpu_res_next(struct gsgpu_res_cursor *cur, u64 size)
{
	struct drm_buddy_block *block;
	struct drm_mm_node *node;
	struct list_head *next;

	BUG_ON(size > cur->remaining);

	cur->remaining -= size;
	if (!cur->remaining)
		return;

	cur->size -= size;
	if (cur->size) {
		cur->start += size;
		return;
	}

	switch (cur->mem_type) {
	case TTM_PL_VRAM:
		block = cur->node;

		next = block->link.next;
		block = list_entry(next, struct drm_buddy_block, link);

		cur->node = block;
		cur->start = gsgpu_vram_mgr_block_start(block);
		cur->size = min(gsgpu_vram_mgr_block_size(block), cur->remaining);
		break;
	case TTM_PL_TT:
		node = cur->node;

		cur->node = ++node;
		cur->start = node->start << PAGE_SHIFT;
		cur->size = min(node->size << PAGE_SHIFT, cur->remaining);
		break;
	default:
		return;
	}
}

unsigned long gsgpu_ttm_io_mem_pfn_buddy(struct ttm_buffer_object *bo,
					unsigned long page_offset)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->bdev);
	struct gsgpu_res_cursor cursor;

	gsgpu_res_first(bo->resource, page_offset << PAGE_SHIFT, 0, &cursor);

	return (adev->gmc.aper_base + cursor.start) >> PAGE_SHIFT;
}

static int gsgpu_ttm_map_buffer(struct ttm_buffer_object *bo,
				 struct ttm_resource *mem,
				 struct gsgpu_res_cursor *mm_cur,
				 unsigned int window, struct gsgpu_ring *ring,
				 bool tmz, u64 *size, u64 *addr)
{
	struct gsgpu_device *adev = ring->adev;
	unsigned int offset, num_pages, num_dw, num_bytes;
	u64 src_addr, dst_addr;
	struct gsgpu_job *job;
	struct dma_fence *fence;
	void *cpu_addr;
	u64 flags;
	unsigned int i;
	int r;

	BUG_ON(adev->mman.buffer_funcs->copy_max_bytes <
	       GSGPU_GTT_MAX_TRANSFER_SIZE * 8);

	/* Map only what can't be accessed directly */
	if (!tmz && mem->start != GSGPU_BO_INVALID_OFFSET) {
		*addr = gsgpu_ttm_domain_start(adev, mem->mem_type) +
			mm_cur->start;
		return 0;
	}

	/*
	 * If start begins at an offset inside the page, then adjust the size
	 * and addr accordingly
	 */
	offset = mm_cur->start & ~PAGE_MASK;

	num_pages = PFN_UP(*size + offset);
	num_pages = min_t(uint32_t, num_pages, GSGPU_GTT_MAX_TRANSFER_SIZE);

	*size = min(*size, (u64)num_pages * PAGE_SIZE - offset);

	*addr = adev->gmc.gart_start;
	*addr += (u64)window * GSGPU_GTT_MAX_TRANSFER_SIZE *
		GSGPU_GPU_PAGE_SIZE;
	*addr += offset;

	num_dw = ALIGN(adev->mman.buffer_funcs->copy_num_dw, 8);
	num_bytes = num_pages * 8 * GSGPU_GPU_PAGES_IN_CPU_PAGE;

	r = gsgpu_job_alloc_with_ib(adev, num_dw * 4 + num_bytes, &job);
	if (r)
		return r;

	src_addr = num_dw * 4;
	src_addr += job->ibs[0].gpu_addr;

	dst_addr = gsgpu_bo_gpu_offset(adev->gart.robj);
	dst_addr += window * GSGPU_GTT_MAX_TRANSFER_SIZE * 8;
	gsgpu_emit_copy_buffer(adev, &job->ibs[0], src_addr,
				dst_addr, num_bytes);

	gsgpu_ring_pad_ib(ring, &job->ibs[0]);
	WARN_ON(job->ibs[0].length_dw > num_dw);

	flags = gsgpu_ttm_tt_pte_flags(adev, bo->ttm, mem);

	cpu_addr = &job->ibs[0].ptr[num_dw];

	if (mem->mem_type == TTM_PL_TT) {
		dma_addr_t *dma_addr;

		dma_addr = &bo->ttm->dma_address[mm_cur->start >> PAGE_SHIFT];
		gsgpu_gart_map(adev, 0, num_pages, dma_addr, flags, cpu_addr);
	} else {
		dma_addr_t dma_address;

		dma_address = mm_cur->start;
		dma_address += adev->vm_manager.vram_base_offset;

		for (i = 0; i < num_pages; ++i) {
			gsgpu_gart_map(adev, i << PAGE_SHIFT, 1, &dma_address,
					flags, cpu_addr);
			dma_address += PAGE_SIZE;
		}
	}

	r = gsgpu_job_submit(job, &adev->mman.entity,
			     GSGPU_FENCE_OWNER_UNDEFINED, &fence, 0);
	if (r)
		gsgpu_job_free(job);

	dma_fence_put(fence);

	return 0;
}

static int gsgpu_ttm_prepare_job(struct gsgpu_device *adev,
				  bool direct_submit,
				  unsigned int num_dw,
				  struct dma_resv *resv,
				  bool vm_needs_flush,
				  struct gsgpu_job **job,
				  bool delayed)
{
	int r;

	r = gsgpu_job_alloc_with_ib(adev, num_dw * 4, job);
	if (r)
		return r;

	if (!resv)
		return 0;

	return drm_sched_job_add_resv_dependencies(&(*job)->base, resv,
						   DMA_RESV_USAGE_BOOKKEEP);
}

static int gsgpu_ttm_fill_mem(struct gsgpu_ring *ring, uint32_t src_data,
			       u64 dst_addr, uint32_t byte_count,
			       struct dma_resv *resv,
			       struct dma_fence **fence,
			       bool vm_needs_flush, bool delayed)
{
	struct gsgpu_device *adev = ring->adev;
	unsigned int num_loops, num_dw;
	struct gsgpu_job *job;
	uint32_t max_bytes;
	unsigned int i;
	int r;

	max_bytes = adev->mman.buffer_funcs->fill_max_bytes;
	num_loops = DIV_ROUND_UP_ULL(byte_count, max_bytes);
	num_dw = ALIGN(num_loops * adev->mman.buffer_funcs->fill_num_dw, 8);
	r = gsgpu_ttm_prepare_job(adev, false, num_dw, resv, vm_needs_flush,
				   &job, delayed);
	if (r)
		return r;

	for (i = 0; i < num_loops; i++) {
		uint32_t cur_size = min(byte_count, max_bytes);


		gsgpu_emit_fill_buffer(adev, &job->ibs[0], src_data, dst_addr,
					cur_size);

		dst_addr += cur_size;
		byte_count -= cur_size;
	}

	gsgpu_ring_pad_ib(ring, &job->ibs[0]);
	WARN_ON(job->ibs[0].length_dw > num_dw);
	r = gsgpu_job_submit(job, &adev->mman.entity,
			     GSGPU_FENCE_OWNER_UNDEFINED, fence, 0);
	if (r)
		gsgpu_job_free(job);

	return 0;
}

int gsgpu_fill_buffer_buddy(struct gsgpu_bo *bo,
			uint32_t src_data,
			struct dma_resv *resv,
			struct dma_fence **f,
			bool delayed)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	struct gsgpu_ring *ring = adev->mman.buffer_funcs_ring;
	struct dma_fence *fence = NULL;
	struct gsgpu_res_cursor dst;
	int r;

	if (!adev->mman.buffer_funcs_enabled) {
		DRM_ERROR("Trying to clear memory with ring turned off.\n");
		return -EINVAL;
	}

	gsgpu_res_first(bo->tbo.resource, 0, gsgpu_bo_size(bo), &dst);

	mutex_lock(&adev->mman.gtt_window_lock);
	while (dst.remaining) {
		struct dma_fence *next;
		u64 cur_size, to;

		/* Never fill more than 256MiB at once to avoid timeouts */
		cur_size = min(dst.size, 256ULL << 20);

		r = gsgpu_ttm_map_buffer(&bo->tbo, bo->tbo.resource, &dst,
					  1, ring, false, &cur_size, &to);
		if (r)
			goto error;

		r = gsgpu_ttm_fill_mem(ring, src_data, to, cur_size, resv,
					&next, true, delayed);
		if (r)
			goto error;

		dma_fence_put(fence);
		fence = next;

		gsgpu_res_next(&dst, cur_size);
	}
error:
	mutex_unlock(&adev->mman.gtt_window_lock);
	if (f)
		*f = dma_fence_get(fence);
	dma_fence_put(fence);
	return r;
}

#endif
