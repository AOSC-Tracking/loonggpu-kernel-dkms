#ifndef __LOONGGPU_GTT_MGR_HELPER_H__
#define __LOONGGPU_GTT_MGR_HELPER_H__

struct loonggpu_gtt_node {
	struct drm_mm_node node;
	struct ttm_buffer_object *tbo;
};

static inline lg_ttm_mem_t *lg_tbo_to_mem(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return tbo->resource;
#else
	return &tbo->mem;
#endif
}

static inline void lg_tbo_set_mem(struct ttm_buffer_object *tbo, lg_ttm_mem_t *new_mem)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	tbo->resource = new_mem;
#else
	tbo->mem = *new_mem;
#endif
}

static inline struct loonggpu_device *lg_gtt_mgr_man_to_adev(lg_ttm_manager_t *man,
						struct loonggpu_gtt_mgr *gtt_mgr)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return container_of(gtt_mgr, struct loonggpu_device, mman.gtt_mgr);
#else
	return loonggpu_ttm_adev(man->bdev);
#endif
}

static inline void lg_gtt_mgr_set_man_priv(lg_ttm_manager_t *man,
					   struct loonggpu_gtt_mgr *mgr)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	man->priv = mgr;
#endif
}

static inline void lg_gtt_mgr_fini_free_mgr(struct loonggpu_gtt_mgr *mgr)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	kfree(mgr);
#endif
}

#if defined(LG_TTM_RESOURCE_MANAGER)
#define lg_gtt_mgr_set_funcs	.alloc = loonggpu_gtt_mgr_new, \
				.free = loonggpu_gtt_mgr_del, \
				.debug = loonggpu_gtt_mgr_debug
#else
#define lg_gtt_mgr_set_funcs	.init = loonggpu_gtt_mgr_init, \
				.takedown = loonggpu_gtt_mgr_fini, \
				.get_node = loonggpu_gtt_mgr_new, \
				.put_node = loonggpu_gtt_mgr_del, \
				.debug = loonggpu_gtt_mgr_debug
#endif

static inline lg_ttm_manager_t *lg_bdev_to_ttm_man(lg_ttm_device_t *bdev, int mem_type)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return ttm_manager_type(bdev, mem_type);
#else
	return &bdev->man[mem_type];
#endif
}

static inline uint64_t lg_bdev_to_gpu_offset(lg_ttm_device_t *bdev, int mem_type)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return loonggpu_ttm_domain_start(loonggpu_ttm_adev(bdev), mem_type);
#else
	return bdev->man[mem_type].gpu_offset;
#endif
}

static inline struct loonggpu_gtt_mgr *lg_man_to_gtt_mgr(lg_ttm_manager_t *man)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return container_of(man, struct loonggpu_gtt_mgr, manager);
#else
	return man->priv;
#endif
}

#if defined(LG_TTM_RESOURCE_MANAGER)
#define lg_gtt_mgr_init_mgr(man, mgr)  mgr = container_of(man, struct loonggpu_gtt_mgr, manager);
#else
#define lg_gtt_mgr_init_mgr(man, mgr)	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL); \
					if (!mgr)                                \
						return -ENOMEM;
#endif

static inline void lg_gtt_mgr_func_setting(lg_ttm_manager_t *man,
					   struct loonggpu_device *adev,
                                           const lg_ttm_mem_func_t *loonggpu_gtt_mgr_func,
					   unsigned long size)
{
#if !defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	man->use_tt = true;
	man->func = loonggpu_gtt_mgr_func;
#if defined(LG_TTM_RESOURCE_MANAGER_INIT_HAS_BDEV)
	ttm_resource_manager_init(man, &adev->mman.bdev, size);
#else
	ttm_resource_manager_init(man, size);
#endif
#endif
}

static inline void lg_ttm_resource_manager_init(lg_ttm_manager_t *man,
                                                struct loonggpu_device *adev,
                                                unsigned long size)
{
#if !defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
#if defined(LG_TTM_RESOURCE_MANAGER_INIT_HAS_BDEV)
	ttm_resource_manager_init(man, &adev->mman.bdev, size);
#else
	ttm_resource_manager_init(man, size);
#endif
#endif
}

static inline void lg_gtt_mgr_init_set_man_and_used(lg_ttm_manager_t *man,
					lg_ttm_device_t *bdev)
{
#if !defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	ttm_set_driver_manager(bdev, TTM_PL_TT, man);
	ttm_resource_manager_set_used(man, true);
#endif
}

static inline int lg_gtt_mgr_init(struct loonggpu_device *adev, uint64_t gtt_size)
{
#if defined(LG_TTM_BO_INIT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_init_mm(&adev->mman.bdev, TTM_PL_TT, gtt_size);
#else
	return loonggpu_gtt_mgr_init(&adev->mman.gtt_mgr.manager, gtt_size);
#endif
}

static inline int lg_gtt_mgr_fini_set_used_clean_list(lg_ttm_manager_t *man,
					lg_ttm_device_t *bdev)
{
#if !defined(LG_TTM_BO_CLEAN_MM)
	ttm_resource_manager_set_used(man, false);
#if defined(LG_TTM_RESOURCE_MANAGER_EVICT_ALL)
	return ttm_resource_manager_evict_all(bdev, man);
#else
	return ttm_resource_manager_force_list_clean(bdev, man);
#endif
#endif
	return 0;
}

static inline void lg_gtt_mgr_fini_clean_up(lg_ttm_manager_t *man,
					lg_ttm_device_t *bdev)
{
#if !defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(bdev, TTM_PL_TT, NULL);
#endif
}

static inline void lg_loonggpu_gtt_mgr_fini(struct loonggpu_device *adev)
{
#if !defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	loonggpu_gtt_mgr_fini(&adev->mman.gtt_mgr.manager);
#endif
}

static inline void lg_ttm_bo_clean_mm(struct loonggpu_device *adev, int type)
{
#if defined(LG_TTM_BO_CLEAN_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	ttm_bo_clean_mm(&adev->mman.bdev, type);
#else
	ttm_range_man_fini(&adev->mman.bdev, type);
#endif
}

#if defined(LG_TTM_RESOURCE_ALLOC)
#define LG_GTT_MGR_NEW_RET_VAL -ENOSPC
#else
#define LG_GTT_MGR_NEW_RET_VAL 0
#endif

static inline phys_addr_t lg_get_bus_placement_base(lg_ttm_mem_t *mem, struct loonggpu_device *adev)
{
#if defined(LG_TTM_BUS_PLACEMENT_HAS_SIZE)
	return mem->bus.base;
#else
	return adev->gmc.aper_base;
#endif
}

static inline void lg_loonggpu_ttm_alloc_gart_set_bo_offset(struct ttm_buffer_object *bo)
{
#if defined(LG_TTM_BUFFER_OBJ_HAS_OFFSET)
	bo->offset = (lg_tbo_to_mem(bo)->start << PAGE_SHIFT) +
		lg_bdev_to_ttm_man(bo->bdev, lg_tbo_to_mem(bo)->mem_type)->gpu_offset;
#endif
}

static inline uint64_t lg_loonggpu_ttm_alloc_gart_get_bo_offset(struct loonggpu_device *adev,
                                                            struct ttm_buffer_object *bo)
{
#if defined(LG_TTM_BUFFER_OBJ_HAS_OFFSET)
	return bo->offset;
#else
	return (lg_tbo_to_mem(bo)->start << PAGE_SHIFT) +
		loonggpu_ttm_domain_start(adev, lg_tbo_to_mem(bo)->mem_type);
#endif
}

#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
#define lg_loonggpu_gtt_node struct ttm_range_mgr_node
#else
#define lg_loonggpu_gtt_node struct loonggpu_gtt_node
#endif

#if defined(LG_MAN_FUNC_ALLOC_HAS_RES)
#define lg_gtt_mgr_new_mem_reg lg_ttm_mem_t **mem
#define lg_gtt_mgr_alloc_arg &gtt_node->base
#define lg_gtt_mgr_set_res_ptr *mem = &gtt_node->base;
#define lg_gtt_get_pages(tbo, mem) PFN_UP(tbo->base.size)
#else
#define lg_gtt_mgr_new_mem_reg lg_ttm_mem_t *mem
#define lg_gtt_mgr_alloc_arg mem
#define lg_gtt_mgr_set_res_ptr
#define lg_gtt_get_pages(tbo, mem) mem->num_pages
#endif

#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	#define lg_alloc_gtt_node(gtt_node)\
	gtt_node = kzalloc(struct_size(gtt_node, mm_nodes, 1), GFP_KERNEL);
#else
	#define lg_alloc_gtt_node(gtt_node)\
	gtt_node = kzalloc(sizeof(*gtt_node), GFP_KERNEL);
#endif


static inline lg_loonggpu_gtt_node *lg_res_to_gtt_node(lg_ttm_mem_t *mem)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	return to_ttm_range_mgr_node(mem);
#else
	return mem->mm_node;
#endif
}

static inline struct drm_mm_node *lg_res_to_drm_node(lg_ttm_mem_t *mem)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	return &to_ttm_range_mgr_node(mem)->mm_nodes[0];
#else
	return &((struct loonggpu_gtt_node *)(mem->mm_node))->node;
#endif
}

static inline void lg_clear_drm_node(struct drm_mm_node *node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	memset(node, 0, sizeof(*node));
#else
	node = NULL;
#endif
}

static inline struct drm_mm_node *lg_gtt_node_to_drm_node(lg_loonggpu_gtt_node *gtt_node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	return &gtt_node->mm_nodes[0];
#else
	return &gtt_node->node;
#endif
}

#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
#define lg_gtt_node_set_tbo(tbo)
#define lg_gtt_node_set_mm_node(node)
#define lg_gtt_node_set_mm_start(val) gtt_node->base.start = val;
#define lg_gtt_del_mgr_num_pages mm_node->size
#define lg_gtt_mgr_new_check_mem tbo->resource
#else
#define lg_gtt_node_set_tbo(tbo) gtt_node->tbo = tbo;
#define lg_gtt_node_set_mm_node(node) mem->mm_node = node;
#define lg_gtt_node_set_mm_start(val) mem->start = val;
#define lg_gtt_del_mgr_num_pages mem->num_pages
#define lg_gtt_mgr_new_check_mem &tbo->mem == mem
#endif

static inline void lg_ttm_resource_init(struct ttm_buffer_object *bo,
					const struct ttm_place *place,
					lg_loonggpu_gtt_node *node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RESOURCE_H_PRESENT)
	ttm_resource_init(bo, place, &node->base);
#endif
}

static inline void lg_ttm_resource_fini(lg_ttm_manager_t *man,
					lg_loonggpu_gtt_node *node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RESOURCE_H_PRESENT)
	ttm_resource_fini(man, &node->base);
#endif
}

static inline lg_loonggpu_gtt_node *lg_drm_node_to_gtt_node(struct drm_mm_node *mm_node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RESOURCE_H_PRESENT)
	return container_of(mm_node, struct ttm_range_mgr_node, mm_nodes[0]);
#else
	return container_of(mm_node, struct loonggpu_gtt_node, node);
#endif
}

static inline struct ttm_buffer_object *lg_gtt_node_to_tbo(lg_loonggpu_gtt_node *node)
{
#if defined(LG_TTM_RANGE_MGR_NODE) && defined(LG_DRM_TTM_TTM_RESOURCE_H_PRESENT)
	return node->base.bo;
#else
	return node->tbo;
#endif
}

static inline int lg_ttm_resource_manager_evict_all(struct loonggpu_device *adev, int type)
{
#if defined(LG_TTM_RESOURCE_MANAGER_EVICT_ALL) && defined(LG_TTM_RESOURCE_MANAGER)
	lg_ttm_manager_t *man = lg_bdev_to_ttm_man(&adev->mman.bdev, type);
	return ttm_resource_manager_evict_all(&adev->mman.bdev, man);
#else
	return ttm_bo_evict_mm(&adev->mman.bdev, type);
#endif
}

#endif /* __LOONGGPU_GTT_MGR_HELPER_H__ */
