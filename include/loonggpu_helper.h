#ifndef __LOONGGPU_HELPER_H__
#define __LOONGGPU_HELPER_H__

#include "loonggpu_drm.h"
#include <linux/dma-buf.h>
#include <linux/mmu_context.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <linux/version.h>
#include <drm/gpu_scheduler.h>
#include <drm/drm_syncobj.h>
#include <drm/drm_cache.h>
#include <drm/drm_file.h>
#include <drm/drm_bridge.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fb_helper.h>
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#include <drm/ttm/ttm_bo.h>
#else
#include <drm/ttm/ttm_bo_api.h>
#endif
#include <linux/pwm.h>
#include "loonggpu.h"
#if defined(LG_LINUX_APERTURE_H_PRESENT)
#include <linux/aperture.h>
#endif
#include <linux/vgaarb.h>
#include <linux/console.h>
#if defined(LG_DRM_DRM_APERTURE_H_PRESENT)
#include <drm/drm_aperture.h>
#endif
#include <drm/drm_drv.h>
#include <linux/dma-mapping.h>
#if defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
#include <linux/pci-dma-compat.h>
#endif
#if defined(LG_DRM_TTM_TTM_PAGE_ALLOC_H_PRESENT)
#include <drm/ttm/ttm_page_alloc.h>
#endif
#if defined(LG_DRM_DRM_IRQ_H_PRESENT)
#include <drm/drm_irq.h>
#endif

/*
 * drm_sched_hw_job_reset() was replaced with drm_seched_stop.
 *
 * fix case 7.
 */
static inline void lg_drm_sched_stop(struct drm_gpu_scheduler *sched,
				     struct drm_sched_job *bad)
{
#if defined(LG_DRM_SCHED_STOP_PRESENT)
	drm_sched_stop(sched, bad);
#else
	drm_sched_hw_job_reset(sched, bad);
#endif
}

/*
 * Use drm_sched_job_cleanup() .
 *
 */
static inline void lg_drm_sched_job_cleanup(struct drm_sched_job *job)
{
#if defined(LG_DRM_SCHED_JOB_CLEANUP_PRESENT)
	drm_sched_job_cleanup(job);
#endif
}

/*
 * If the function drm_fb_helper_fill_info() is not present,
 * implement it.
 *
 * fix case 22.
 */
static inline void lg_drm_fb_helper_fill_info(struct fb_info *info,
					      struct drm_fb_helper *fb_helper,
					      struct drm_fb_helper_surface_size *sizes)
{
#if defined(LG_DRM_FB_HELPER_FILL_INFO_PRESENT)
	drm_fb_helper_fill_info(info, fb_helper, sizes);
#else
	struct drm_framebuffer *fb = fb_helper->fb;

	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->format->depth);
	drm_fb_helper_fill_var(info, fb_helper,
			       sizes->fb_width, sizes->fb_height);

	info->par = fb_helper;
	snprintf(info->fix.id, sizeof(info->fix.id), "%sdrmfb",
		 fb_helper->dev->driver->name);
#endif
}

/*
 * fix case 1/10/13
 */
static inline int lg_loonggpu_ttm_bo_device_init(struct loonggpu_device *adev,
					#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
					      struct ttm_device_funcs *driver
					#else
					      struct ttm_bo_driver *driver
					#endif
					     )
{
#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
	return ttm_device_init(&adev->mman.bdev,
				driver, adev->dev,
				adev->ddev->anon_inode->i_mapping,
				adev->ddev->vma_offset_manager,
				adev->need_swiotlb,
				dma_addressing_limited(adev->dev));
#elif defined(LG_TTM_BO_DEVICE_INIT_HAS_GLOB_ARG)
	return ttm_bo_device_init(&adev->mman.bdev,
				  adev->mman.bo_global_ref.ref.object,
				  driver,
				  adev->ddev->anon_inode->i_mapping,
				  DRM_FILE_PAGE_OFFSET,
				  adev->need_dma32);
#elif defined(LG_TTM_BO_DEVICE_INIT_HAS_MAN)
	return ttm_bo_device_init(&adev->mman.bdev,
				  driver,
				  adev->ddev->anon_inode->i_mapping,
				  adev->ddev->vma_offset_manager,
				  adev->need_dma32);
#else
	return ttm_bo_device_init(&adev->mman.bdev,
				  driver,
				  adev->ddev->anon_inode->i_mapping,
				  adev->need_dma32);
#endif
}

/*
 * fix case 25.
 */
static inline int lg_ttm_eu_reserve_buffers(struct ww_acquire_ctx *ticket,
					    struct list_head *list, bool intr,
					    struct list_head *dups, bool del_lru)
{
#if defined(LG_TTM_EU_RESERVE_BUFFERS_HAS_DEL_LRU_ARG)
	return ttm_eu_reserve_buffers(ticket, list, intr, dups, del_lru);
#else
	return ttm_eu_reserve_buffers(ticket, list, intr, dups);
#endif
}

/*
 * fix case 26
 */
static inline int lg_drm_syncobj_find_fence(struct drm_file *file_private,
					    u32 handle, u64 point, u64 flags,
					    struct dma_fence **fence)
{
#if defined(LG_DRM_SYNCOBJ_FIND_FENCE_HAS_FLAGS_ARG)
	return drm_syncobj_find_fence(file_private, handle, point, flags, fence);
#else
	return drm_syncobj_find_fence(file_private, handle, fence);
#endif
}

#if defined(LG_TTM_BO_MOVE_TO_LRU_TAIL_HAS_BULK_ARG)
#define lg_ttm_bo_move_define_args struct ttm_buffer_object *bo, struct ttm_lru_bulk_move *bulk
#define lg_ttm_bo_move_pass_args(buffer_obj, bulk) buffer_obj, bulk
#else
#define lg_ttm_bo_move_define_args struct ttm_buffer_object *bo
#define lg_ttm_bo_move_pass_args(buffer_obj, bulk) buffer_obj
#endif

static inline void lg_ttm_bo_move_to_lru_tail(lg_ttm_bo_move_define_args)
{
#if defined(LG_TTM_BO_MOVE_TO_LRU_TAIL_HAS_BULK_ARG)
	ttm_bo_move_to_lru_tail(bo, bulk);
#else
	ttm_bo_move_to_lru_tail(bo);
#endif
}

/*
 * fix case 35
 */
static inline int lg_drm_hdmi_avi_infoframe_from_display_mode(
						struct hdmi_avi_infoframe *frame,
						struct drm_connector *connector,
						const struct drm_display_mode *mode)
{
#if defined(LG_DRM_HDMI_AVI_INFOFRAME_FROM_DISPLAY_MODE_HAS_CONNECTOR_ARG) || \
    defined(LG_DRM_HDMI_AVI_INFOFRAME_FROM_DISPLAY_MODE_HAS_CONST_CONNECTOR_ARG)
	return drm_hdmi_avi_infoframe_from_display_mode(frame, connector, mode);
#else
	bool is_hdmi2_sink;

	/*
         * FIXME: sil-sii8620 doesn't have a connector around when
         * we need one, so we have to be prepared for a NULL connector.
         */
	if (!connector)
		is_hdmi2_sink = true;
	else
		is_hdmi2_sink = connector->display_info.hdmi.scdc.supported ||
					(connector->display_info.color_formats &
					 DRM_COLOR_FORMAT_YCRCB420);

	return drm_hdmi_avi_infoframe_from_display_mode(frame, mode, is_hdmi2_sink);
#endif
}

/*
 * fix case 5
 */
lg_dma_resv_t *lg_loonggpu_gem_prime_res_obj(struct drm_gem_object *obj);

#if defined(LG_DRM_DRIVER_HAS_GEM_PRIME_RES_OBJ)
#define LG_DRM_DRIVER_GEM_PRIME_RES_OBJ_CALLBACK \
		.gem_prime_res_obj = lg_loonggpu_gem_prime_res_obj,
#else
#define LG_DRM_DRIVER_GEM_PRIME_RES_OBJ_CALLBACK
#endif

/*
 * fix case 24
 */
static inline void lg_ttm_validate_buffer_set_shared(struct ttm_validate_buffer *valid_buf, unsigned int num_shared)
{
#if defined(LG_TTM_VALIDATE_BUFFER_HAS_NUM_SHARED)
	valid_buf->num_shared = num_shared;
#else
	valid_buf->shared = num_shared;
#endif
}

static inline void lg_ttm_validate_buffer_copy_shared(struct ttm_validate_buffer *dst, struct ttm_validate_buffer *src)
{
#if defined(LG_TTM_VALIDATE_BUFFER_HAS_NUM_SHARED)
	dst->num_shared = src->num_shared;
#else
	dst->shared = src->shared;
#endif
}

/*
 * fix case 34
 */
static inline bool lg_drm_crtc_state_async_flip(struct drm_crtc_state *state)
{
#if defined(LG_DRM_CRTC_STATE_HAS_ASYNC_FLIP)
	return state->async_flip;
#else
	return (state->pageflip_flags & DRM_MODE_PAGE_FLIP_ASYNC);
#endif
}

/*
 * fix case 32
 */
static inline bool lg_drm_need_swiotlb(int dma_bits)
{
#if defined(LG_DRM_NEED_SWIOTLB_PRESENT)
	return drm_need_swiotlb(dma_bits);
#else
	return drm_get_max_iomem() > ((u64)1 << dma_bits);
#endif
}

/*
 * fix case 4/28
 */
static inline struct dma_buf *lg_drm_gem_prime_export(struct drm_gem_object *gobj,
						      int flags)
{
#if defined(LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG)
	return drm_gem_prime_export(gobj->dev, gobj, flags);
#else
	return drm_gem_prime_export(gobj, flags);
#endif
}

struct dma_buf *lg_loonggpu_gem_prime_export(
#if defined(LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG)
					  struct drm_device *dev,
#endif
					  struct drm_gem_object *gobj,
					  int flags);

static inline void lg_loongson_screen_state(int state)
{
#if defined(LG_LOONGSON_SCREEN_STATE)
	loongson_screen_state(state);
#endif
}

#if defined(LG_DRM_CONNECTOR_FOR_EACH_POSSIBLE_ENCODER_HAS_I)
#define lg_drm_connector_for_each_possible_encoder(connector, encoder, __i)\
	drm_connector_for_each_possible_encoder(connector, encoder, __i)
#else
#define lg_drm_connector_for_each_possible_encoder(connector, encoder, __i)\
	drm_connector_for_each_possible_encoder(connector, encoder)
#endif

static inline struct i2c_client *lg_i2c_new_device(struct i2c_adapter *adap,
						   struct i2c_board_info const *info)
{
#if defined(LG_I2C_NEW_CLIENT_DEVICE)
	return i2c_new_client_device(adap, info);
#else
	return i2c_new_device(adap, info);
#endif
}

static inline void lg_drm_gem_object_put(struct drm_gem_object *gobj)
{
#if defined(LG_DRM_GEM_OBJECT_PUT_UNLOCKED)
	drm_gem_object_put_unlocked(gobj);
#else
	drm_gem_object_put(gobj);
#endif
}

static inline int lg_drm_bridge_attach(struct drm_encoder *encoder,
					struct drm_bridge *bridge,
					struct drm_bridge *previous,
					int flags)
{
#if defined(LG_DRM_BRIDGE_ATTACH_HAS_FLAGS_ARG)
	return drm_bridge_attach(encoder, bridge, previous, (enum drm_bridge_attach_flags)flags);
#else
	return drm_bridge_attach(encoder, bridge, previous);
#endif
}


#if defined(LG_DRM_BRIDGE_ATTACH_HAS_FLAGS_ARG)
#define lg_bridge_phy_attach_args struct drm_bridge *bridge, enum drm_bridge_attach_flags flags
#else
#define lg_bridge_phy_attach_args struct drm_bridge *bridge
#endif

#if defined(LG_DRM_CRTC_HELPER_FUNCS_HAS_GET_SCANOUT_POSITION)
#define lg_get_scanout_position_setting(func) .get_scanout_position = func
#else
#define lg_get_scanout_position_setting(func)
#endif

#if defined(LG_DRM_DRIVER_HAS_GET_VBLANK_TIMESTAMP)
#define lg_drm_driver_get_vblank_timestamp_setting() .get_vblank_timestamp = drm_calc_vbltimestamp_from_scanoutpos,
#define lg_drm_driver_get_scanout_position_setting(func) .get_scanout_position = func,
#define lg_drm_driver_get_vblank_counter_setting(func)   .get_vblank_counter = func,
#else
#define lg_drm_driver_get_vblank_timestamp_setting(func)
#define lg_drm_driver_get_scanout_position_setting(func)
#define lg_drm_driver_get_vblank_counter_setting(func)
#endif

#if defined(LG_DRM_DRIVER_HAS_GEM_FREE)
#define lg_drm_driver_gem_free_setting(func) .gem_free_object_unlocked = func,
#define lg_drm_driver_gem_open_setting(func) .gem_open_object = func,
#define lg_drm_driver_gem_close_setting(func) .gem_close_object = func,
#define lg_drm_driver_gem_prime_get_sg_setting(func) .gem_prime_get_sg_table = func,
#define lg_drm_driver_gem_prime_vmap_setting(func) .gem_prime_vmap = func,
#define lg_drm_driver_gem_prime_vunmap_setting(func) .gem_prime_vunmap = func,
#define lg_drm_driver_gem_prime_mmap_setting(func) .gem_prime_mmap = func,
#else
#define lg_drm_driver_gem_free_setting(func)
#define lg_drm_driver_gem_open_setting(func)
#define lg_drm_driver_gem_close_setting(func)
#define lg_drm_driver_gem_prime_get_sg_setting(func)
#define lg_drm_driver_gem_prime_vmap_setting(func)
#define lg_drm_driver_gem_prime_vunmap_setting(func)
#define lg_drm_driver_gem_prime_mmap_setting(func)
#endif

#if defined(LG_DRM_DRIVER_HAS_GEM_PRIME_EXPORT)
#define lg_drm_driver_gem_prime_export_setting(func) .gem_prime_export = func,
#define lg_drm_driver_prime_fd_to_handle_setting(func) .prime_fd_to_handle = func,
#define lg_drm_driver_prime_handle_to_fd_setting(func) .prime_handle_to_fd = func,
#else
#define lg_drm_driver_gem_prime_export_setting(func)
#define lg_drm_driver_prime_fd_to_handle_setting(func)
#define lg_drm_driver_prime_handle_to_fd_setting(func)
#endif

static inline int lg_drm_fb_helper_init(struct drm_device *dev,
					struct drm_fb_helper *fb_helper,
					int max_conn_count)
{
#if defined(LG_DRM_FB_HELP_INIT_HAS_MAX_CONN_COUNT)
	return drm_fb_helper_init(dev, fb_helper, max_conn_count);
#else
	return drm_fb_helper_init(dev, fb_helper);
#endif
}

static inline struct rw_semaphore *lg_mm_struct_get_sem(struct mm_struct *mm)
{
#if defined(LG_MM_STRUCT_HAS_MMAP_LOCK)
	return &mm->mmap_lock;
#else
	return &mm->mmap_sem;
#endif
}

static inline struct sg_table *lg_drm_prime_pages_to_sg(struct drm_device *dev,
                                        struct page **pages, unsigned int nr_pages)
{
#if defined(LG_DRM_PRIME_PAGES_TO_SG_HAS_DRM_DEVICE)
	return drm_prime_pages_to_sg(dev, pages, nr_pages);
#else
	return drm_prime_pages_to_sg(pages, nr_pages);
#endif
}

static inline int lg_ttm_fbdev_mmap(struct vm_area_struct *vma, struct ttm_buffer_object *bo)
{
#if defined(LG_TTM_FBDEV_MMAP) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_fbdev_mmap(vma, bo);
#else
	if (vma->vm_pgoff != 0)
		return -EACCES;

        return ttm_bo_mmap_obj(vma, bo);
#endif
}

static inline int lg_atomic_read_drm_open_count(struct drm_device *dev)
{
#if defined(LG_DRM_DEVICE_HAS_ATOMIC_OPEN_COUNT)
	return atomic_read(&dev->open_count);
#else
	return dev->open_count;
#endif
}

#if defined(LG_DRM_SCHED_BACKEND_OPS_TIMEDOUT_JOB_RET_SCHED_STAT)
#define lg_loonggpu_job_timedout_ret enum drm_gpu_sched_stat
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
#define LG_DRM_GPU_SCHED_STAT_NOMINAL DRM_GPU_SCHED_STAT_RESET
#else
#define LG_DRM_GPU_SCHED_STAT_NOMINAL DRM_GPU_SCHED_STAT_NOMINAL
#endif
#else
#define lg_loonggpu_job_timedout_ret void
#define LG_DRM_GPU_SCHED_STAT_NOMINAL
#endif

static inline void lg_drm_sched_entity_push_job(struct drm_sched_job *sched_job,
					struct drm_sched_entity *entity)
{
#if defined(LG_DRM_SCHED_ENTITY_PUSH_JOB_REMOVES_ENTITY)
	drm_sched_entity_push_job(sched_job);
#else
	drm_sched_entity_push_job(sched_job, entity);
#endif
}

static inline void lg_drm_sched_job_arm(struct drm_sched_job *job)
{
#if defined(LG_DRM_SCHED_JOB_ARM)
	drm_sched_job_arm(job);
#endif
}

#if defined(LG_DRM_SCHED_BACKEDN_OPS_HAS_PREPARE)
#define lg_sched_ops_set_prepare .prepare_job = loonggpu_job_dependency,
#else
#define lg_sched_ops_set_prepare .dependency = loonggpu_job_dependency,
#endif

#if defined(LG_DMA_RESV_USAGE_BOOKKEEP_PRESENT)
#define LG_DMA_RESV_USAGE_BOOKKEEP	DMA_RESV_USAGE_BOOKKEEP
#define LG_DMA_RESV_USAGE_WRITE		DMA_RESV_USAGE_WRITE
#define LG_DMA_RESV_USAGE_KERNEL	DMA_RESV_USAGE_KERNEL
#define LG_DMA_RESV_USAGE_READ		DMA_RESV_USAGE_READ
#else
#define LG_DMA_RESV_USAGE_BOOKKEEP	0
#define LG_DMA_RESV_USAGE_WRITE		0
#define LG_DMA_RESV_USAGE_KERNEL	0
#define LG_DMA_RESV_USAGE_READ		0
#endif

#if defined(LG_ATOMIC_CHECK_HAS_DRM_ATOMIC_STATE)
#define lg_dc_plane_atomic_check_args	struct drm_plane *plane, struct drm_atomic_state *state
#else
#define lg_dc_plane_atomic_check_args	struct drm_plane *plane, struct drm_plane_state *state
#endif

#if defined(LG_VGA_CLIENT_REGISTER_HAS_COOKIE)
#define lg_vga_set_decode_args void *cookie, bool state
#define lg_vga_set_decode_get_adev cookie
#else
#define lg_vga_set_decode_args struct pci_dev *pdev, bool state
#define lg_vga_set_decode_get_adev ((struct drm_device *)pci_get_drvdata(pdev))->dev_private
#endif
static inline void lg_vga_client_register(struct pci_dev *pdev, void *cookie,
					  void (*irq_set_state)(void *cookie, bool state),
					  unsigned int (*set_vga_decode)(lg_vga_set_decode_args))
{
#if defined(LG_VGA_CLIENT_REGISTER_HAS_COOKIE)
	vga_client_register(pdev, cookie, irq_set_state, set_vga_decode);
#else
	vga_client_register(pdev, set_vga_decode);
#endif
}

static inline int lg_ttm_bo_lock_delayed_workqueue(struct loonggpu_device *adev)
{
#if defined(LG_TTM_BO_DELAYED_WORKQUEUE) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_lock_delayed_workqueue(&adev->mman.bdev);
#endif
	return 0;
}

static inline void lg_ttm_bo_unlock_delayed_workqueue(struct loonggpu_device *adev, int sched)
{
#if defined(LG_TTM_BO_DELAYED_WORKQUEUE) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_unlock_delayed_workqueue(&adev->mman.bdev, sched);
#endif
}

static inline struct fb_info *lg_drm_fb_helper_alloc_info(struct drm_fb_helper *helper)
{
#if defined(LG_DRM_FB_HELPER_ALLOC_INFO)
	return drm_fb_helper_alloc_info(helper);
#else
	return drm_fb_helper_alloc_fbi(helper);
#endif
}

static inline void lg_drm_fb_helper_unregister_info(struct drm_fb_helper *helper)
{
#if defined(LG_DRM_FB_HELPER_ALLOC_INFO)
	drm_fb_helper_unregister_info(helper);
#else
	drm_fb_helper_unregister_fbi(helper);
#endif
}

static inline void lg_set_info_apertures(struct fb_info *info, struct loonggpu_device *adev)
{
#if defined(LG_FB_INFO_HAS_APERTURES)
	/* setup aperture base/size for vesafb takeover */
	info->apertures->ranges[0].base = adev->ddev->mode_config.fb_base;
	info->apertures->ranges[0].size = adev->gmc.aper_size;
#endif
}

static inline void lg_drm_fb_helper_prepare(struct drm_device *dev, struct drm_fb_helper *helper,
						unsigned int bpp,
						const struct drm_fb_helper_funcs *funcs)
{
#if defined(LG_DRM_FB_HELPER_PREPARE_HAS_BPP)
	drm_fb_helper_prepare(dev, helper, bpp, funcs);
#else
	drm_fb_helper_prepare(dev, helper, funcs);
#endif
}

static inline void lg_drm_fb_helper_single_add_all_connectors(struct drm_fb_helper *helper)
{
#if defined(LG_DRM_FB_HELPER_SINGLE_ADD_ALL_CONNECTORS)
	drm_fb_helper_single_add_all_connectors(helper);
#endif
}


static inline void lg_drm_fb_helper_initial_config(struct drm_fb_helper *helper, unsigned int bpp)
{
#if defined(LG_DRM_FB_HELPER_PREPARE_HAS_BPP)
	drm_fb_helper_initial_config(helper);
#else
	drm_fb_helper_initial_config(helper, bpp);
#endif
}

static inline void lg_pwm_free(struct pwm_device *pwm)
{
#if defined(LG_PWM_FREE)
	pwm_free(pwm);
#else
	pwm_put(pwm);
#endif
}

static inline void lg_dc_mode_config_init_fb_base(struct loonggpu_device *adev)
{
#if defined(LG_DRM_MODE_CONFIG_HAS_FB_BASE)
	adev->ddev->mode_config.fb_base = adev->gmc.aper_base;
#endif
}

#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#define lg_get_bo_mem_type(bo)		bo->tbo.resource->mem_type
#define lg_get_bo_node_start(bo)	to_ttm_range_mgr_node(bo->tbo.resource)->mm_nodes->start
#else
#define lg_get_bo_mem_type(bo)		bo->tbo.mem.mem_type
#define lg_get_bo_node_start(bo)	((struct drm_mm_node *)(bo->tbo.mem.mm_node))->start
#endif

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
#define lg_get_bo_buddy_start		loonggpu_res_first(bo->tbo.resource, 0, gobj->size, &cursor);
#define lg_get_bo_start			cursor.start
#else
#define lg_get_bo_buddy_start
#define lg_get_bo_start			lg_get_bo_node_start(bo)
#endif

static inline int lg_pci_set_dma_mask(struct pci_dev *pci_dev, struct device *dev, u64 mask)
{
#if defined(LG_PCI_SET_DMA_MASK) && defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	return pci_set_dma_mask(pci_dev, mask);
#else
	return dma_set_mask(dev, mask);
#endif
}

static inline int lg_pci_set_consistent_dma_mask(struct pci_dev *pci_dev, struct device *dev, u64 mask)
{
#if defined(LG_PCI_SET_DMA_MASK) && defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	return pci_set_consistent_dma_mask(pci_dev, mask);
#else
	return dma_set_coherent_mask(dev, mask);
#endif
}

#if defined(LG_TTM_GLOBAL)
typedef struct ttm_global lg_ttm_global_t;
#else
typedef struct ttm_bo_global lg_ttm_global_t;
#endif

#if defined(LG_ATOMIC_CHECK_HAS_CRTC_STATE_ARG)
typedef struct drm_crtc_state lg_atomic_check_state_arg;
#else
typedef struct drm_atomic_state lg_atomic_check_state_arg;
#endif

static inline lg_ttm_global_t *lg_get_ttm_bo_glob(lg_ttm_device_t *bdev)
{
#if !defined(LG_TTM_BO_DEVICE_HAS_GLOB)
#if defined(LG_TTM_GLOBAL)
	return &ttm_glob;
#else
	return &ttm_bo_glob;
#endif
#else
	return bdev->glob;
#endif
}

static inline void lg_spin_lock_glob_lock(lg_ttm_global_t *glob,
					  struct loonggpu_device *adev)
{
#if defined(LG_TTM_DEVICE_HAS_LRU_LOCK)
	spin_lock(&adev->mman.bdev.lru_lock);
#else
	spin_lock(&glob->lru_lock);
#endif
}

static inline void lg_spin_lock_glob_unlock(lg_ttm_global_t *glob,
					  struct loonggpu_device *adev)
{
#if defined(LG_TTM_DEVICE_HAS_LRU_LOCK)
	spin_unlock(&adev->mman.bdev.lru_lock);
#else
	spin_unlock(&glob->lru_lock);
#endif
}

#if defined(LG_DRM_PLANE_HELPER_DESTROY)
#define lg_setting_dc_destroy  .destroy = drm_plane_helper_destroy
#else
#define lg_setting_dc_destroy  .destroy = dc_plane_destroy
#endif

#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
#define lg_bo_move_ttm_mem_place_arg lg_ttm_mem_t *new_mem, struct ttm_place *hop
#define lg_define_ttm_bo_driver static struct ttm_device_funcs loonggpu_bo_driver = { \
						.ttm_tt_create = &loonggpu_ttm_tt_create, \
						.ttm_tt_populate = &loonggpu_ttm_tt_populate, \
						.ttm_tt_unpopulate = &loonggpu_ttm_tt_unpopulate, \
						.ttm_tt_destroy = &loonggpu_ttm_backend_destroy, \
						.eviction_valuable = &loonggpu_ttm_bo_eviction_valuable, \
						.evict_flags = &loonggpu_evict_flags, \
						.move = &loonggpu_bo_move, \
						.io_mem_reserve = &loonggpu_ttm_io_mem_reserve, \
						.io_mem_pfn = loonggpu_ttm_io_mem_pfn_buddy, \
						.access_memory = &loonggpu_ttm_access_memory, \
						};
#else
#define lg_bo_move_ttm_mem_place_arg lg_ttm_mem_t *new_mem
#define lg_define_ttm_bo_driver static struct ttm_bo_driver loonggpu_bo_driver = { \
					.ttm_tt_create = &loonggpu_ttm_tt_create, \
					.ttm_tt_populate = &loonggpu_ttm_tt_populate, \
					.ttm_tt_unpopulate = &loonggpu_ttm_tt_unpopulate, \
					lg_ttm_bo_driver_set_invalidate_caches \
					lg_ttm_bo_driver_set_init_mem_type \
					lg_ttm_bo_driver_set_bind_unbind_destroy \
					.eviction_valuable = loonggpu_ttm_bo_eviction_valuable, \
					.evict_flags = &loonggpu_evict_flags, \
					.move = &loonggpu_bo_move, \
					.verify_access = &loonggpu_verify_access, \
					.move_notify = &loonggpu_bo_move_notify, \
					.fault_reserve_notify = &loonggpu_bo_fault_reserve_notify, \
					.io_mem_reserve = &loonggpu_ttm_io_mem_reserve, \
					.io_mem_free = &loonggpu_ttm_io_mem_free, \
					.io_mem_pfn = loonggpu_ttm_io_mem_pfn, \
					.access_memory = &loonggpu_ttm_access_memory \
					};
#endif
static void lg_loonggpu_ttm_bo_device_fini(struct loonggpu_device *adev)
{
#if defined(LG_DRM_TTM_TTM_DEVICE_H_PRESENT)
	ttm_device_fini(&adev->mman.bdev);
#else
	ttm_bo_device_release(&adev->mman.bdev);
#endif
}

#if defined(LG_TTM_BO_MMAP) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
#define lg_set_file_mmap_ops .mmap = loonggpu_mmap,
#else
#define lg_set_file_mmap_ops .mmap = drm_gem_mmap,
#endif

static inline void lg_mem_bus_caching(lg_ttm_mem_t *mem)
{
#if defined(LG_DRM_TTM_TTM_CACHING_H_PRESENT)
	mem->bus.caching = ttm_write_combined;
#endif
}

static inline long lg_get_user_pages(uint64_t userptr, unsigned num_pages,
				     unsigned int flags, struct page **pages,
				     struct vm_area_struct **vmas)
{
#if defined(LG_GET_USER_PAGES_HAS_VMAS)
	return get_user_pages(userptr, num_pages, flags, pages, vmas);
#else
	return get_user_pages(userptr, num_pages, flags, pages);
#endif
}

static inline struct drm_sched_rq *lg_sched_to_sched_rq(struct drm_gpu_scheduler *sched,
					enum drm_sched_priority priority)
{
#if defined(LG_DRM_SCHED_INIT_HAS_DEVICE_RQ) || defined (LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ) || defined (LG_DRM_SCHED_INIT_HAS_STRUCT_ARG)
	return sched->sched_rq[priority];
#else
	return &sched->sched_rq[priority];
#endif
}

static inline int lg_drm_sched_init(struct loonggpu_ring *ring,
				const struct drm_sched_backend_ops *ops,
				unsigned num_hw_submission,
				unsigned hang_limit,
				long timeout, struct loonggpu_device *adev)
{
#if defined (LG_DRM_SCHED_INIT_HAS_STRUCT_ARG)
	const struct drm_sched_init_args args = {
		.ops = ops,
		.num_rqs = DRM_SCHED_PRIORITY_COUNT,
		.credit_limit = num_hw_submission,
		.hang_limit = hang_limit,
		.timeout = timeout,
		.name = ring->name,
		.dev = adev->dev,
	};

	return drm_sched_init(&ring->sched, &args);
#elif defined(LG_DRM_SCHED_INIT_HAS_DEVICE)
	return drm_sched_init(&ring->sched, ops,
			   num_hw_submission, hang_limit,
			   timeout, NULL,
			   NULL, ring->name,
			   adev->dev);
#elif defined(LG_DRM_SCHED_INIT_HAS_DEVICE_RQ)
	return drm_sched_init(&ring->sched, ops, DRM_SCHED_PRIORITY_COUNT,
			   num_hw_submission, hang_limit,
			   timeout, NULL,
			   NULL, ring->name,
			   adev->dev);
#elif defined(LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ)
	return drm_sched_init(&ring->sched, ops, NULL, DRM_SCHED_PRIORITY_COUNT,
			   num_hw_submission, hang_limit,
			   timeout, NULL,
			   NULL, ring->name,
			   adev->dev);
#else
	return drm_sched_init(&ring->sched, ops,
			   num_hw_submission, hang_limit,
			   timeout, ring->name);
#endif
}

static int lg_map_page(struct loonggpu_device *adev, struct page *dummy_page)
{
#if defined(LG_PCI_MAP_PAGE) && defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	adev->dummy_page_addr = pci_map_page(adev->pdev, dummy_page, 0,
					     PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	if (pci_dma_mapping_error(adev->pdev, adev->dummy_page_addr)) {
		dev_err(&adev->pdev->dev, "Failed to DMA MAP the dummy page\n");
		adev->dummy_page_addr = 0;
		return -ENOMEM;
	}
#else
	adev->dummy_page_addr = dma_map_page(adev->dev, dummy_page, 0,
					     PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(&adev->pdev->dev, adev->dummy_page_addr)) {
		dev_err(adev->dev, "Failed to DMA MAP the dummy page\n");
		adev->dummy_page_addr = 0;
		return -ENOMEM;
	}
#endif

	return 0;
}

static void lg_unmap_page(struct loonggpu_device *adev)
{
#if defined(LG_PCI_MAP_PAGE) && defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	pci_unmap_page(adev->pdev, adev->dummy_page_addr,
		       PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
#else
	dma_unmap_page(adev->dev, adev->dummy_page_addr,
		       PAGE_SIZE, DMA_BIDIRECTIONAL);
#endif
}

#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#define lg_ttm_bo_init_reserved ttm_bo_init_reserved(&adev->mman.bdev, \
						&bo->tbo, bp->type, \
						&bo->placement, page_align, &ctx, \
						NULL, bp->resv, &loonggpu_bo_destroy)
#else
#define lg_ttm_bo_init_reserved ttm_bo_init_reserved(&adev->mman.bdev, \
						&bo->tbo, size, bp->type, \
						&bo->placement, \
						page_align, \
						&ctx, acc_size, \
						NULL, bp->resv, &loonggpu_bo_destroy)
#endif

static inline size_t lg_ttm_bo_dma_acc_size(struct loonggpu_device *adev, unsigned long bo_size,
				unsigned struct_size)
{
#if defined(LG_HAS_TTM_BO_DMA_ACC_SIZE) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_dma_acc_size(&adev->mman.bdev, bo_size, struct_size);
#else
	return 0;
#endif
}

static int lg_kick_out_firmware_fb(unsigned long base, unsigned long size, struct drm_driver *drv)
{
#if defined(LG_FB_INFO_HAS_APERTURES)
	struct apertures_struct *ap;
	bool primary = false;

	ap = alloc_apertures(1);
	if (!ap)
		return -ENOMEM;

	ap->ranges[0].base = base;
	ap->ranges[0].size = size;

	drm_fb_helper_remove_conflicting_framebuffers(ap, "loonggpudrmfb", primary);
	kfree(ap);

	return 0;
#elif defined(LG_DRM_DRM_APERTURE_H_PRESENT)
	drm_aperture_remove_conflicting_framebuffers(base, size, drv);
	return 0;
#else
	aperture_remove_conflicting_devices(base, size, drv->name);
	return 0;
#endif
}

static inline bool lg_vgacon_text_force(void)
{
#if defined(LG_VGACON_TEXT_FORCE)
	return vgacon_text_force();
#else
	return drm_firmware_drivers_only();
#endif
}

static inline void *lg_pci_alloc_consistent(struct loonggpu_device *adev)
{
#if defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	return pci_alloc_consistent(adev->pdev, adev->irq.ih.ring_size + 8,
				    &adev->irq.ih.rb_dma_addr);
#else
	return dma_alloc_coherent(&adev->pdev->dev,
				adev->irq.ih.ring_size + 8,
				&adev->irq.ih.rb_dma_addr,
				GFP_ATOMIC);
#endif
}

static inline void lg_pci_free_consistent(struct loonggpu_device *adev)
{
#if defined(LG_LINUX_PCI_DMA_COMPAT_H_PRESENT)
	pci_free_consistent(adev->pdev, adev->irq.ih.ring_size + 8,
				    (void *)adev->irq.ih.ring,
				    adev->irq.ih.rb_dma_addr);
#else
	dma_free_coherent(&adev->pdev->dev,
			adev->irq.ih.ring_size + 8,
			(void *)adev->irq.ih.ring,
			adev->irq.ih.rb_dma_addr);
#endif
}

static inline u32 lg_tbo_get_page_alignment(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return tbo->mem.page_alignment;
#else
	return tbo->page_alignment;
#endif
}

static inline int lg_drm_irq_install(struct drm_device *ddev, struct pci_dev *pdev,
					int *irq, int vec, irq_handler_t handler)
{
	int r;
#if defined(LG_DRM_DRM_IRQ_H_PRESENT)
#if !defined(CONFIG_DRM_LEGACY)
	ddev->driver->irq_handler = loonggpu_irq_handler;
#endif
	r = drm_irq_install(ddev, pdev->irq);
#else
	r = pci_irq_vector(pdev, vec);
	if (r < 0)
		return r;
	*irq = r;
	/* PCI devices require shared interrupts. */
	r = request_irq(*irq, handler, IRQF_SHARED,
			ddev->driver->name, ddev);
#endif
	return r;
}


static inline void lg_drm_irq_uninstall(struct drm_device *ddev, int irq)
{
#if defined(LG_DRM_DRM_IRQ_H_PRESENT)
	drm_irq_uninstall(ddev);
#else
	free_irq(irq, ddev);
#endif
}

static inline void lg_set_tbo_moving_fence(struct ttm_buffer_object *tbo, struct dma_fence *fence)
{
#if !defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	dma_fence_put(tbo->moving);
	tbo->moving = dma_fence_get(fence);
#endif
}

static inline void lg_ttm_move_null(struct ttm_buffer_object *tbo,
			lg_ttm_mem_t *old_mem, lg_ttm_mem_t *new_mem)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	ttm_bo_move_null(tbo, new_mem);
#else
	BUG_ON(old_mem->mm_node != NULL);
	*old_mem = *new_mem;
	new_mem->mm_node = NULL;
#endif
}

static inline lg_dma_resv_t *lg_tbo_to_resv(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT) || !defined(LG_TBO_HAS_RESV)
	return tbo->base.resv;
#else
	return tbo->resv;
#endif
}

static inline lg_dma_resv_t *lg_tbo_to_ttm_resv(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT) || !defined(LG_TBO_HAS_RESV)
	return &tbo->base._resv;
#else
	return &tbo->ttm_resv;
#endif
}

static inline uint64_t lg_mem_to_size(lg_ttm_mem_t *mem)
{
#if defined(LG_TTM_RES_HAS_NUM_PAGES)
	return mem->num_pages << PAGE_SHIFT;
#else
	return mem->size;
#endif
}

static inline uint64_t lg_mem_to_num_pages(lg_ttm_mem_t *mem)
{
#if defined(LG_TTM_RES_HAS_NUM_PAGES)
	return mem->num_pages;
#else
	return mem->size >> PAGE_SHIFT;
#endif
}

static inline uint64_t lg_tbo_to_size(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return tbo->base.size;
#else
	return tbo->num_pages << PAGE_SHIFT;
#endif
}

static inline int lg_ttm_bo_move_ttm(struct ttm_buffer_object *tbo,
			struct ttm_operation_ctx *ctx, lg_ttm_mem_t *new_mem)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return 0;
#else
	return ttm_bo_move_ttm(tbo, ctx, new_mem);
#endif
}

static inline int lg_ttm_populate_and_map_pages(struct loonggpu_device *adev,
					#if defined(LG_TTM_DMA_TT)
						struct ttm_dma_tt *ttm,
					#else
						struct ttm_tt *ttm,
					#endif
						struct ttm_operation_ctx *ctx)
{
#if defined(LG_DRM_TTM_TTM_PAGE_ALLOC_H_PRESENT)
	return ttm_populate_and_map_pages(adev->dev, ttm, ctx);
#else
	int ret, i;

	ret = ttm_pool_alloc(&adev->mman.bdev.pool, ttm, ctx);
	if (ret)
		return ret;

	for (i = 0; i < ttm->num_pages; ++i)
		ttm->pages[i]->mapping = adev->mman.bdev.dev_mapping;
	return 0;
#endif
}

static inline void lg_ttm_unmap_and_unpopulate_pages(struct loonggpu_device *adev,
					#if defined(LG_TTM_DMA_TT)
						struct ttm_dma_tt *ttm
					#else
						struct ttm_tt *ttm
					#endif
						)
{
#if defined(LG_DRM_TTM_TTM_PAGE_ALLOC_H_PRESENT)
	ttm_unmap_and_unpopulate_pages(adev->dev, ttm);
#else
	for (pgoff_t i = 0; i < ttm->num_pages; i++)
		ttm->pages[i]->mapping = NULL;
	ttm_pool_free(&adev->mman.bdev.pool, ttm);
#endif
}

static inline dma_addr_t *lg_tbo_to_dma_address(struct ttm_buffer_object *tbo)
{
#if defined(LG_TTM_DMA_TT)
	struct ttm_dma_tt *ttm;
	ttm = container_of(tbo->ttm, struct ttm_dma_tt, ttm);
	return ttm->dma_address;
#else
	return tbo->ttm->dma_address;
#endif
}

static bool loonggpu_drm_sched_dependency_optimized(struct dma_fence* fence,
					struct drm_sched_entity *entity)
{
	struct drm_gpu_scheduler *sched = entity->rq->sched;
	struct drm_sched_fence *s_fence;

	if (!fence || dma_fence_is_signaled(fence))
		return false;
	if (fence->context == entity->fence_context)
		return true;
	s_fence = to_drm_sched_fence(fence);
	if (s_fence && s_fence->sched == sched)
		return true;

	return false;
}

static inline bool lg_drm_sched_dependency_optimized(struct dma_fence *fence,
						struct drm_sched_entity *s_entity)
{
#if defined(LG_DRM_SCHED_DEPENDENCY_OPTIMIZED)
	return drm_sched_dependency_optimized(fence, s_entity);
#else
	return loonggpu_drm_sched_dependency_optimized(fence, s_entity);
#endif
}

static inline int lg_pwm_apply_state(struct pwm_device *pwm,
				     struct pwm_state *state)
{
#if defined(LG_PWM_APPLY_STATE)
	return pwm_apply_state(pwm, state);
#else
	return pwm_apply_might_sleep(pwm, state);
#endif
}

static inline struct pwm_device *lg_pwm_request(struct device *dev,
						const char *pwm_name,
						int pwm, const char *lable)
{
#if defined(LG_PWM_REQUEST)
	return pwm_request(pwm, lable);
#else
	return pwm_get(dev, pwm_name);
#endif
}

static inline unsigned long lg_get_man_size(lg_ttm_manager_t *man)
{
#if defined(LG_DRM_TTM_TTM_RESOURCE_H_PRESENT)
	return man->size;
#else
	return man->size << PAGE_SHIFT;
#endif
}

static inline unsigned long lg_get_tbo_res_size(struct ttm_buffer_object *bo)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return PFN_UP(bo->resource->size) << PAGE_SHIFT;
#else
	return bo->mem.num_pages << PAGE_SHIFT;
#endif
}

#if defined(LG_TTM_BUFFER_OBJECT_HAS_BASE) || defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#define lg_gbo_to_gem_obj(bo) bo->tbo.base
#else
#define lg_gbo_to_gem_obj(bo) bo->gem_base
#endif

static inline void lg_drm_syncobj_add_point(struct drm_syncobj *syncobj,
						void *chain,
						struct dma_fence *fence,
						uint64_t point)
{

#ifdef LG_DRM_DRIVER_SYNCOBJ_TIMELINE_PRESENT
	drm_syncobj_add_point(syncobj, (struct dma_fence_chain *)chain, fence, point);
#endif
}

static inline bool lg_ring_sched_thread_avai(struct loonggpu_ring *ring)
{
	if (!ring)
		return false;
#if defined(LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ) || defined(LG_DRM_SCHED_INIT_HAS_STRUCT_ARG) 
	if (!drm_sched_wqueue_ready(&ring->sched))
		return false;
#else
	if (!ring->sched.thread)
		return false;
#endif
	return true;
}

static inline void lg_ring_sched_thread_park(struct loonggpu_ring *ring)
{
#if defined(LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ) || defined(LG_DRM_SCHED_INIT_HAS_STRUCT_ARG)
	drm_sched_wqueue_stop(&ring->sched);
#else
	kthread_park(ring->sched.thread);
#endif
}

static inline void lg_ring_sched_thread_unpark(struct loonggpu_ring *ring)
{
#if defined(LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ) || defined(LG_DRM_SCHED_INIT_HAS_STRUCT_ARG)
	drm_sched_wqueue_start(&ring->sched);
#else
	kthread_unpark(ring->sched.thread);
#endif
}

static inline int lg_drm_sched_job_init(struct drm_sched_job *job,
					struct drm_sched_entity *entity,
					u32 credits, void *owner, u64 client_id)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
	return drm_sched_job_init(job, entity, credits, owner, client_id);
#elif defined(LG_DRM_SCHED_JOB_INIT_HAS_CREDITS)
	return drm_sched_job_init(job, entity, credits, owner);
#else
	return drm_sched_job_init(job, entity, owner);
#endif
}

static inline void lg_ttm_placement_set_busy_place(struct ttm_placement *placement,
                                                   const struct ttm_place *busy_place,
                                                   int num_busy_place)
{
#if defined(LG_TTM_PLACEMENT_HAS_BUSY_PLACEMENT)
	placement->num_busy_placement = num_busy_place;
	placement->busy_placement = busy_place;
#endif
}

static inline void lg_ttm_placement_set_num_busy(struct ttm_placement *placement,
                                                   int num_busy_place)
{
#if defined(LG_TTM_PLACEMENT_HAS_BUSY_PLACEMENT)
	placement->num_busy_placement = num_busy_place;
#endif
}

#if defined(LG_DRM_MODE_CFG_FUNC_HAS_OUTPUT_POLL_CHANGED)
#define lg_drm_mode_cfg_func_set_output_poll_changed \
        .output_poll_changed = drm_fb_helper_output_poll_changed,
#else
#define lg_drm_mode_cfg_func_set_output_poll_changed
#endif

#if defined(LG_DRM_DRIVER_HAS_LASTCLOSE)
#define lg_drm_driver_set_lastclose .lastclose = loonggpu_driver_lastclose_kms,
#else
#define lg_drm_driver_set_lastclose
#endif

#if defined(LG_DRM_DRIVER_TIMER_CONTAINER_OF)
#define lg_drm_timer_container_of timer_container_of
#else
#define lg_drm_timer_container_of from_timer
#endif

#if defined(LG_DRM_DRIVER_TIMER_DELETE_SYNC)
#define lg_drm_timer_delete_sync timer_delete_sync
#define lg_drm_timer_delete timer_delete
#else
#define lg_drm_timer_delete_sync del_timer_sync
#define lg_drm_timer_delete del_timer
#endif

#if defined(VERIFY_WRITE)
#define lg_access_ok(type, addr, size) access_ok(type, addr, size)
#else
#define lg_access_ok(type, addr, size) access_ok(addr, size)
#endif

#if defined(LG_SYSFS_EMIT)
#define lg_sysfs_emit sysfs_emit
#else
#define lg_sysfs_emit sprintf
#endif

static inline int lg_ttm_bo_wait(struct ttm_buffer_object *tbo,
				bool interruptible, bool no_wait)
{
#if defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_wait(tbo, interruptible, no_wait);
#else
	struct ttm_operation_ctx ctx2 = { interruptible, no_wait };
	return ttm_bo_wait_ctx(tbo, &ctx2);
#endif
}

static inline void lg_use_mm(struct mm_struct *mm)
{
#if defined(LG_USE_MM)
	use_mm(mm);
#else
	kthread_use_mm(mm);
#endif
}

static inline void lg_unuse_mm(struct mm_struct *mm)
{
#if defined(LG_USE_MM)
	unuse_mm(mm);
#else
	kthread_unuse_mm(mm);
#endif
}

#endif /* __LOONGGPU_HELPER_H__ */
