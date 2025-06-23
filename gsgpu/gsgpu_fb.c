#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/vga_switcheroo.h>

#include "gsgpu.h"
#include <drm/drm_fourcc.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include "gsgpu_drm.h"

#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "gsgpu_display.h"
#include "gsgpu_helper.h"

/* object hierarchy -
   this contains a helper + a gsgpu fb
   the helper contains a pointer to gsgpu framebuffer baseclass.
*/

static int
gsgpufb_open(struct fb_info *info, int user)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *ddev = fb_helper->dev;
	int ret;

	ret = pm_runtime_get_sync(ddev->dev);
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_mark_last_busy(ddev->dev);
		pm_runtime_put_autosuspend(ddev->dev);
		return ret;
	}
	return 0;
}

static int
gsgpufb_release(struct fb_info *info, int user)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *ddev = fb_helper->dev;

	pm_runtime_mark_last_busy(ddev->dev);
	pm_runtime_put_autosuspend(ddev->dev);
	return 0;
}

static void gsgpufb_destroy_pinned_object(struct drm_gem_object *gobj)
{
	struct gsgpu_bo *abo = gem_to_gsgpu_bo(gobj);
	int ret;

	ret = gsgpu_bo_reserve(abo, true);
	if (likely(ret == 0)) {
		gsgpu_bo_kunmap(abo);
		gsgpu_bo_unpin(abo);
		gsgpu_bo_unreserve(abo);
	}
	lg_drm_gem_object_put(gobj);
}

static void gsgpu_fbdev_fb_destroy(struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_framebuffer *fb = fb_helper->fb;
	struct drm_gem_object *gobj = drm_gem_fb_get_obj(fb, 0);

	drm_framebuffer_unregister_private(fb);
	drm_framebuffer_cleanup(fb);
	kfree(fb);
	fb_helper->fb = NULL;

	gsgpufb_destroy_pinned_object(gobj);

	drm_fb_helper_fini(fb_helper);
}

static struct fb_ops gsgpufb_ops = {
	.owner = THIS_MODULE,
	DRM_FB_HELPER_DEFAULT_OPS,
	.fb_open = gsgpufb_open,
	.fb_release = gsgpufb_release,
	.fb_destroy = gsgpu_fbdev_fb_destroy,
#if defined(__FB_DEFAULT_IOMEM_OPS_DRAW)
	__FB_DEFAULT_IOMEM_OPS_DRAW
#else
	.fb_fillrect = drm_fb_helper_cfb_fillrect,
	.fb_copyarea = drm_fb_helper_cfb_copyarea,
	.fb_imageblit = drm_fb_helper_cfb_imageblit,
#endif
};


int gsgpu_align_pitch(struct gsgpu_device *adev, int width, int cpp, bool tiled)
{
	int aligned = width;

	aligned *= cpp;
	aligned += 255;
	aligned &= ~255;

	return aligned;
}

static int gsgpufb_create_pinned_object(struct drm_fb_helper *fb_helper,
					struct drm_mode_fb_cmd2 *mode_cmd,
					struct drm_gem_object **gobj_p)
{
	struct gsgpu_device *adev = fb_helper->dev->dev_private;
	struct drm_gem_object *gobj = NULL;
	const struct drm_format_info *info;
	struct gsgpu_bo *abo = NULL;
	bool fb_tiled = false; /* useful for testing */
	u32 tiling_flags = 0, domain;
	int ret;
	int aligned_size, size;
	int height = mode_cmd->height;
	u32 cpp;

	info = drm_get_format_info(adev->ddev, mode_cmd);
	cpp = info->cpp[0];

	/* need to align pitch with crtc limits */
	mode_cmd->pitches[0] = gsgpu_align_pitch(adev, mode_cmd->width, cpp,
						  fb_tiled);
	domain = gsgpu_display_supported_domains(adev);

	height = ALIGN(mode_cmd->height, 8);
	size = mode_cmd->pitches[0] * height;
	aligned_size = ALIGN(size, PAGE_SIZE);

	ret = gsgpu_gem_object_create(adev, aligned_size, 0, domain,
				       GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED |
				       GSGPU_GEM_CREATE_VRAM_CONTIGUOUS |
				       GSGPU_GEM_CREATE_VRAM_CLEARED,
				       ttm_bo_type_device, NULL, &gobj);

	if (ret) {
		pr_err("failed to allocate framebuffer (%d)\n", aligned_size);
		return -ENOMEM;
	}
	abo = gem_to_gsgpu_bo(gobj);

	ret = gsgpu_bo_reserve(abo, false);
	if (unlikely(ret != 0))
		goto out_unref;

	if (tiling_flags) {
		ret = gsgpu_bo_set_tiling_flags(abo,
						 tiling_flags);
		if (ret)
			dev_err(adev->dev, "FB failed to set tiling flags\n");
	}

	ret = gsgpu_bo_pin(abo, domain);
	if (ret) {
		gsgpu_bo_unreserve(abo);
		goto out_unref;
	}

	ret = gsgpu_ttm_alloc_gart(&abo->tbo);
	if (ret) {
		gsgpu_bo_unreserve(abo);
		dev_err(adev->dev, "%p bind failed\n", abo);
		goto out_unref;
	}

	ret = gsgpu_bo_kmap(abo, NULL);
	gsgpu_bo_unreserve(abo);
	if (ret) {
		goto out_unref;
	}

	*gobj_p = gobj;
	return 0;
out_unref:
	gsgpufb_destroy_pinned_object(gobj);
	*gobj_p = NULL;
	return ret;
}

static int gsgpufb_create(struct drm_fb_helper *helper,
			   struct drm_fb_helper_surface_size *sizes)
{
	struct gsgpu_device *adev = helper->dev->dev_private;
	struct drm_mode_fb_cmd2 mode_cmd = { };
	struct fb_info *info;
	struct drm_gem_object *gobj;
	struct gsgpu_bo *abo;
	struct drm_framebuffer *fb;
	unsigned long tmp;
	int ret;

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	if (sizes->surface_bpp == 24)
		sizes->surface_bpp = 32;

	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
							  sizes->surface_depth);

	ret = gsgpufb_create_pinned_object(helper, &mode_cmd, &gobj);
	if (ret) {
		DRM_ERROR("failed to create fbcon object %d\n", ret);
		return ret;
	}

	abo = gem_to_gsgpu_bo(gobj);

	fb = kzalloc(sizeof(struct gsgpu_framebuffer), GFP_KERNEL);
	if (!fb) {
		ret = -ENOMEM;
		goto err_gsgpufb_destroy_pinned_object;
	}

	ret = gsgpu_display_framebuffer_init(adev->ddev, to_gsgpu_framebuffer(fb),
					      &mode_cmd, gobj);
	if (ret) {
		DRM_ERROR("failed to initialize framebuffer %d\n", ret);
		goto err_kfree;
	}

	/* setup helper */
	helper->fb = fb;

	/* okay we have an object now allocate the framebuffer */
	info = lg_drm_fb_helper_alloc_info(helper);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		goto err_drm_framebuffer_unregister_private;
	}

	info->fbops = &gsgpufb_ops;
	info->skip_vt_switch = false;

	lg_drm_fb_helper_fill_info(info, helper, sizes);

	/* not alloc from VRAM but GTT */
	tmp = gsgpu_bo_gpu_offset(abo) - adev->gmc.vram_start;
	info->fix.smem_start = adev->gmc.aper_base + tmp;
	info->fix.smem_len = gsgpu_bo_size(abo);
	info->screen_base = gsgpu_bo_kptr(abo);
	info->screen_size = gsgpu_bo_size(abo);

	/* setup aperture base/size for vesafb takeover */
	lg_set_info_apertures(info, adev);

	/* Use default scratch pixmap (info->pixmap.flags = FB_PIXMAP_SYSTEM) */

	DRM_INFO("fb mappable at 0x%lX\n",  info->fix.smem_start);
	DRM_INFO("vram apper at 0x%lX\n",  (unsigned long)adev->gmc.aper_base);
	DRM_INFO("size %lu\n", (unsigned long)gsgpu_bo_size(abo));
	DRM_INFO("fb depth is %d\n", fb->format->depth);
	DRM_INFO("   pitch is %d\n", fb->pitches[0]);
	DRM_INFO("screen_base 0x%llX\n", (u64)info->screen_base);
	DRM_INFO("screen_size 0x%lX\n", info->screen_size);

	vga_switcheroo_client_fb_set(adev->pdev, info);
	return 0;

err_drm_framebuffer_unregister_private:
	helper->fb = NULL;
	drm_framebuffer_unregister_private(fb);
	drm_framebuffer_cleanup(fb);
err_kfree:
	kfree(fb);
err_gsgpufb_destroy_pinned_object:
	gsgpufb_destroy_pinned_object(gobj);
	return ret;
}

static const struct drm_fb_helper_funcs gsgpu_fb_helper_funcs = {
	.fb_probe = gsgpufb_create,
};

int gsgpu_fbdev_init(struct gsgpu_device *adev)
{
	struct drm_fb_helper *fb_helper;
	int bpp_sel = 32;
	int ret;

	/* don't init fbdev on hw without DCE */
	if (!adev->mode_info.mode_config_initialized)
		return 0;

	/* don't init fbdev if there are no connectors */
	if (list_empty(&adev->ddev->mode_config.connector_list))
		return 0;

	/* select 8 bpp console on low vram cards */
	if (adev->gmc.real_vram_size <= (32*1024*1024))
		bpp_sel = 8;

	fb_helper = kzalloc(sizeof(*fb_helper), GFP_KERNEL);
	if (!fb_helper)
		return -ENOMEM;

	lg_drm_fb_helper_prepare(adev->ddev, fb_helper,
				bpp_sel, &gsgpu_fb_helper_funcs);

	ret = lg_drm_fb_helper_init(adev->ddev, fb_helper,
				 GSGPUFB_CONN_LIMIT);
	if (ret)
		goto free;

	lg_drm_fb_helper_single_add_all_connectors(fb_helper);
	ret = drm_fb_helper_initial_config(fb_helper);
	if (ret)
		goto fini;

	return 0;

fini:
	drm_fb_helper_fini(fb_helper);
free:
	drm_fb_helper_unprepare(fb_helper);
	kfree(fb_helper);
	return ret;
}

void gsgpu_fbdev_fini(struct gsgpu_device *adev)
{
	if (!adev->ddev->fb_helper)
		return;

	drm_fb_helper_unregister_info(adev->ddev->fb_helper);
	drm_fb_helper_unprepare(adev->ddev->fb_helper);
	kfree(adev->ddev->fb_helper);
	adev->ddev->fb_helper = NULL;
}

void gsgpu_fbdev_set_suspend(struct gsgpu_device *adev, int state)
{
	if (adev->ddev->fb_helper)
		drm_fb_helper_set_suspend_unlocked(adev->ddev->fb_helper,
					        state);
}

bool gsgpu_fbdev_robj_is_fb(struct gsgpu_device *adev, struct gsgpu_bo *robj)
{
	struct drm_fb_helper *fb_helper = adev->ddev->fb_helper;
	struct drm_gem_object *gobj;

	if (!fb_helper)
		return false;

	gobj = drm_gem_fb_get_obj(fb_helper->fb, 0);
	if (!gobj)
		return false;
	if (gobj != &robj->tbo.base)
		return false;

	return true;
}
