#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/vga_switcheroo.h>

#include "loonggpu.h"
#include <drm/drm_fourcc.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include "loonggpu_drm.h"

#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "loonggpu_display.h"
#include "loonggpu_helper.h"

/* object hierarchy -
   this contains a helper + a loonggpu fb
   the helper contains a pointer to loonggpu framebuffer baseclass.
*/

static int
loonggpufb_open(struct fb_info *info, int user)
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
loonggpufb_release(struct fb_info *info, int user)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_device *ddev = fb_helper->dev;

	pm_runtime_mark_last_busy(ddev->dev);
	pm_runtime_put_autosuspend(ddev->dev);
	return 0;
}

static void loonggpufb_destroy_pinned_object(struct drm_gem_object *gobj)
{
	struct loonggpu_bo *abo = gem_to_loonggpu_bo(gobj);
	int ret;

	ret = loonggpu_bo_reserve(abo, true);
	if (likely(ret == 0)) {
		loonggpu_bo_kunmap(abo);
		loonggpu_bo_unpin(abo);
		loonggpu_bo_unreserve(abo);
	}
	lg_drm_gem_object_put(gobj);
}

static void loonggpu_fbdev_fb_destroy(struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_framebuffer *fb = fb_helper->fb;
	struct drm_gem_object *gobj = drm_gem_fb_get_obj(fb, 0);

	drm_fb_helper_fini(fb_helper);

	drm_framebuffer_unregister_private(fb);
	drm_framebuffer_cleanup(fb);
	kfree(fb);
	fb_helper->fb = NULL;

	loonggpufb_destroy_pinned_object(gobj);
	drm_client_release(&fb_helper->client);
	drm_fb_helper_unprepare(fb_helper);
	kfree(fb_helper);
}

static struct fb_ops loonggpufb_ops = {
	.owner = THIS_MODULE,
	DRM_FB_HELPER_DEFAULT_OPS,
	.fb_open = loonggpufb_open,
	.fb_release = loonggpufb_release,
	.fb_destroy = loonggpu_fbdev_fb_destroy,
#if defined(__FB_DEFAULT_IOMEM_OPS_DRAW)
	__FB_DEFAULT_IOMEM_OPS_DRAW
#else
	.fb_fillrect = drm_fb_helper_cfb_fillrect,
	.fb_copyarea = drm_fb_helper_cfb_copyarea,
	.fb_imageblit = drm_fb_helper_cfb_imageblit,
#endif
};


int loonggpu_align_pitch(struct loonggpu_device *adev, int width, int cpp, bool tiled)
{
	int aligned = width;

	aligned *= cpp;
	aligned += 255;
	aligned &= ~255;

	return aligned;
}

static int loonggpufb_create_pinned_object(struct drm_fb_helper *fb_helper,
					 struct drm_mode_fb_cmd2 *mode_cmd,
					 struct drm_gem_object **gobj_p)
{
	struct loonggpu_device *adev = fb_helper->dev->dev_private;
	struct drm_gem_object *gobj = NULL;
	const struct drm_format_info *info;
	struct loonggpu_bo *abo = NULL;
	bool fb_tiled = false; /* useful for testing */
	u32 tiling_flags = 0, domain;
	int ret;
	int aligned_size, size;
	int height = mode_cmd->height;
	u32 cpp;

	info = drm_get_format_info(adev->ddev, mode_cmd);
	cpp = info->cpp[0];

	/* need to align pitch with crtc limits */
	mode_cmd->pitches[0] = loonggpu_align_pitch(adev, mode_cmd->width, cpp,
						  fb_tiled);
	domain = loonggpu_display_supported_domains(adev);

	height = ALIGN(mode_cmd->height, 8);
	size = mode_cmd->pitches[0] * height;
	aligned_size = ALIGN(size, PAGE_SIZE);

	ret = loonggpu_gem_object_create(adev, aligned_size, 0, domain,
				       LOONGGPU_GEM_CREATE_CPU_ACCESS_REQUIRED |
				       LOONGGPU_GEM_CREATE_VRAM_CONTIGUOUS |
				       LOONGGPU_GEM_CREATE_VRAM_CLEARED,
				       ttm_bo_type_device, NULL, &gobj);

	if (ret) {
		pr_err("failed to allocate framebuffer (%d)\n", aligned_size);
		return -ENOMEM;
	}
	abo = gem_to_loonggpu_bo(gobj);

	ret = loonggpu_bo_reserve(abo, false);
	if (unlikely(ret != 0))
		goto out_unref;

	if (tiling_flags) {
		ret = loonggpu_bo_set_tiling_flags(abo,
						 tiling_flags);
		if (ret)
			dev_err(adev->dev, "FB failed to set tiling flags\n");
	}

	ret = loonggpu_bo_pin(abo, domain);
	if (ret) {
		loonggpu_bo_unreserve(abo);
		goto out_unref;
	}

	ret = loonggpu_ttm_alloc_gart(&abo->tbo);
	if (ret) {
		loonggpu_bo_unreserve(abo);
		dev_err(adev->dev, "%p bind failed\n", abo);
		goto out_unref;
	}

	ret = loonggpu_bo_kmap(abo, NULL);
	loonggpu_bo_unreserve(abo);
	if (ret) {
		goto out_unref;
	}

	*gobj_p = gobj;
	return 0;
out_unref:
	loonggpufb_destroy_pinned_object(gobj);
	*gobj_p = NULL;
	return ret;
}

static int loonggpufb_create(struct drm_fb_helper *helper,
			   struct drm_fb_helper_surface_size *sizes)
{
	struct loonggpu_device *adev = helper->dev->dev_private;
	struct drm_mode_fb_cmd2 mode_cmd = { };
	struct fb_info *info;
	struct drm_gem_object *gobj;
	struct loonggpu_bo *abo;
	struct drm_framebuffer *fb;
	unsigned long tmp;
	int ret;

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	if (sizes->surface_bpp == 24)
		sizes->surface_bpp = 32;

	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
							  sizes->surface_depth);

	ret = loonggpufb_create_pinned_object(helper, &mode_cmd, &gobj);
	if (ret) {
		DRM_ERROR("failed to create fbcon object %d\n", ret);
		return ret;
	}

	abo = gem_to_loonggpu_bo(gobj);

	fb = kzalloc(sizeof(struct loonggpu_framebuffer), GFP_KERNEL);
	if (!fb) {
		ret = -ENOMEM;
		goto err_loonggpufb_destroy_pinned_object;
	}

	ret = loonggpu_display_framebuffer_init(adev->ddev, to_loonggpu_framebuffer(fb),
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

	info->fbops = &loonggpufb_ops;
	info->skip_vt_switch = false;

	lg_drm_fb_helper_fill_info(info, helper, sizes);

	/* not alloc from VRAM but GTT */
	tmp = loonggpu_bo_gpu_offset(abo) - adev->gmc.vram_start;
	info->fix.smem_start = adev->gmc.aper_base + tmp;
	info->fix.smem_len = loonggpu_bo_size(abo);
	info->screen_base = loonggpu_bo_kptr(abo);
	info->screen_size = loonggpu_bo_size(abo);

	/* setup aperture base/size for vesafb takeover */
	lg_set_info_apertures(info, adev);

	/* Use default scratch pixmap (info->pixmap.flags = FB_PIXMAP_SYSTEM) */

	DRM_INFO("fb mappable at 0x%lX\n",  info->fix.smem_start);
	DRM_INFO("vram apper at 0x%lX\n",  (unsigned long)adev->gmc.aper_base);
	DRM_INFO("size %lu\n", (unsigned long)loonggpu_bo_size(abo));
	DRM_INFO("fb depth is %d\n", fb->format->depth);
	DRM_INFO("   pitch is %d\n", fb->pitches[0]);
	DRM_INFO("screen_base 0x%llX\n", (u64)info->screen_base);
	DRM_INFO("screen_size 0x%lX\n", info->screen_size);

	return 0;

err_drm_framebuffer_unregister_private:
	helper->fb = NULL;
	drm_framebuffer_unregister_private(fb);
	drm_framebuffer_cleanup(fb);
err_kfree:
	kfree(fb);
err_loonggpufb_destroy_pinned_object:
	loonggpufb_destroy_pinned_object(gobj);
	return ret;
}

static const struct drm_fb_helper_funcs loonggpu_fb_helper_funcs = {
	.fb_probe = loonggpufb_create,
};

static void loonggpu_fbdev_client_unregister(struct drm_client_dev *client)
{
	struct drm_fb_helper *fb_helper = drm_fb_helper_from_client(client);
	struct drm_device *dev = fb_helper->dev;
	struct loonggpu_device *rdev = dev->dev_private;

	if (fb_helper->info) {
		vga_switcheroo_client_fb_set(rdev->pdev, NULL);
		drm_fb_helper_unregister_info(fb_helper);
	} else {
		drm_client_release(&fb_helper->client);
		drm_fb_helper_unprepare(fb_helper);
		kfree(fb_helper);
	}
}

static int loonggpu_fbdev_client_restore(struct drm_client_dev *client)
{
	drm_fb_helper_lastclose(client->dev);
	vga_switcheroo_process_delayed_switch();

	return 0;
}

static int loonggpu_fbdev_client_hotplug(struct drm_client_dev *client) {
	struct drm_fb_helper *fb_helper = drm_fb_helper_from_client(client);
	struct drm_device *dev = client->dev;
	struct loonggpu_device *rdev = dev->dev_private;
	int ret;

	if (dev->fb_helper)
		return drm_fb_helper_hotplug_event(dev->fb_helper);

	ret = drm_fb_helper_init(dev, fb_helper);
	if (ret)
		goto err_drm_err;

	if (!drm_drv_uses_atomic_modeset(dev))
		drm_helper_disable_unused_functions(dev);

	ret = drm_fb_helper_initial_config(fb_helper);
	if (ret)
		goto err_drm_fb_helper_fini;

	vga_switcheroo_client_fb_set(rdev->pdev, fb_helper->info);

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(fb_helper);
err_drm_err:
	drm_err(dev, "Failed to setup loonggpu fbdev emulation (ret = %d)\n",
		ret);
	return ret;
}

static const struct drm_client_funcs loonggpu_fbdev_client_funcs = {
	.owner = THIS_MODULE,
	.unregister = loonggpu_fbdev_client_unregister,
	.restore = loonggpu_fbdev_client_restore,
	.hotplug = loonggpu_fbdev_client_hotplug,
};

void loonggpu_fbdev_setup(struct loonggpu_device *adev)
{
	struct drm_fb_helper *fb_helper;
	int bpp_sel = 32;
	int ret;

	fb_helper = kzalloc(sizeof(*fb_helper), GFP_KERNEL);
	if (!fb_helper)
		return;

	lg_drm_fb_helper_prepare(adev->ddev, fb_helper,
				bpp_sel, &loonggpu_fb_helper_funcs);

	ret = drm_client_init(adev->ddev, &fb_helper->client, "loonggpu-fbdev",
			      &loonggpu_fbdev_client_funcs);
	if (ret) {
		drm_err(adev->ddev, "Failed to register client: %d\n", ret);
		goto err_drm_client_init;
	}

	drm_client_register(&fb_helper->client);

	return;

err_drm_client_init:
	drm_fb_helper_unprepare(fb_helper);
	kfree(fb_helper);
}

void loonggpu_fbdev_set_suspend(struct loonggpu_device *adev, int state)
{
	if (adev->ddev->fb_helper)
		drm_fb_helper_set_suspend_unlocked(adev->ddev->fb_helper,
					        state);
}

bool loonggpu_fbdev_robj_is_fb(struct loonggpu_device *adev, struct loonggpu_bo *robj)
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
