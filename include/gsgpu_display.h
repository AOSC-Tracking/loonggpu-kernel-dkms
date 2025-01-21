#ifndef __GSGPU_DISPLAY_H__
#define __GSGPU_DISPLAY_H__

uint32_t gsgpu_display_supported_domains(struct gsgpu_device *adev);
struct drm_framebuffer *
gsgpu_display_user_framebuffer_create(struct drm_device *dev,
				       struct drm_file *file_priv,
				       const struct drm_mode_fb_cmd2 *mode_cmd);
int gsgpu_display_crtc_page_flip_target(struct drm_crtc *crtc,
                                struct drm_framebuffer *fb,
                                struct drm_pending_vblank_event *event,
                                uint32_t page_flip_flags, uint32_t target,
                                struct drm_modeset_acquire_ctx *ctx);

#endif /* __GSGPU_DISPLAY_H__ */
