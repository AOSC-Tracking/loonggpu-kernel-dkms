#include <linux/delay.h>
#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_resource.h"
#include "loonggpu_dc_interface.h"
#include "loonggpu_dc_crtc.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_helper.h"

int dc_interface_noaudio_init(struct loonggpu_dc_crtc *crtc)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		if (crtc->intf[i].connected) {
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				dc->hw_ops->hdmi_noaudio_init(crtc, crtc->intf[i].index);
				break;
			case INTERFACE_EDP:
				dc->hw_ops->dp_noaudio_init(crtc, crtc->intf[i].index);
				break;
			}
		}
	}
	return 0;
}

int dc_interface_audio_init(struct loonggpu_dc_crtc *crtc)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		if (crtc->intf[i].connected) {
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				dc->hw_ops->hdmi_audio_init(crtc, crtc->intf[i].index);
				break;
			case INTERFACE_EDP:
				dc->hw_ops->dp_audio_init(crtc, crtc->intf[i].index);
				break;
			}
		}
	}
	return 0;
}

int dc_interface_init(struct loonggpu_dc_crtc *crtc)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		switch (crtc->intf[i].type) {
		case INTERFACE_HDMI:
			dc->hw_ops->hdmi_init(crtc, crtc->intf[i].index);
			break;
		case INTERFACE_DP:
		case INTERFACE_EDP:
			dc->hw_ops->dp_init(crtc, crtc->intf[i].index);
			break;
		}
	}
	return 0;
}

void dc_interface_suspend(struct loonggpu_dc_crtc *crtc)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		if (crtc->intf[i].connected) {
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				dc->hw_ops->hdmi_suspend(crtc, crtc->intf[i].index);
				break;
			case INTERFACE_DP:
			case INTERFACE_EDP:
				dc->hw_ops->dp_suspend(crtc, crtc->intf[i].index);
				break;
			}
		}
	}
}

int dc_interface_resume(struct loonggpu_dc_crtc *crtc)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		switch (crtc->intf[i].type) {
		case INTERFACE_HDMI:
			dc->hw_ops->hdmi_resume(crtc, crtc->intf[i].index);
			break;
		case INTERFACE_DP:
		case INTERFACE_EDP:
			dc->hw_ops->dp_resume(crtc, crtc->intf[i].index);
			break;
		}
	}
	return 0;
}

bool dc_interface_enable(struct loonggpu_dc_crtc *crtc, bool enable)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		switch (crtc->intf[i].type) {
		case INTERFACE_HDMI:
			dc->hw_ops->hdmi_enable(crtc, crtc->intf[i].index, enable);
			break;
		case INTERFACE_DP:
		case INTERFACE_EDP:
			dc->hw_ops->dp_enable(crtc, crtc->intf[i].index, enable);
			break;
		}
	}
	return 0;
}

void dc_interface_pll_set(struct loonggpu_dc_crtc *crtc, struct dc_timing_info *timing)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		if (crtc->intf[i].connected) {
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				dc->hw_ops->hdmi_pll_set(crtc, crtc->intf[i].index, timing->clock);
				break;
			case INTERFACE_DP:
			case INTERFACE_EDP:
				dc->hw_ops->dp_pll_set(crtc, crtc->intf[i].index, timing);
				break;
			}
		}
	}
}

void dc_interface_i2c_set(struct loonggpu_dc_crtc *crtc, bool use_gpio_i2c)
{
	struct loonggpu_dc *dc = crtc->dc;
	int i;

	for (i = 0; i < crtc->interfaces; i++) {
		switch (crtc->intf[i].type) {
		case INTERFACE_HDMI:
		case INTERFACE_DVO:
			dc->hw_ops->hdmi_i2c_set(crtc, crtc->resource->base.link, use_gpio_i2c);
			break;
		}
	}
}
