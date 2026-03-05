#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_vbios.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_dp.h"
#include "bridge_phy.h"

static void ls9a1000_hpd_init(struct loonggpu_bridge_phy *phy, struct loonggpu_connector* lconnector, bool has_ext_encoder)
{
	struct loonggpu_device *adev = phy->adev;
	struct connector_resource *connector_res = adev->dc->link_info[lconnector->connector_id].connector;

	switch (connector_res->hotplug) {
	case IRQ:
		lconnector->base.polled = DRM_CONNECTOR_POLL_HPD;
		break;
	case POLLING:
	default:
		lconnector->base.polled = DRM_CONNECTOR_POLL_CONNECT |
					  DRM_CONNECTOR_POLL_DISCONNECT;
		break;
	case FORCE_ON:
		lconnector->base.polled = 0;
		break;
	}
}

static enum drm_connector_status ls9a1000_get_connect_status(struct loonggpu_bridge_phy *phy)
{
	struct drm_connector *connector = phy->connector;
	struct loonggpu_device *adev = phy->adev;
	struct loonggpu_dc_crtc *crtc = adev->dc->link_info[phy->connector->index].crtc;
	enum drm_connector_status status = connector_status_disconnected;
	struct loonggpu_dc_video *dc_video = &crtc->dc_video;
	int i;
	int reg_val;

	if (connector->polled == 0) {
		status = connector_status_connected;
		for (i = 0; i < crtc->interfaces; i++)
			crtc->intf[i].connected = true;
	} else {
		for (i = 0; i < crtc->interfaces; i++) {
			crtc->intf[i].connected = false;
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				reg_val = dc_readl(adev, gdc_reg->hdmi_reg_v2[dc_video->hdmi_num].irq_hpd);
				if (reg_val & 0x1) {
					status = connector_status_connected;
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_DP:
				reg_val = dc_readl(adev, gdc_reg->dp_reg[dc_video->dp_num].dp_hpd);
				if (reg_val & 0x1) {
					status = connector_status_connected;
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_VGA:
				if (dc_video->vga_connected) {
					status = connector_status_connected;
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_DVO:
				reg_val = dc_readl(adev, gdc_reg->dvo_reg.dvo_int_status);
				if (reg_val & 0x2) {
					status = connector_status_connected;
					crtc->intf[i].connected = true;
				}
				break;
			}
		}
	}

	return status;
}

static struct bridge_phy_hpd_funcs ls9a1000_hpd_funcs = {
	.hpd_init = ls9a1000_hpd_init,
	.get_connect_status = ls9a1000_get_connect_status,
};

static int ls9a1000_get_modes(struct loonggpu_bridge_phy *phy,
			      struct drm_connector *connector)
{
	unsigned int count = 0;

	count = drm_add_modes_noedid(connector, 640, 480);
	drm_set_preferred_mode(connector, 640, 480);

	return count;
}

static struct bridge_phy_ddc_funcs ls9a1000_ddc_funcs = {
	.get_modes = ls9a1000_get_modes,
};

static enum drm_mode_status ls9a1000_mode_valid(struct drm_connector *connector,
					      struct drm_display_mode *mode)
{
	if (mode->hdisplay > 640 || mode->vdisplay > 480)
		return MODE_BAD;

	if (mode->vdisplay < 480)
		return MODE_BAD;

	return MODE_OK;
}

static const struct bridge_phy_cfg_funcs ls9a1000_cfg_funcs = {
	.mode_valid = ls9a1000_mode_valid,
};

int internal_bridge_ls9a1000_register(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_bridge_phy *ls9a1000_phy;
	int index = dc_bridge->display_pipe_index;
	struct connector_resource *connector_res =
			dc_bridge->adev->dc->link_info[index].connector;

	ls9a1000_phy = kzalloc(sizeof(*ls9a1000_phy), GFP_KERNEL);
	if (IS_ERR(ls9a1000_phy)) {
		DRM_ERROR("Failed to alloc loonggpu bridge phy!\n");
		return -1;
	}

	ls9a1000_phy->display_pipe_index = index;
	ls9a1000_phy->bridge.driver_private = ls9a1000_phy;
	ls9a1000_phy->adev = dc_bridge->adev;
	ls9a1000_phy->res = dc_bridge;
	ls9a1000_phy->li2c = dc_bridge->adev->i2c[index];
	ls9a1000_phy->connector_type = connector_res->type;

	ls9a1000_phy->cfg_funcs = &ls9a1000_cfg_funcs;
	ls9a1000_phy->hpd_funcs = &ls9a1000_hpd_funcs;
	ls9a1000_phy->ddc_funcs = &ls9a1000_ddc_funcs;

	dc_bridge->internal_bp = ls9a1000_phy;
	DRM_DEBUG_DRIVER("internal bridge phy register success!\n");

	return 0;
}

int bridge_phy_ls9a1000_init(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_bridge_phy *ls9a1000_phy;

	ls9a1000_phy = bridge_phy_alloc(dc_bridge);

	return bridge_phy_init(ls9a1000_phy);
}
