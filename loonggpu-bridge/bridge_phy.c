#include <drm/drm_atomic_helper.h>
#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_encoder.h"
#include "loonggpu_dc_resource.h"
#include "loonggpu_dc_vbios.h"
#include "loonggpu_dc_reg.h"
#include "bridge_phy.h"
#include "loonggpu_backlight.h"
#include "loonggpu_helper.h"

/**
 * @section Bridge-phy connector functions
 */
static int check_hdmi_audio(struct loonggpu_device *adev, struct drm_connector *connector, struct edid *edid)
{
	u8 *ext_edid = NULL;
	struct loonggpu_dc_crtc *dc_crtc;
	bool audio_support;

	dc_crtc = adev->dc->link_info[connector->index].crtc;
	if (!edid->extensions) {
		DRM_DEBUG("This monitor does not support audio. \n");
		return -EINVAL;
	}

	/* This function must be called under i2c_method condition, where some notebook use eDP transferred
	 * from hdmi interface to connect display. So we count on it here. */
	if ((connector->connector_type != DRM_MODE_CONNECTOR_HDMIA) && (connector->connector_type != DRM_MODE_CONNECTOR_eDP)) {
		DRM_DEBUG("This monitor does not support audio. \n");
		return -EINVAL;
	}

	ext_edid = (u8 *)connector->edid_blob_ptr->data;
	audio_support = ext_edid[131] & HDMI_EDID_AUDIO_STATUS ? true : false;

	if (ext_edid && audio_support) {
		dc_interface_audio_init(dc_crtc);
		DRM_DEBUG("This monitor supports audio and turns on hdmi audio. \n");
	} else {
		dc_interface_noaudio_init(dc_crtc);
		DRM_DEBUG("This monitor does not support audio and turns off hdmi audio. \n");
	}

	return 0;
}

static int bridge_phy_connector_get_modes(struct drm_connector *connector)
{
	struct edid *edid = NULL;
	unsigned int count = 0;
	struct loonggpu_device *adev = connector->dev->dev_private;
	struct loonggpu_bridge_phy *phy =
		adev->mode_info.encoders[connector->index]->bridge;
	struct loonggpu_bridge_phy *internal_bp = phy->res->internal_bp;
	unsigned short used_method = phy->res->edid_method;
	struct connector_resource *connector_resource;
	int size = sizeof(u8) * EDID_LENGTH * 2;

	switch (used_method) {
	case via_i2c:
		if (phy->li2c)
			edid = drm_get_edid(connector, &phy->li2c->adapter);

		if (edid) {
			drm_connector_update_edid_property(connector, edid);
			check_hdmi_audio(adev, connector, edid);
			count = drm_add_edid_modes(connector, edid);
			kfree(edid);
		}
		break;
	case via_encoder:
		if (phy->ddc_funcs && phy->ddc_funcs->get_modes)
			count = phy->ddc_funcs->get_modes(phy, connector);
		else if (internal_bp && internal_bp->ddc_funcs && internal_bp->ddc_funcs->get_modes)
			count = internal_bp->ddc_funcs->get_modes(internal_bp, connector);
		break;
	case via_vbios:
		connector_resource = dc_get_vbios_resource(internal_bp->adev->dc->vbios, connector->index, LOONGGPU_RESOURCE_CONNECTOR);

		edid = kmalloc(size, GFP_KERNEL);
		if (edid && connector_resource) {
			memcpy(edid, connector_resource->internal_edid, size);
			drm_connector_update_edid_property(connector, edid);
			count = drm_add_edid_modes(connector, edid);
			kfree(edid);
		}
	default:
		break;
	}

	if (!count) {
		count = drm_add_modes_noedid(connector, 1920, 1080);
		drm_set_preferred_mode(connector, 1024, 768);
		DRM_DEBUG_DRIVER("[Bridge_phy] Setting %s edid.\n",
				 phy->res->chip_name);
	}

	return count;
}

static bool is_resolution_valid(struct panel_resource *panel_resource, const struct drm_display_mode *mode)
{
	u32 vrefresh = 0;
	u32 index;

	if (!panel_resource)
		return false;

	vrefresh = drm_mode_vrefresh(mode);

	if (mode->hdisplay > panel_resource->max_hdisplay &&    \
					mode->vdisplay > panel_resource->max_vdisplay)
			return true;
	else if (mode->hdisplay == panel_resource->max_hdisplay &&      \
					mode->vdisplay == panel_resource->max_vdisplay) {
			if (vrefresh > panel_resource->max_vrefresh)
					return true;
	}

	for (index = 0; index < panel_resource->count; index++) {
		if (panel_resource->timing[index].vrefresh == vrefresh &&   \
			panel_resource->timing[index].hdisplay == mode->hdisplay &&   \
			panel_resource->timing[index].vdisplay == mode->vdisplay)
			return true;
	}

	return false;
}

static enum drm_mode_status
bridge_phy_connector_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct loonggpu_device *adev = connector->dev->dev_private;
	struct loonggpu_bridge_phy *phy =
		adev->mode_info.encoders[connector->index]->bridge;
	struct loonggpu_bridge_phy *internal_bp = phy->res->internal_bp;
	struct panel_resource *panel_resource;
	struct loonggpu_dc *dc = adev->dc;

	panel_resource = dc_get_vbios_resource(dc->vbios,
				connector->index, LOONGGPU_RESOURCE_PANEL);

	if (is_resolution_valid(panel_resource, mode))
		return MODE_BAD;

	if (phy->cfg_funcs && phy->cfg_funcs->mode_valid)
		return phy->cfg_funcs->mode_valid(connector, mode);
	else
		return internal_bp->cfg_funcs->mode_valid(connector, mode);
	return MODE_OK;
}

static struct drm_encoder *
bridge_phy_connector_best_encoder(struct drm_connector *connector)
{
	struct drm_encoder *encoder;
	int i;

	/* pick the first one */
	lg_drm_connector_for_each_possible_encoder(connector, encoder, i)
		return encoder;

	return NULL;
}

static struct drm_connector_helper_funcs bridge_phy_connector_helper_funcs = {
	.get_modes = bridge_phy_connector_get_modes,
	.mode_valid = bridge_phy_connector_mode_valid,
	.best_encoder = bridge_phy_connector_best_encoder,
};

bool is_connected(struct drm_connector *connector)
{
	struct loonggpu_device *adev = connector->dev->dev_private;
	struct loonggpu_dc_i2c *i2c = adev->i2c[connector->index];
	unsigned char start = 0x0;
	struct i2c_adapter *adapter;
	struct i2c_msg msgs = {
		.addr = DDC_ADDR,
		.flags = I2C_M_RD,
		.len = 1,
		.buf = &start,
	};

	if (!i2c)
		return false;

	adapter = &i2c->adapter;
	if (i2c_transfer(adapter, &msgs, 1) != 1) {
		DRM_DEBUG_KMS("display-%d not connect\n", connector->index);
		return false;
	}

	return true;
}

static enum drm_connector_status
bridge_phy_connector_detect(struct drm_connector *connector, bool force)
{
	struct loonggpu_device *adev = connector->dev->dev_private;
	enum drm_connector_status status = connector_status_connected;
	struct loonggpu_bridge_phy *phy = adev->mode_info.encoders[connector->index]->bridge;
	struct loonggpu_bridge_phy *internal_bp = phy->res->internal_bp;

	if (internal_bp->hpd_funcs && internal_bp->hpd_funcs->get_connect_status)
		status = internal_bp->hpd_funcs->get_connect_status(phy);

	return status;
}

static void loonggpu_dc_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static int loonggpu_drm_helper_probe_single_connector_modes(
				struct drm_connector *connector,
				uint32_t maxX, uint32_t maxY)
{
	int count;
	struct drm_display_mode *mode;

	count = drm_helper_probe_single_connector_modes(connector, maxX, maxY);

	/*
	 * if there is no preferred mode in 'connector->modes',
	 * chose the first one as preferred mode.
	 */
	if (count > 0) {
		mode = list_first_entry(&connector->modes, struct drm_display_mode, head);
		if (mode && !(mode->type & DRM_MODE_TYPE_PREFERRED))
			mode->type |= DRM_MODE_TYPE_PREFERRED;
	}

	return count;
}

static const struct drm_connector_funcs bridge_phy_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = bridge_phy_connector_detect,
	.fill_modes = loonggpu_drm_helper_probe_single_connector_modes,
	.destroy = loonggpu_dc_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.late_register = loonggpu_backlight_register
};

/**
 * @section Bridge-phy core functions
 */
static void bridge_phy_enable(struct drm_bridge *bridge)
{
	struct loonggpu_bridge_phy *phy = to_bridge_phy(bridge);

	DRM_DEBUG("[Bridge_phy] [%s] enable\n", phy->res->chip_name);
	if (phy->cfg_funcs && phy->cfg_funcs->afe_high)
		phy->cfg_funcs->afe_high(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->afe_set_tx)
		phy->cfg_funcs->afe_set_tx(phy, TRUE);
	if (phy->cfg_funcs && phy->cfg_funcs->hdmi_audio)
		phy->cfg_funcs->hdmi_audio(phy);
}

static void bridge_phy_disable(struct drm_bridge *bridge)
{
	struct loonggpu_bridge_phy *phy = to_bridge_phy(bridge);

	DRM_DEBUG("[Bridge_phy] [%s] disable\n", phy->res->chip_name);
	if (phy->cfg_funcs && phy->cfg_funcs->afe_low)
		phy->cfg_funcs->afe_low(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->afe_set_tx)
		phy->cfg_funcs->afe_set_tx(phy, FALSE);
}

static int __bridge_phy_mode_set(struct loonggpu_bridge_phy *phy,
				 const struct drm_display_mode *mode,
				 const struct drm_display_mode *adj_mode)
{
	if (phy->mode_config.input_mode.gen_sync)
		DRM_DEBUG("[Bridge_phy] [%s] bridge_phy gen_sync\n",
				phy->res->chip_name);
	if (phy->cfg_funcs && phy->cfg_funcs->mode_set_pre)
		phy->cfg_funcs->mode_set_pre(&phy->bridge, mode, adj_mode);
	if (phy->cfg_funcs && phy->cfg_funcs->mode_set)
		phy->cfg_funcs->mode_set(phy, mode, adj_mode);
	if (phy->cfg_funcs && phy->cfg_funcs->mode_set_post)
		phy->cfg_funcs->mode_set_post(&phy->bridge, mode, adj_mode);

	return 0;
}

void bridge_phy_mode_set(struct loonggpu_bridge_phy *phy,
				struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	if (!phy)
		return;

	DRM_DEBUG("[Bridge_phy] [%s] mode set\n", phy->res->chip_name);
	drm_mode_debug_printmodeline(mode);

	__bridge_phy_mode_set(phy, mode, adj_mode);
}

static int bridge_phy_attach (lg_bridge_phy_attach_args)
{
	struct loonggpu_bridge_phy *phy = to_bridge_phy(bridge);
	struct loonggpu_connector *lconnector;
	int link_index = phy->display_pipe_index;
	struct loonggpu_dc_bridge *dc_bridge = phy->res;
	struct loonggpu_bridge_phy *internal_bp = dc_bridge->internal_bp;
	int ret;

	DRM_DEBUG("[Bridge_phy] %s attach\n", phy->res->chip_name);
	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found\n");
		return -ENODEV;
	}

	lconnector = kzalloc(sizeof(*lconnector), GFP_KERNEL);
	if (!lconnector)
		return -ENOMEM;

	ret = drm_connector_init(bridge->dev, &lconnector->base,
				 &bridge_phy_connector_funcs,
				 phy->connector_type);
	if (ret) {
		DRM_ERROR("[Bridge_phy] %s Failed to initialize connector\n",
				phy->res->chip_name);
		return ret;
	}

	lconnector->connector_id = link_index;
	phy->connector = &lconnector->base;
	phy->adev->mode_info.connectors[link_index] = lconnector;
	phy->connector->dpms = DRM_MODE_DPMS_OFF;

	drm_connector_helper_add(&lconnector->base,
				 &bridge_phy_connector_helper_funcs);

	if (phy->hpd_funcs && phy->hpd_funcs->hpd_init)
		phy->hpd_funcs->hpd_init(phy, lconnector, phy->is_ext_encoder);
	else if (internal_bp->hpd_funcs && internal_bp->hpd_funcs->hpd_init)
		internal_bp->hpd_funcs->hpd_init(phy, lconnector, phy->is_ext_encoder);

	DRM_DEBUG_DRIVER("[Bridge_phy] %s Set connector poll=0x%x.\n",
			 phy->res->chip_name, phy->connector->polled);

	return ret;
}

static const struct drm_bridge_funcs bridge_funcs = {
	.enable = bridge_phy_enable,
	.disable = bridge_phy_disable,
	.attach = bridge_phy_attach,
};

static int bridge_phy_bind(struct loonggpu_bridge_phy *phy)
{
	int ret;

	phy->bridge.funcs = &bridge_funcs;
	drm_bridge_add(&phy->bridge);
	ret = lg_drm_bridge_attach(phy->encoder, &phy->bridge, NULL, 0);
	if (ret) {
		DRM_ERROR("[Bridge_phy] %s Failed to attach phy ret %d\n",
			  phy->res->chip_name, ret);
		return ret;
	}

	DRM_INFO("[Bridge_phy] %s encoder-%d be attach to this bridge.\n",
		 phy->res->chip_name, phy->encoder->index);

	return 0;
}

/**
 * @section Bridge-phy helper functions
 */
void bridge_phy_reg_mask_seq(struct loonggpu_bridge_phy *phy,
			     const struct reg_mask_seq *seq, size_t seq_size)
{
	unsigned int i;
	struct regmap *regmap;

	regmap = phy->phy_regmap;
	for (i = 0; i < seq_size; i++)
		regmap_update_bits(regmap, seq[i].reg, seq[i].mask, seq[i].val);
}

void bridge_phy_reg_update_bits(struct loonggpu_bridge_phy *phy, unsigned int reg,
				unsigned int mask, unsigned int val)
{
	unsigned int reg_val;

	regmap_read(phy->phy_regmap, reg, &reg_val);
	val ? (reg_val |= (mask)) : (reg_val &= ~(mask));
	regmap_write(phy->phy_regmap, reg, reg_val);
}

int bridge_phy_reg_dump(struct loonggpu_bridge_phy *phy, size_t start,
			size_t count)
{
	u8 *buf;
	int ret;
	unsigned int i;

	buf = kzalloc(count, GFP_KERNEL);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		return -ENOMEM;
	}
	ret = regmap_raw_read(phy->phy_regmap, start, buf, count);
	for (i = 0; i < count; i++)
		pr_info("[%lx]=%02x", start + i, buf[i]);

	kfree(buf);
	return ret;
}

static char *get_encoder_chip_name(int encoder_obj)
{
	switch (encoder_obj) {
	case ENCODER_CHIP_ID_NONE:
		return "none";
	case ENCODER_CHIP_ID_EDP_NCS8803:
		return "ncs8803";
	case ENCODER_CHIP_ID_EDP_NCS8805:
		return "ncs8805";
	case ENCODER_CHIP_ID_EDP_LT9721:
		return "lt9721";
	case ENCODER_CHIP_ID_EDP_LT6711:
		return "lt6711";
	case ENCODER_CHIP_ID_LVDS_LT8619:
		return "lt8619";
	case ENCODER_CHIP_ID_DP_LT8718:
		return "lt8718";
	case ENCODER_CHIP_ID_VGA_TRANSPARENT:
	case ENCODER_CHIP_ID_INTERNAL_DVO:
		return "vga";
	case ENCODER_CHIP_ID_DVI_TRANSPARENT:
		return "dvi";
	case ENCODER_CHIP_ID_HDMI_TRANSPARENT:
	case ENCODER_CHIP_ID_INTERNAL_HDMI:
		return "hdmi";
	case ENCODER_CHIP_ID_EDP_TRANSPARENT:
	case ENCODER_CHIP_ID_INTERNAL_EDP:
		return "edp";
	case ENCODER_CHIP_ID_INTERNAL_DP:
		return "dp";
	default:
		DRM_WARN("No ext encoder chip 0x%x.\n", encoder_obj);
		return "Unknown";
	}
}

static int bridge_phy_register_irq_num(struct loonggpu_bridge_phy *phy)
{
	int ret = -1;
	char irq_name[NAME_SIZE_MAX];
	struct loonggpu_dc_bridge *res;

	res = phy->res;
	phy->irq_num = -1;

	if(!phy->adev->dc->link_info[phy->display_pipe_index].encoder->has_ext_encoder)
		return 0;

	if (phy->hpd_funcs || phy->hpd_funcs->isr_thread || phy->hpd_funcs->irq_handler)
		return ret;

	if (phy->res->hotplug != IRQ)
		return ret;

	if (res->gpio_placement)
		res->irq_gpio += LS7A_GPIO_OFFSET;

	ret = gpio_is_valid(res->irq_gpio);
	if (!ret)
		goto error_gpio_valid;
	sprintf(irq_name, "%s-irq", res->chip_name);

	ret = gpio_request(res->irq_gpio, irq_name);
	if (ret)
		goto error_gpio_req;
	ret = gpio_direction_input(res->irq_gpio);
	if (ret)
		goto error_gpio_cfg;

	phy->irq_num = gpio_to_irq(res->irq_gpio);
	if (phy->irq_num < 0) {
		ret = phy->irq_num;
		DRM_ERROR("GPIO %d has no interrupt\n", res->irq_gpio);
		return ret;
	}

	DRM_DEBUG("[Bridge_phy] %s register irq num %d.\n", res->chip_name,
		  phy->irq_num);
	return 0;

error_gpio_cfg:
	DRM_ERROR("Failed to config gpio %d free it %d\n", res->irq_gpio, ret);
	gpio_free(res->irq_gpio);
error_gpio_req:
	DRM_ERROR("Failed to request gpio %d, %d\n", res->irq_gpio, ret);
error_gpio_valid:
	DRM_ERROR("Invalid gpio %d, %d\n", res->irq_gpio, ret);
	return ret;
}

static int bridge_phy_register_irq_handle(struct loonggpu_bridge_phy *phy)
{
	int ret = -1;
	irqreturn_t (*irq_handler)(int irq, void *dev);
	irqreturn_t (*isr_thread)(int irq, void *dev);

	if(!phy->adev->dc->link_info[phy->display_pipe_index].encoder->has_ext_encoder)
		return 0;

	if (phy->hpd_funcs || phy->hpd_funcs->isr_thread || phy->hpd_funcs->irq_handler)
		return ret;

	if (phy->res->hotplug != IRQ)
		return ret;

	if (phy->irq_num <= 0) {
		phy->connector->polled = DRM_CONNECTOR_POLL_CONNECT |
					 DRM_CONNECTOR_POLL_DISCONNECT;
		return ret;
	}

	if (phy->hpd_funcs && phy->hpd_funcs->isr_thread) {
		irq_handler = phy->hpd_funcs->irq_handler;
		isr_thread = phy->hpd_funcs->isr_thread;
	}

	ret = devm_request_threaded_irq(
		&phy->i2c_phy->dev, phy->irq_num, irq_handler, isr_thread,
		IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_HIGH,
		phy->res->chip_name, phy);
	if (ret)
		goto error_irq;

	DRM_DEBUG_DRIVER("[Bridge_phy] %s register irq handler succeed %d.\n",
			 phy->res->chip_name, phy->irq_num);
	return 0;

error_irq:
	gpio_free(phy->res->irq_gpio);
	DRM_ERROR("Failed to request irq handler for irq %d.\n", phy->irq_num);
	return ret;
}

static int bridge_phy_hw_reset(struct loonggpu_bridge_phy *phy)
{
	if (phy->cfg_funcs && phy->cfg_funcs->hw_reset)
		phy->cfg_funcs->hw_reset(phy);

	return 0;
}

static int bridge_phy_misc_init(struct loonggpu_bridge_phy *phy)
{
	if (phy->misc_funcs && phy->misc_funcs->debugfs_init)
		phy->misc_funcs->debugfs_init(phy);

	return 0;
}

static int bridge_phy_regmap_init(struct loonggpu_bridge_phy *phy)
{
	int ret;
	struct regmap *regmap;

	mutex_init(&phy->ddc_status.ddc_bus_mutex);
	atomic_set(&phy->irq_status, 0);

	if (!phy->regmap_cfg)
		return 0;

	regmap = devm_regmap_init_i2c(phy->li2c->ddc_client,
				      phy->regmap_cfg);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		return -ret;
	}
	phy->phy_regmap = regmap;

	return 0;
}

static int bridge_phy_chip_id_verify(struct loonggpu_bridge_phy *phy)
{
	int ret;
	char str[NAME_SIZE_MAX] = "";

	if (phy->misc_funcs && phy->misc_funcs->chip_id_verify) {
		ret = phy->misc_funcs->chip_id_verify(phy, str);
		if (!ret)
			DRM_ERROR("Failed to verify chip %s, return [%s]\n",
				  phy->res->chip_name, str);
		strncpy(phy->res->vendor_str, str, NAME_SIZE_MAX - 1);
		return ret;
	}

	return -ENODEV;
}

static int bridge_phy_video_config(struct loonggpu_bridge_phy *phy)
{
	if (phy->cfg_funcs && phy->cfg_funcs->video_input_cfg)
		phy->cfg_funcs->video_input_cfg(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->video_output_cfg)
		phy->cfg_funcs->video_output_cfg(phy);

	return 0;
}

static int bridge_phy_hdmi_config(struct loonggpu_bridge_phy *phy)
{
	if (phy->cfg_funcs && phy->cfg_funcs->hdmi_audio)
		phy->cfg_funcs->hdmi_audio(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->hdmi_csc)
		phy->cfg_funcs->hdmi_csc(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->hdmi_hdcp_init)
		phy->cfg_funcs->hdmi_hdcp_init(phy);

	return 0;
}

static int bridge_phy_sw_init(struct loonggpu_bridge_phy *phy)
{
	bridge_phy_chip_id_verify(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->sw_reset)
		phy->cfg_funcs->sw_reset(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->reg_init)
		phy->cfg_funcs->reg_init(phy);
	if (phy->cfg_funcs && phy->cfg_funcs->sw_enable)
		phy->cfg_funcs->sw_enable(phy);
	bridge_phy_video_config(phy);
	bridge_phy_hdmi_config(phy);

	DRM_DEBUG_DRIVER("[Bridge_phy] %s sw init completed\n",
			 phy->res->chip_name);

	return 0;
}

static int bridge_phy_internal_init(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_device *adev = dc_bridge->adev;

	switch(adev->chip) {
	case dev_7a2000:
		return bridge_phy_ls7a2000_init(dc_bridge);
	case dev_2k2000:
		return bridge_phy_ls2k2000_init(dc_bridge);
	case dev_2k3000:
		return bridge_phy_ls2k3000_init(dc_bridge);
	default:
		DRM_DEBUG_DRIVER("No matching chip! Skip internal bridge phy init\n");
		break;
	}

	return 0;
}

static int bridge_phy_encoder_obj_select(struct loonggpu_dc_bridge *dc_bridge)
{
	int ret = 0;

	switch (dc_bridge->encoder_obj) {
	case ENCODER_CHIP_ID_EDP_LT6711:
		ret = bridge_phy_lt6711_init(dc_bridge);
		break;
	case ENCODER_CHIP_ID_EDP_LT9721:
		ret = bridge_phy_lt9721_init(dc_bridge);
		break;
	case ENCODER_CHIP_ID_LVDS_LT8619:
		ret = bridge_phy_lt8619_init(dc_bridge);
		break;
	case ENCODER_CHIP_ID_EDP_NCS8805:
		ret = bridge_phy_ncs8805_init(dc_bridge);
		break;
	case ENCODER_CHIP_ID_DP_LT8718:
		ret = bridge_phy_lt8718_init(dc_bridge);
		break;
	case ENCODER_CHIP_ID_INTERNAL_DVO:
	case ENCODER_CHIP_ID_INTERNAL_HDMI:
	case ENCODER_CHIP_ID_INTERNAL_EDP:
	case ENCODER_CHIP_ID_INTERNAL_DP:
	case ENCODER_CHIP_ID_NONE:
	case ENCODER_CHIP_ID_VGA_TRANSPARENT:
	case ENCODER_CHIP_ID_HDMI_TRANSPARENT:
	case ENCODER_CHIP_ID_EDP_TRANSPARENT:
		ret = bridge_phy_internal_init(dc_bridge);
		break;
	default:
		DRM_DEBUG_DRIVER("No matching chip! Skip bridge phy init\n");
		break;
	}

	return ret;
}

int bridge_phy_init(struct loonggpu_bridge_phy *phy)
{
	bridge_phy_hw_reset(phy);
	bridge_phy_regmap_init(phy);
	bridge_phy_register_irq_num(phy);

	bridge_phy_misc_init(phy);
	bridge_phy_bind(phy);
	bridge_phy_sw_init(phy);
	bridge_phy_register_irq_handle(phy);
	DRM_INFO("[Bridge_phy] %s init finish.\n", phy->res->chip_name);

	return 0;
}

/**
 * @section Bridge-phy interface
 */
struct loonggpu_bridge_phy *bridge_phy_alloc(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_bridge_phy *bridge_phy;
	int index = dc_bridge->display_pipe_index;
	struct connector_resource *connector_res =
			dc_bridge->adev->dc->link_info[index].connector;

	bridge_phy = kzalloc(sizeof(*bridge_phy), GFP_KERNEL);
	if (IS_ERR(bridge_phy)) {
		DRM_ERROR("Failed to alloc loonggpu bridge phy!\n");
		return NULL;
	}

	bridge_phy->display_pipe_index = dc_bridge->display_pipe_index;
	bridge_phy->bridge.driver_private = bridge_phy;
	bridge_phy->adev = dc_bridge->adev;
	bridge_phy->res = dc_bridge;
	bridge_phy->encoder = &dc_bridge->adev->mode_info.encoders[index]->base;
	bridge_phy->li2c = dc_bridge->adev->i2c[index];
	bridge_phy->connector_type = connector_res->type;
	dc_bridge->adev->mode_info.encoders[index]->bridge = bridge_phy;

	return bridge_phy;
}

struct loonggpu_dc_bridge
*dc_bridge_construct(struct loonggpu_dc *dc,
		     struct encoder_resource *encoder_res,
		     struct connector_resource *connector_res)
{
	struct loonggpu_dc_bridge *dc_bridge;
	const char *chip_name;
	u32 link;

	if (IS_ERR_OR_NULL(dc) ||
	    IS_ERR_OR_NULL(encoder_res) ||
	    IS_ERR_OR_NULL(connector_res))
		return NULL;

	dc_bridge = kzalloc(sizeof(*dc_bridge), GFP_KERNEL);
	if (IS_ERR_OR_NULL(dc_bridge))
		return NULL;

	link = encoder_res->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	dc_bridge->adev = dc->adev;
	dc_bridge->dc = dc;
	dc_bridge->display_pipe_index = link;

	dc_bridge->encoder_obj = encoder_res->chip;
	chip_name = get_encoder_chip_name(dc_bridge->encoder_obj);
	snprintf(dc_bridge->chip_name, NAME_SIZE_MAX, "%s", chip_name);
	dc_bridge->i2c_bus_num = encoder_res->i2c_id;
	dc_bridge->i2c_dev_addr = encoder_res->chip_addr;

	dc_bridge->hotplug = connector_res->hotplug;
	dc_bridge->edid_method = connector_res->edid_method;
	dc_bridge->gpio_placement = connector_res->gpio_placement;
	dc_bridge->irq_gpio = connector_res->irq_gpio;

	switch (dc_bridge->encoder_obj) {
		case ENCODER_CHIP_ID_EDP_NCS8805:
		case ENCODER_CHIP_ID_EDP_LT9721:
		case ENCODER_CHIP_ID_EDP_LT6711:
		case ENCODER_CHIP_ID_LVDS_LT8619:
		case ENCODER_CHIP_ID_DP_LT8718:
			dc->link_info[link].encoder->has_ext_encoder = true;
			break;
		default:
			dc->link_info[link].encoder->has_ext_encoder = false;
			break;
	}

	DRM_INFO("Encoder Parse: #0x%02x-%s feature:%d type:%s hotplug:%s.\n",
		 dc_bridge->encoder_obj, dc_bridge->chip_name, encoder_res->feature,
		 encoder_type_to_str(encoder_res->type),
		 hotplug_to_str(dc_bridge->hotplug));
	DRM_INFO("Encoder Parse: config_type:%s edid_method:%s reset_gpio:%d.\n",
		 encoder_config_to_str(encoder_res->config_type),
		 edid_method_to_str(dc_bridge->edid_method),
		 (encoder_res->feature >= 1) ? encoder_res->reset_gpio : 0);

	return dc_bridge;
}

static int bridge_phy_internal_register(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_device *adev = dc_bridge->adev;

	switch(adev->chip) {
	case dev_7a2000:
		return internal_bridge_ls7a2000_register(dc_bridge);
	case dev_2k2000:
		return internal_bridge_ls2k2000_register(dc_bridge);
	case dev_2k3000:
		return internal_bridge_ls2k3000_register(dc_bridge);
	default:
		DRM_DEBUG_DRIVER("No matching chip! Skip internal bridge phy register\n");
		break;
	}

	return 0;
}

int loonggpu_dc_bridge_init(struct loonggpu_device *adev, int link_index)
{
	struct loonggpu_dc_bridge *dc_bridge =
					adev->dc->link_info[link_index].bridge;
	int ret;

	if (link_index >= 2)
		return -1;

	bridge_phy_internal_register(dc_bridge);
	ret = bridge_phy_encoder_obj_select(dc_bridge);
	if (ret)
		return ret;

	return ret;
}

void loonggpu_bridge_suspend(struct loonggpu_device *adev)
{
	struct loonggpu_bridge_phy *phy;
	int i;

	for (i = 0; i < adev->dc->links; i++) {
		phy = adev->mode_info.encoders[i]->bridge;
		if (phy && phy->cfg_funcs && phy->cfg_funcs->suspend) {
			phy->cfg_funcs->suspend(phy);
			DRM_INFO("[Bridge_phy] %s suspend completed.\n",
					phy->res->chip_name);
		}
	}
}

void loonggpu_bridge_resume(struct loonggpu_device *adev)
{
	struct loonggpu_bridge_phy *phy = NULL;
	int i;

	for (i = 0; i < adev->dc->links; i++) {
		phy = adev->mode_info.encoders[i]->bridge;
		if (phy) {
			if (phy->cfg_funcs &&  phy->cfg_funcs->resume)
				phy->cfg_funcs->resume(phy);
			else {
				bridge_phy_hw_reset(phy);
				bridge_phy_sw_init(phy);
			}
			DRM_INFO("[Bridge_phy] %s resume completed.\n",
					phy->res->chip_name);
		}
	}
}
