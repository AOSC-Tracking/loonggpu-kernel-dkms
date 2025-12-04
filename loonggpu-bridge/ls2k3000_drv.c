#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_vbios.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_dp.h"
#include "bridge_phy.h"

static bool query_dp_monitor15_state(struct loonggpu_device *adev, int index)
{
	uint16_t high;
	uint16_t low;
	u32 value;

	value = dc_readl(adev, gdc_reg->dp_reg[index].link_monitor15);

	high  = (uint16_t)(value >> 16);
	low = (uint16_t)(value & 0xFFFF);

	if ((high == low || high - low == 1) && value) {
		return true;
	} else {
		return false;
	}
}

static bool reconfigure_dc_dp_reg(struct loonggpu_dc_crtc *crtc, struct dc_timing_info *timing, u32 crtc_cfg)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	u32 link;

	if (IS_ERR_OR_NULL(crtc) || IS_ERR_OR_NULL(timing))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	dc_interface_pll_set(crtc, crtc->timing);
	dc_writel_check(adev, gdc_reg->crtc_reg[link].cfg, crtc_cfg);

	return true;
}

static enum drm_connector_status ls2k3000_get_connect_status(struct loonggpu_bridge_phy *phy)
{
	struct drm_connector *connector = phy->connector;
	struct loonggpu_device *adev = phy->adev;
	struct loonggpu_dc_crtc *crtc = adev->dc->link_info[phy->connector->index].crtc;
	enum drm_connector_status status = connector_status_disconnected;
	aux_msg_t aux_msg;
	u32 link_cfg0;
	u32 crtc_cfg;
	u32 reg_val;
	u32 ret;
	u32 tmp;
	int i;

	if (connector->polled == 0) {
		status = connector_status_connected;
		crtc->intf[phy->connector->index].connected = true;
		/* Fixme: actually, we should not use index of contector to index intf, and should change later......*/
	} else {
		if (phy->hpd_funcs && phy->hpd_funcs->get_connect_status)
			status = phy->hpd_funcs->get_connect_status(phy);

		for (i = 0; i < crtc->interfaces; i++) {
			crtc->intf[i].connected = false;
			switch (crtc->intf[i].type) {
			case INTERFACE_HDMI:
				reg_val = dc_readl(adev, gdc_reg->global_reg.hdmi_hp_stat);
				if (reg_val & 0x1) {
					status = connector_status_connected;
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_DP:
			case INTERFACE_EDP:
				if (!in_interrupt()) {
					aux_msg.addr      = 0x0;
					aux_msg.size      = 5;
					ret = aux_config(adev, READ, aux_msg, i);

					if (!ret) {
						status = connector_status_connected;
						crtc->intf[i].connected = true;
						if (!query_dp_monitor15_state(adev, i)) {
							tmp = dc_readl(adev, gdc_reg->crtc_reg[i].cfg);
							crtc_cfg = tmp;
							tmp &= ~CRTC_CFG_ENABLE;
							dc_writel_check(adev, gdc_reg->crtc_reg[i].cfg, tmp);

							link_cfg0 = dc_readl(adev, gdc_reg->dp_reg[i].link_cfg0);
							link_cfg0 &= ~0x1;
							dc_writel(adev, gdc_reg->dp_reg[i].link_cfg0, link_cfg0);

							dc_writel_check(adev, gdc_reg->crtc_reg[i].cfg, 0);
							reconfigure_dc_dp_reg(crtc, crtc->timing, crtc_cfg);
						}
					}
				}
				break;
			}
		}
	}

	return status;
}

static void ls2k3000_hpd_init(struct loonggpu_bridge_phy *phy, struct loonggpu_connector* lconnector, bool has_ext_encoder)
{
	struct loonggpu_device *adev = phy->adev;
	struct connector_resource *connector_res = adev->dc->link_info[lconnector->connector_id].connector;

	if (has_ext_encoder) {
		if (lconnector->connector_id == 0) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C0;
			lconnector->irq_source_hpd = DC_IRQ_SOURCE_HPD_HDMI0_NULL;
		} else if (lconnector->connector_id == 1) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C1;
			lconnector->irq_source_hpd = DC_IRQ_SOURCE_HPD_HDMI1_NULL;
		}
	} else {
		if (lconnector->connector_id == 0) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C0;
			lconnector->irq_source_hpd = DC_IRQ_SOURCE_INVALID;
		} else if (lconnector->connector_id == 1) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C1;
			lconnector->irq_source_hpd = DC_IRQ_SOURCE_HPD_HDMI0;
		}
	}

	if (connector_res->hotplug != FORCE_ON)
		connector_res->hotplug = POLLING;

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

static struct bridge_phy_hpd_funcs ls2k3000_hpd_funcs = {
	.hpd_init = ls2k3000_hpd_init,
	.get_connect_status = ls2k3000_get_connect_status,
};

static enum drm_mode_status ls2k3000_mode_valid(struct drm_connector *connector,
					        const struct drm_display_mode *mode)
{
	if (mode->hdisplay > 4096 ||
	    mode->vdisplay > 2160 ||
	    mode->vdisplay < 480)
		return MODE_BAD;

	return MODE_OK;
}

static const u8 edid_header[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
};

int sort_edid(unsigned char internal_edid[256], unsigned char valid_edid[256])
{
	int header_position = 0;
	int remaining_bytes = 0;
	int index = -1;
	int i;

	for (i = 0; i < 256 - 7; i++) {
		if (memcmp(&internal_edid[i], edid_header, 8) == 0) {
			index = i;
			break;
		}
	}

	if (index == -1) {
		DRM_INFO("Error: EDID header not found!\n");
		return -1;
	}

	header_position = index;

	remaining_bytes = 256 - header_position;
	memcpy(valid_edid, &internal_edid[header_position], remaining_bytes);

	memcpy(&valid_edid[remaining_bytes], internal_edid, header_position);

	DRM_INFO("EDID processing complete!\n");

	return 0;
}

static unsigned int aux_transfer(struct loonggpu_device *adev,
				unsigned int rd_wr, aux_msg_t aux_msg,
				int intf, unsigned char index,
				unsigned char internal_edid[256])
{
	unsigned int dp_detect_flag = 0;
	unsigned int val32, count;
	unsigned int edid_value;
	unsigned char j;
	u32 value;

	DRM_DEBUG_DRIVER("aux read addr: 0x%x\n", aux_msg.addr);

	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel1, aux_msg.addr & 0xfffff);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel2, aux_msg.size & 0xfffff);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
	value |= (0x1 << 2);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

	udelay(10000);

	val32 =	dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
	count = 20;
	while ((val32 & 0x3) == 0) {
		val32 =	dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
		if (val32 & 0x2) {
			DRM_DEBUG_DRIVER("dp transaction success!\n");
			break;
		} else if (val32 & 0x1) {
			/* i2c deffer, wait....*/
			if ((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5) & 0x80)) {
				udelay(100000);
				return 2;
			}

			/* After stress test for reading edid on several boards
			 * (for dp, edp and edp to hdmi cases),
			 * only once case of aux_monitor7 & 0xf00 == 0 is found when reading
			 * edid error. Once the case occured, one more aux_transfer call
			 * will be ok. So, return 2 to indicate more aux_transfer call.
			 * */
			if ((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7) & 0xf00) == 0) {
				udelay(100000);
				return 2;
			} else {
				DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
				DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
				DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
				DRM_DEBUG_DRIVER("dp transaction fail!\n");
				return 3;
			}
		}
		count--;
		udelay(10000);

		/* todo can't run here. */
		if (!count) {
			DRM_DEBUG_DRIVER("NOTE: aux read nothing, because monitor 0 val is 0\n");
			DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
			DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
			DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
			dp_detect_flag = 1;
			return dp_detect_flag;
		}
	}

	for (j = 0; j <= aux_msg.size; j++) {
		DRM_DEBUG_DRIVER("aux data read: [aux base: 0x%x]: 0x%x\n", gdc_reg->dp_reg[intf].aux_monitor1 + j,
					readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_monitor1 + j));
		edid_value = readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_monitor1 + j);
		internal_edid[index * (aux_msg.size + 1) + j] = edid_value;
	}

	DRM_DEBUG_DRIVER("EDID: index=%02d, offse=0x%02x, <0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x>\n",
			index, index * (aux_msg.size + 1),
			internal_edid[index * (aux_msg.size + 1) + 0],
			internal_edid[index * (aux_msg.size + 1) + 1],
			internal_edid[index * (aux_msg.size + 1) + 2],
			internal_edid[index * (aux_msg.size + 1) + 3],
			internal_edid[index * (aux_msg.size + 1) + 4],
			internal_edid[index * (aux_msg.size + 1) + 5],
			internal_edid[index * (aux_msg.size + 1) + 6],
			internal_edid[index * (aux_msg.size + 1) + 7]);

	return 0;
}


static bool drm_edid_block_checksum(const unsigned char *edid)
{
	unsigned char checksum = 0, i;

	for (i = 0; i < 128; i++) {
		checksum += edid[i];
	}

	return !checksum;
}

static bool read_edid_form_aux(struct loonggpu_device *adev, int intf, struct drm_connector *connector, unsigned char *edid)
{
	unsigned int loop_deffer, loop_head;
	int size = sizeof(u8) * EDID_LENGTH * 2;
	bool ret0 = true, ret1 = true;
	unsigned char internal_edid[256] = {0};
	unsigned char block_num;
	aux_msg_t aux_msg;
	unsigned int value;
	int dp_detect;
	int index;

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel8);
	value |= 0x3 | (0xff << 16);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel8, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
	value &= ~(0x1 << 3);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

	loop_deffer = 10;
	loop_head = 32;

	/* read block0  (16*8=128byte) */
	for (index = 0; index < 16; index++) {
		aux_msg.addr      = 0x50;
		aux_msg.size      = 7;
		aux_msg.data_low  = 0;
		aux_msg.data_high = 0;
		dp_detect = aux_transfer(adev, READ, aux_msg, intf, index, internal_edid);
		if (index == 0 && memcmp(internal_edid, edid_header, 8) && --loop_head) {
			index = index - 1;
			continue;
		}

		if (!loop_head)
			return false;

		if (dp_detect == 1)
			DRM_DEBUG_DRIVER("No Dp device detected! \n");

		if (dp_detect == 2 && --loop_deffer) {
			DRM_DEBUG_DRIVER("dp read edid index %d, loop_deffer %d!\n", index, loop_deffer);
			index = index - 1;
			continue;
		}

		if (!loop_deffer)
			return false;

		loop_deffer = 10;
	}

	if (!drm_edid_block_checksum(internal_edid) || memcmp(internal_edid, edid_header, 8))
		ret0 = false;

	DRM_DEBUG_DRIVER("read block 0 end. %s\n", ret0 ? "ok": "fail");

	/* read other block  */
	block_num = internal_edid[126];

	if (block_num) {
		for (index = 16; index < 32; index++) {
			aux_msg.addr      = 0x50;
			aux_msg.size      = 7;
			aux_msg.data_low  = 0;
			aux_msg.data_high = 0;
			dp_detect = aux_transfer(adev, READ, aux_msg, intf, index, internal_edid);

			if (dp_detect == 1)
				DRM_DEBUG_DRIVER("no dp device detected! \n");

			if (dp_detect == 2 && --loop_deffer) {
				DRM_DEBUG_DRIVER("dp read edid index %d, loop_deffer %d!\n", index, loop_deffer);
				index = index - 1;
				continue;
			}

			if (!loop_deffer)
				return false;

			loop_deffer = 10;
		}

		if (!drm_edid_block_checksum(&internal_edid[128]))
			ret1 = false;

		DRM_DEBUG_DRIVER("read block 1 end. %s\n", ret1 ? "ok": "fail");
	}

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel8);
	value &= ~0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel8, value);

	memcpy(edid, internal_edid, size);

	return (ret0 && ret1);
}

static unsigned int dp_read_edid(struct loonggpu_device *adev, int intf, struct drm_connector *connector)
{
	int size = sizeof(u8) * EDID_LENGTH * 2;
	unsigned char valid_edid[256];
	unsigned long count = 0;
	struct edid *edid;

	if (!read_edid_form_aux(adev, intf, connector, valid_edid)) {
		DRM_INFO("read edid failed index-%d\n", intf);
		return 0;
	}

	edid = kmalloc(size, GFP_KERNEL);
	if (edid) {
		memcpy(edid, valid_edid, size);
		drm_connector_update_edid_property(connector, edid);
		count = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return count;
}

static int ls2k3000_get_modes(struct loonggpu_bridge_phy *phy,
                           struct drm_connector *connector)
{
       struct loonggpu_device *adev = phy->adev;
       int index = phy->display_pipe_index;
       unsigned int count = 0;

       count = dp_read_edid(adev, index, connector);

       return count;
}

static struct bridge_phy_ddc_funcs ls2k3000_ddc_funcs = {
       .get_modes = ls2k3000_get_modes,
};

static const struct bridge_phy_cfg_funcs ls2k3000_cfg_funcs = {
        .mode_valid = ls2k3000_mode_valid,
};

int internal_bridge_ls2k3000_register(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_bridge_phy *ls2k3000_phy;
	int index = dc_bridge->display_pipe_index;
	struct connector_resource *connector_res =
			dc_bridge->adev->dc->link_info[index].connector;

	ls2k3000_phy = kzalloc(sizeof(*ls2k3000_phy), GFP_KERNEL);
	if (IS_ERR(ls2k3000_phy)) {
		DRM_ERROR("Failed to alloc loonggpu bridge phy!\n");
		return -1;
	}

	ls2k3000_phy->display_pipe_index = index;
	ls2k3000_phy->bridge.driver_private = ls2k3000_phy;
	ls2k3000_phy->adev = dc_bridge->adev;
	ls2k3000_phy->res = dc_bridge;
	ls2k3000_phy->li2c = dc_bridge->adev->i2c[index];
	ls2k3000_phy->connector_type = connector_res->type;

	ls2k3000_phy->cfg_funcs = &ls2k3000_cfg_funcs;
	ls2k3000_phy->hpd_funcs = &ls2k3000_hpd_funcs;
	ls2k3000_phy->ddc_funcs = &ls2k3000_ddc_funcs;

	/* TODO:
	 * Use VBIOS config */
	if (dc_bridge->display_pipe_index == 0)
		ls2k3000_phy->connector_type = DRM_MODE_CONNECTOR_DisplayPort;
	if (dc_bridge->display_pipe_index == 1)
		ls2k3000_phy->connector_type = DRM_MODE_CONNECTOR_HDMIA;

	dc_bridge->internal_bp = ls2k3000_phy;
	DRM_DEBUG_DRIVER("internal bridge phy register success!\n");

	return 0;
}

int bridge_phy_ls2k3000_init(struct loonggpu_dc_bridge *dc_bridge)
{
	struct loonggpu_bridge_phy *ls2k3000_phy;

	ls2k3000_phy = bridge_phy_alloc(dc_bridge);

	return bridge_phy_init(ls2k3000_phy);
}

