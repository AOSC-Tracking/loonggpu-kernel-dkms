#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_vbios.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_dp.h"
#include "bridge_phy.h"

static enum drm_connector_status ls2k3000_get_connect_status(struct loonggpu_bridge_phy *phy)
{
	struct drm_connector *connector = phy->connector;
	struct loonggpu_device *adev = phy->adev;
	struct loonggpu_dc_crtc *crtc = adev->dc->link_info[phy->connector->index].crtc;
	enum drm_connector_status status = connector_status_disconnected;
	u32 reg_val;
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
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_DP:
				/*
				* Don't clear dp_status, which is set by interrupt asynchronously.
				* */
				if (adev->dp_status[1] & 0x2) {
					crtc->intf[i].connected = true;
				}
				break;
			case INTERFACE_EDP:
				/*
				* Don't clear dp_status, which is set by interrupt asynchronously.
				* */
				if (adev->dp_status[0] & 0x1) {
					crtc->intf[i].connected = true;
				}
				break;
			}
		}
	}

	for (i = 0; i < crtc->interfaces; i++) {
		if (crtc->intf[i].connected == true)
			status = connector_status_connected;
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
			lconnector->irq_source_hpd[0] = DC_IRQ_SOURCE_HPD_EDP_NULL;
		} else if (lconnector->connector_id == 1) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C1;
			lconnector->irq_source_hpd[0] = DC_IRQ_SOURCE_HPD_HDMI0_NULL;
			lconnector->irq_source_hpd[1] = DC_IRQ_SOURCE_HPD_DP_NULL;
		}
	} else {
		if (lconnector->connector_id == 0) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C0;
			lconnector->irq_source_hpd[0] = DC_IRQ_SOURCE_HPD_EDP;
		} else if (lconnector->connector_id == 1) {
			lconnector->irq_source_i2c = DC_IRQ_SOURCE_I2C1;
			lconnector->irq_source_hpd[0] = DC_IRQ_SOURCE_HPD_HDMI0;
			lconnector->irq_source_hpd[1] = DC_IRQ_SOURCE_HPD_DP;
		}
	}

	if (connector_res->hotplug != FORCE_ON)
		connector_res->hotplug = IRQ;

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
					      struct drm_display_mode *mode)
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

static unsigned int aux_transfer(struct loonggpu_device *adev,
				int intf,
				unsigned int pkt_size,
				unsigned char *data)
{
	unsigned int val32, count;
	unsigned char i, j;

	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel1, 0x50);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel2, (pkt_size - 1) & 0xfffff);

	val32 = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
	val32 |= (0x1 << 2);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, val32);

	udelay(10000);

	count = 20;
	for (i = 0; i < count; i++) {
		val32 =	dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
		if (val32 & 0x2) {
			DRM_DEBUG_DRIVER("dp transaction success!\n");
			for (j = 0; j < pkt_size; j++) {
				DRM_DEBUG_DRIVER("aux data read: [aux base: 0x%x]: 0x%x\n", gdc_reg->dp_reg[intf].aux_monitor1 + j,
							readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_monitor1 + j));
				data[j] = readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_monitor1 + j);
			}
			return 0;
		} else if (val32 & 0x1) {
			/*
			 * 1.i2c deffer, wait....
			 *
			 * 2. After stress test for reading edid on several boards
			 * (for dp, edp and edp to hdmi cases),
			 * only once case of aux_monitor7 & 0xf00 == 0 is found when reading
			 * edid error. Once the case occured, one more aux_transfer call
			 * will be ok. So, return 2 to indicate more aux_transfer call.
			 * */
			if ((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5) & 0x80) ||
				((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7) & 0xf00) == 0)){
				udelay(100000);
				return 2;
			} else {
				DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
				DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
				DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
				DRM_DEBUG_DRIVER("dp transaction fail!\n");
				return 1;
			}
		}
		udelay(10000);
	}

	/* todo can't run here. */
	if (i == count) {
		DRM_DEBUG_DRIVER("NOTE: aux read nothing, because monitor 0 val is 0\n");
		DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
		DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
		DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
		return 1;
	}

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

static bool read_one_packet(struct loonggpu_device *adev, int intf, int pkt_size, unsigned char *data)
{
	int retry_cnt = 10;
	int ret;
	int i;

	for (i = 0; i < retry_cnt; i++) {
		ret = aux_transfer(adev, intf, pkt_size, data);

		if (!ret) {
			return true;
		} else if (ret == 1) {
			DRM_DEBUG_DRIVER("No Dp device detected! \n");
			return false;
		}
	}

	return false;
}

static bool header_is_ok(struct loonggpu_device *adev, int intf, unsigned int pkt_size)
{
	unsigned char data[256] = {0};
	int index;
	unsigned pkt_cnt = 256/pkt_size;

	for (index = 0; index < (7/pkt_size); index++) {
		if (false == read_one_packet(adev, intf, pkt_size, &data[index * pkt_size])) {
			return false;
		}
	}

	for (; index < pkt_cnt; index++) {
		if (false == read_one_packet(adev, intf, pkt_size, &data[index * pkt_size])) {
			return false;
		}
		if (!memcmp(&data[(index - (7/pkt_size)) * pkt_size], edid_header, 8)) {
			return true;
		}
	}

	return false;
}

static bool read_edid_form_aux(struct loonggpu_device *adev, int intf, struct drm_connector *connector, unsigned char *edid)
{
	unsigned char internal_edid[2*EDID_LENGTH] = {0};
	unsigned char block_num;
	unsigned int value;
	int index;
	unsigned int pkt_size = 0x8; /* can be 1, 2, 4, 8 */
	unsigned pkt_cnt = (EDID_LENGTH * 2)/pkt_size;

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel8);
	value |= 0x3 | (0xff << 16);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel8, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
	value &= ~(0x1 << 3);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

	/* find out header */
	if (!header_is_ok(adev, intf, pkt_size)) {
		return false;
	}

	memcpy(internal_edid, edid_header, 8);

	/* read block0 */
	pkt_cnt = 128/pkt_size;
	for (index = 8/pkt_size; index < pkt_cnt; index++) {
		if (false == read_one_packet(adev, intf, pkt_size, &internal_edid[index * pkt_size])) {
			return false;
		}
	}

	if (!drm_edid_block_checksum(internal_edid)) {
		return false;
	}

	/* read block1 */
	block_num = internal_edid[126];
	if (block_num) {
		for (index = pkt_cnt; index < (2 * pkt_cnt) ; index++) {
			if (false == read_one_packet(adev, intf, pkt_size, &internal_edid[index * pkt_size])) {
				return false;
			}
		}
		if (!drm_edid_block_checksum(&internal_edid[128])) {
			return false;
		}
	}

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel8);
	value &= ~0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel8, value);

	memcpy(edid, internal_edid, EDID_LENGTH * 2);

	return true;
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

