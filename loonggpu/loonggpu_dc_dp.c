#include <linux/delay.h>
#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_dp.h"
#include "loonggpu_dc_crtc.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_resource.h"
#include "loonggpu_helper.h"

static const dp_bandwidth_entry_t bw_table[] = {
	{ 162000 * 1 / 3, DP_PHY_1P62G, DP_PHY_X1 , DP_LINK_1P62G, DP_LINK_X1},
	{ 162000 * 2 / 3, DP_PHY_1P62G, DP_PHY_X2 , DP_LINK_1P62G, DP_LINK_X2},
	{ 162000 * 4 / 3, DP_PHY_1P62G, DP_PHY_X4 , DP_LINK_1P62G, DP_LINK_X4},
	{ 270000 * 1 / 3, DP_PHY_2P7G,  DP_PHY_X1 , DP_LINK_2P7G,  DP_LINK_X1},
	{ 270000 * 2 / 3, DP_PHY_2P7G,  DP_PHY_X2 , DP_LINK_2P7G,  DP_LINK_X2},
	{ 270000 * 4 / 3, DP_PHY_2P7G,  DP_PHY_X4 , DP_LINK_2P7G,  DP_LINK_X4},
	{ 540000 * 1 / 3, DP_PHY_5P4G,  DP_PHY_X1 , DP_LINK_5P4G,  DP_LINK_X1},
	{ 540000 * 2 / 3, DP_PHY_5P4G,  DP_PHY_X2 , DP_LINK_5P4G,  DP_LINK_X2},
	{ 540000 * 4 / 3, DP_PHY_5P4G,  DP_PHY_X4 , DP_LINK_5P4G,  DP_LINK_X4},
};

static const dp_rate_info_t  rate_table[] = {
	{ DP_LINK_1P62G, DP_PHY_1P62G, 162 },
	{ DP_LINK_2P16G, DP_PHY_2P16G, 216 },
	{ DP_LINK_2P43G, DP_PHY_2P43G, 243 },
	{ DP_LINK_2P7G,  DP_PHY_2P7G,  270 },
	{ DP_LINK_3P24G, DP_PHY_3P24G, 324 },
	{ DP_LINK_4P32G, DP_PHY_4P32G, 432 },
	{ DP_LINK_5P4G,  DP_PHY_5P4G,  540 },
};

static inline int get_max_rate(unsigned int aux_rate)
{
	switch (aux_rate & 0xff) {
	case 0x6:
		return 162000;
	case 0xa:
		return 270000;
	default:
		return 540000;
	}
}

static inline int get_max_lane(unsigned int aux_lane)
{
	switch (aux_lane & 0xf) {
	case 0x1:
		return 1;
	case 0x2:
		return 2;
	default:
		return 4;
	}
}

static inline int rate_lane_unsupported(int rate, int lane, int idx)
{
	if ((rate == 270000) && (idx >= 6))
		return 1;
	if ((rate == 162000) && (idx >= 3))
		return 1;
	if ((lane == 2) && (idx == 2 || idx == 5 || idx == 8))
		return 1;
	if ((lane == 1) && (idx == 1 || idx == 2 || idx == 4 || idx == 5 || idx == 7 || idx == 8))
		return 1;

	return 0;
}

static bool dp_check_bandwidth(u32 clock, dp_feature_t *dp_param)
{
	unsigned int i;

	for (i = 0; i < sizeof(bw_table) / sizeof(bw_table[0]); i++) {
		if (clock <= bw_table[i].bw) {
			dp_param->dp_phy_rate = bw_table[i].phy_rate;
			dp_param->dp_phy_xlane = bw_table[i].phy_lane;
			dp_param->dp_link_rate = bw_table[i].link_rate;
			dp_param->dp_link_xlane = bw_table[i].link_lane;
			dp_param->dp_pixclk = clock;
			return true;
		}
	}

	DRM_INFO("NOTE: This pclk is not within the normal range!!\n");
	return false;
}

unsigned int aux_config(struct loonggpu_device *adev, unsigned int rd_wr, aux_msg_t aux_msg, int intf)
{
	unsigned int dp_detect_flag = 0;
	unsigned int val32;
	unsigned long count;
	unsigned char i;
	u32 value;

	if (in_interrupt()) {
		pr_warn("DP_AUX can not be used in interrupt context.\n");
		return -1;
	}

	if (rd_wr == READ) {
		DRM_DEBUG_DRIVER("aux read addr: 0x%x\n", aux_msg.addr);
		value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
		value &= ~(0x1 << 3);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel1, aux_msg.addr & 0xfffff);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel2, aux_msg.size & 0xfffff);

		value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
		value |= (0x1 << 2);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

		msleep(10);

		val32 =	dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
		count = 20;
		while ((val32 & 0x3) == 0) {
			val32 =	dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
			if (val32 & 0x2) {
				DRM_DEBUG_DRIVER("dp transaction success!\n");
				break;
			} else if (val32 & 0x1) {
				if ((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7) & 0xf00) == 0) {
					DRM_DEBUG_DRIVER("dp transaction success, MON7 is 0!\n");
					break;
				} else {
					DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
					DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
					DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
					DRM_DEBUG_DRIVER("dp transaction fail!\n");
					return -1;
				}
			}
			count--;
			msleep(10);

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

		for (i = 0; i <= aux_msg.size; i++) {
			DRM_DEBUG_DRIVER("aux data read: [aux base: 0x%x]: 0x%x\n", gdc_reg->dp_reg[intf].aux_monitor1 + i, \
						readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_monitor1 + i));
		}

	} else if (rd_wr == WRITE) {
		DRM_DEBUG_DRIVER("aux write addr: 0x%x\n", aux_msg.addr);
		value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
		value |= (0x1 << 3);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel1, aux_msg.addr & 0xfffff);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel2, aux_msg.size & 0xfffff);

		for (i = 0; i <= aux_msg.size; i++) {
			if (i < 8) {
				writeb((((aux_msg.data_low) >> (8*i)) & 0xff), adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_channel3 + i);
				DRM_DEBUG_DRIVER("aux data write: [aux base: 0x%x]: 0x%x\n", gdc_reg->dp_reg[intf].aux_channel3 + i,  \
								readb(adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_channel3 + i));
			} else
				writeb(((aux_msg.data_high >> (8 * (i - 8))) & 0xff), adev->loongson_dc_rmmio + gdc_reg->dp_reg[intf].aux_channel3 + i);
		}

		value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
		value |= (0x1 << 2);
		dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

		msleep(10);

		val32 = dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
		count = 20;
		while ((val32 & 0x3) == 0) {
			val32 = dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor0);
			if (val32 & 0x2) {
				DRM_DEBUG_DRIVER("dp transaction success!\n");
				break;
			} else if (val32 & 0x1) {
				if ((dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7) & 0xf00) == 0) {
					DRM_DEBUG_DRIVER("dp transaction success, MON7 is 0!\n");
					break;
				} else {
					DRM_DEBUG_DRIVER("aux monitor 5 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor5));
					DRM_DEBUG_DRIVER("aux monitor 6 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor6));
					DRM_DEBUG_DRIVER("aux monitor 7 date:0x%x\n", dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor7));
					DRM_DEBUG_DRIVER("dp transaction fail!\n");
					return -1;
				}
			}
			count--;
			msleep(1);

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
	}

	return 0;
}

static void dp_recheck_bandwidth(struct loonggpu_device *adev, u32 clock, dp_feature_t *dp_param, int intf)
{
	unsigned int i, aux_rate = 0, aux_lane = 0, tu_size = 0;
	unsigned int check_flag = 0;
	unsigned int max_tu_val = 55;
	aux_msg_t aux_data;
	uint64_t tmp;
	int max_rate;
	int max_lane;

	aux_data.addr = 0x1;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, READ, aux_data, intf);
	aux_rate = dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor1);

	aux_data.addr = 0x2;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, READ, aux_data, intf);
	aux_lane = dc_readl(adev, gdc_reg->dp_reg[intf].aux_monitor1);

	max_rate = get_max_rate(aux_rate);
	max_lane = get_max_lane(aux_lane);

	dp_param->dp_pixclk = clock;

	for (i = 0; i < sizeof(bw_table) / sizeof(bw_table[0]); i++) {
		if (bw_table[i].bw < clock)
			continue;

		if (rate_lane_unsupported(max_rate, max_lane, i))
			continue;

		dp_param->dp_phy_rate  = bw_table[i].phy_rate;
		dp_param->dp_phy_xlane = bw_table[i].phy_lane;
		dp_param->dp_link_rate = bw_table[i].link_rate;
		dp_param->dp_link_xlane = bw_table[i].link_lane;

		tmp = ((dp_param->dp_pixclk * 64) / bw_table[i].bw);
		tu_size = tmp + 1;

		if (tu_size <= max_tu_val) {
			check_flag = 1;
			break;
		}

		if (max_rate * max_lane == bw_table[i].bw * 3) {
			dp_param->dp_pixclk = (max_tu_val * bw_table[i].bw) / 64;
			check_flag = 1;
			break;
		}
	}

	DRM_INFO("dp_phy_rate: %d, dp_phy_xlane: %d, dp_link_rate: %d dp_pixclk: %d\n", dp_param->dp_phy_rate,  \
				dp_param->dp_phy_xlane, dp_param->dp_link_rate, dp_param->dp_pixclk);

	if (check_flag)
		DRM_INFO("This pclk is within the normal range.\n");
	else
		DRM_INFO("NOTE: This pclk is not within the normal range!!\n");

	return;
}

static void dp_phy_init(struct loonggpu_device *adev, dp_feature_t dp_param, int intf, uint32_t vswing, uint32_t preemp)
{
	uint64_t CLK_HS;
	uint64_t pixelclk_div_N;
	uint64_t pixelclk_div_F;
	uint32_t ln_vswing[4]= {0}, ln_preemp[4] = {0};
	uint32_t value;
	uint32_t i;

	/* enable phy vswing */
	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0 + 0x2c);
	value |= (0xf << 6);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0 + 0x2c, value);

	/* set phy rate & lane */
	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value &= ~(0x7 << 0x8);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value |= dp_param.dp_phy_rate << 0x8;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value &= ~(0x3 << 0x1);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value |= dp_param.dp_phy_xlane << 0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	/* set vswing
	* 0 or 1, 1; 2, 3/4; 3, 1/2; others, 1/4
	*/
	for (i = 0; i < 4; i++) {
		ln_vswing[i] = vswing;
		ln_preemp[i] = preemp;
	}

	value = ln_vswing[0] | (ln_vswing[1] << 3) | (ln_vswing[2] << 6) | (ln_vswing[3] << 9);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg7, value);
	udelay(100000);

	// set pre-emphasiss
	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg2);
	value &= 0xf;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg2, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg2);
	value |= (ln_preemp[0] << 8) | (ln_preemp[1] << 10) | (ln_preemp[2] << 12) | (ln_preemp[3] << 14);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg2, value);
	udelay(100000);

	/* 2k3000 default */
	CLK_HS = 2700;
	pixelclk_div_N = CLK_HS * 1000 / dp_param.dp_pixclk;
	pixelclk_div_F = CLK_HS * 1000 * 65536 / dp_param.dp_pixclk - pixelclk_div_N * 65536;

	/* DP/EDP phy reg 12 TODO*/
	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg12);
	value &= ~(0xffffffff);
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg12, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg12);
	value |= (0x1 << 31) | (pixelclk_div_F << 8) | pixelclk_div_N;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg12, value);

	/* enable phy tx */
	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value &= ~0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0);
	value |= 0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].phy_cfg0, value);

	DRM_DEBUG_DRIVER("wait dp phy ready......\n");
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x0 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg0));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x4 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg1));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x8 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg2));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0xc = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg3));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x10 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg4));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x14 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg5));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x18 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg6));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x1c = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg7));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x20 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg8));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x30 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_cfg12));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x80 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_monitor0));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x84 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_monitor1));
	DRM_DEBUG_DRIVER("addr_dp_phy_base + 0x88 = %x \n", dc_readl(adev, gdc_reg->dp_reg[intf].phy_monitor2));

	/* wait phy ready */
	while ((dc_readl(adev, gdc_reg->dp_reg[intf].phy_monitor0) & 0x3) != 0x3);

	DRM_DEBUG_DRIVER("dp phy ready......\n");
}

static void dp_link_init(struct loonggpu_device *adev, dp_feature_t dp_param, int intf, struct dc_timing_info *timing)
{
	uint32_t phy_rate, phy_lane;
	uint32_t link_rate, link_lane;
	uint32_t mode_bpc, tu_video_size;
	uint64_t dp_link_clk;
	uint64_t tmp;
	uint64_t dp_color_ratio = 1;	/* 2k3000_default */
	uint32_t dp_color_depth = 8;	/* 2k3000_default */
	uint32_t valid_rate = 0;
	uint32_t value;
	uint32_t i;

	/* color mode */
	if(dp_color_depth == 8)
		mode_bpc = 0;
	else if(dp_color_depth == 10)
		mode_bpc = 1;
	else {
		DRM_DEBUG_DRIVER("dp color depth erro! use default value\n");
		mode_bpc = 0;
	}

	/* dp MSA */
	value = dc_readl(adev,  gdc_reg->dp_reg[intf].sdp_cfg0);
	value |= (0x1 << 30);
	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg0, value);

	value = ((timing->vtotal - timing->vsync_start) << 16) | (timing->htotal - timing->hsync_start);
	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg62, value);

	value = (timing->vtotal << 16) | timing->htotal;
	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg63, value);

	value = (timing->vdisplay << 16) | timing->hdisplay;
	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg64, value);

	value = ((timing->vsync_end - timing->vsync_start) << 16) | (timing->hsync_end - timing->hsync_start);
	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg65, value);

	dc_writel(adev,  gdc_reg->dp_reg[intf].sdp_cfg61, 0x1);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value &= ~(0xf << 16);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value |= (mode_bpc << 16);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	/* link rate & lane count */
	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value &= ~(0xff);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value |= dp_param.dp_link_rate;
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value &= ~(0xff << 8);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1);
	value |= (dp_param.dp_link_xlane << 8);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg1, value);

	phy_rate = (dc_readl(adev,  gdc_reg->dp_reg[intf].phy_cfg0) >> 8) & 0x7;
	phy_lane = (dc_readl(adev,  gdc_reg->dp_reg[intf].phy_cfg0) >> 1) & 0x3;
	link_rate = (dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1) & 0xff);
	link_lane = (dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg1) >> 8) & 0xff;

	for (i = 0; i < sizeof(rate_table) / sizeof(rate_table[0]); i++) {
		if ((link_rate == rate_table[i].link_rate) && (phy_rate == rate_table[i].phy_rate)) {
			dp_link_clk = rate_table[i].clk;
			valid_rate = 1;
			break;
		}
	}

	if (!valid_rate) {
		DRM_INFO("dp rate ERROR......\n");
	}

	if (!(((link_lane == DP_LINK_X1) && (phy_lane == DP_PHY_X1)) ||
	     ((link_lane == DP_LINK_X2) && (phy_lane == DP_PHY_X2)) ||
	     ((link_lane == DP_LINK_X4) && (phy_lane == DP_PHY_X4)))) {
		DRM_INFO("dp lane ERROR......\n");
	}

	/* set tu size */
	tmp  = ((dp_param.dp_pixclk * 3 * dp_color_ratio * 64) / (dp_link_clk * link_lane)) / 1000;
	tu_video_size = (unsigned int)(tmp & 0xffffffff) + 1;

	DRM_DEBUG_DRIVER("==========================================================\n");
	DRM_DEBUG_DRIVER("pixclk: 0x%x\n", dp_param.dp_pixclk);
	DRM_DEBUG_DRIVER("dp_color_ratio: 0x%llx\n", dp_color_ratio);
	DRM_DEBUG_DRIVER("dp_link_clk: 0x%llx\n", dp_link_clk);
	DRM_DEBUG_DRIVER("link_lane: 0x%x\n", link_lane);

	DRM_DEBUG_DRIVER("tmp: 0x%llx\n", tmp);
	DRM_DEBUG_DRIVER("tu_video_size: 0x%x\n", tu_video_size);
	DRM_DEBUG_DRIVER("==========================================================\n");

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg4);
	value &= ~(0xff);
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg4, value);

	value = dc_readl(adev,  gdc_reg->dp_reg[intf].link_cfg4);
	value |= tu_video_size;
	dc_writel(adev,  gdc_reg->dp_reg[intf].link_cfg4, value);
}

static void dp_soft_training(struct loonggpu_device *adev, dp_feature_t dp_param, int intf)
{
	unsigned int ln_vswing[4] = {0}, ln_vswing_max[4] = {0};
	unsigned int ln_preemp[4] = {0}, ln_preemp_max[4] = {0};
	unsigned int ln_set[4] = {0};
	unsigned int dp_detect_flag = 0;
	unsigned int i, tmp, tps3_flag = 0;
	uint32_t ln_cr_done[4] = {0}, cr_done = 0; // CR DONE
	uint32_t ln_eq_done[4] = {0}; // CHANNEL EQ DONE
	uint32_t ln_sl_done[4] = {0}; // SYMBOL LOCKED
	uint32_t interlane_align_done, lt_done, lt_cnt;
	aux_msg_t aux_data;
	uint32_t value;

	/* dp soft reset */
	value = dc_readl(adev, gdc_reg->dp_reg[intf].link_cfg0);
	value |= (1 << 31);
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, value);

	/* TPS1*/
	/* enable dp TPS1 */
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, 0x3850038b);

	aux_data.addr = 0;
	aux_data.size = 5;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	dp_detect_flag = aux_config(adev, READ, aux_data, intf);
	if (dp_detect_flag == 1) {
		DRM_INFO("No Dp device detected! \n");
		goto end;
	}

	aux_data.addr = 0x2;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, READ, aux_data, intf);

	tps3_flag = (dc_readl(adev,  gdc_reg->dp_reg[intf].aux_monitor1) >> 6) & 0x1;
	if(tps3_flag == 1)
		DRM_DEBUG_DRIVER("TPS3 supported...\n");

	aux_data.addr = 0x600;
	aux_data.size = 0;
	aux_data.data_low = 0x2;
	aux_data.data_high = 0;
	aux_config(adev,  WRITE, aux_data, intf);
	udelay(10000);

	aux_data.addr = 0x600;
	aux_data.size = 0;
	aux_data.data_low = 0x1;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);
	udelay(10000);

	aux_data.addr = 0x600;
	aux_data.size = 0;
	aux_data.data_low = 0x1;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);
	udelay(10000);

	aux_data.addr = 0x100;
	aux_data.size = 0;
	aux_data.data_low = dp_param.dp_link_rate;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	aux_data.addr = 0x101;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, READ, aux_data, intf);

	aux_data.addr = 0x101;
	aux_data.size = 0;
	aux_data.data_low = 0xa0 | dp_param.dp_link_xlane;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	aux_data.addr = 0x111;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	aux_data.addr = 0x107;
	aux_data.size = 0;
	aux_data.data_low = 0x10;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	aux_data.addr = 0x101;
	aux_data.size = 0;
	aux_data.data_low = 0xa0 | dp_param.dp_link_xlane;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	aux_data.addr = 0x102;
	aux_data.size = 0;
	aux_data.data_low = 0x21;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);

	DRM_DEBUG_DRIVER("wait for CR_DONE!\n");
	udelay(10000);

	lt_cnt = 0;

	while (lt_cnt < 4) {
		/* training lane req from sink */
		aux_data.addr = 0x206;
		aux_data.size = 1;
		aux_data.data_low = 0;
		aux_data.data_high = 0;
		aux_config(adev, READ, aux_data, intf);

		tmp = dc_readl(adev,  gdc_reg->dp_reg[intf].aux_monitor1);
		for (i = 0; i < 4; i++) {
			ln_preemp[i] = (tmp >> (2 + i * 4)) & 0x3;
			ln_vswing[i] = lt_cnt;
			ln_vswing_max[i] = (ln_vswing[i] == 0x3) ? 1 : 0;
			ln_preemp_max[i] = (ln_preemp[i] == 0x3) ? 1 : 0;
			ln_set[i] = ln_vswing[i] | (ln_vswing_max[i] << 2) |
				(ln_preemp[i] << 3) | (ln_preemp_max[i] << 5);

			aux_data.addr = (0x103 + i);
			aux_data.size = 0;
			aux_data.data_low = ln_set[i];
			aux_data.data_high = 0;
			aux_config(adev, WRITE, aux_data,  intf);
		}
		udelay(10000);

		aux_data.addr = 0x202;
		aux_data.size = 5;
		aux_data.data_low = 0;
		aux_data.data_high = 0;
		aux_config(adev, READ, aux_data, intf);
		tmp = dc_readl(adev,  gdc_reg->dp_reg[intf].aux_monitor1);

		for (i = 0; i < 4; i++)
			ln_cr_done[i] = (tmp >> (i * 4)) & 0x1;

		cr_done = (dp_param.dp_link_xlane == 4) ? (ln_cr_done[0] & ln_cr_done[1] & ln_cr_done[2] & ln_cr_done[3]) :
					(dp_param.dp_link_xlane == 2) ? (ln_cr_done[0] & ln_cr_done[1]) :
					(dp_param.dp_link_xlane == 1) ? (ln_cr_done[0]) : 0;

		if (cr_done)
			break;

		lt_cnt++;
	}

	if (tps3_flag == 1) {
		DRM_INFO("set TPS3...\n");
		aux_data.addr = 0x102;
		aux_data.size = 0;
		aux_data.data_low = 0x23;
		aux_data.data_high = 0;
		aux_config(adev, WRITE, aux_data, intf);
		dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, 0x3810038b | (0x4 << 22)); // enable TPS3
	} else {
		DRM_INFO("set TPS2...\n");
		aux_data.addr = 0x102;
		aux_data.size = 0;
		aux_data.data_low = 0x22;
		aux_data.data_high = 0;
		aux_config(adev, WRITE, aux_data, intf);
		dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, 0x3810038b | (0x2 << 22)); // enable TPS2
	}

	lt_cnt = 0;
	lt_done = 0;

	while (lt_cnt < 4) {
		DRM_DEBUG_DRIVER("wait a second!\n");
		udelay(40000);

		/* training lane req from sink */
		aux_data.addr = 0x206;
		aux_data.size = 1;
		aux_data.data_low = 0;
		aux_data.data_high = 0;
		aux_config(adev, READ, aux_data, intf);
		tmp = dc_readl(adev,  gdc_reg->dp_reg[intf].aux_monitor1);

		for (i = 0; i < 4; i++) {
			ln_vswing[i] = (tmp >> (i * 4)) & 0x3;
			ln_preemp[i] = (tmp >> (2 + i * 4)) & 0x3;
			ln_vswing_max[i] = (ln_vswing[i] == 0x3) ? 1 : 0;
			ln_preemp_max[i] = (ln_preemp[i] == 0x3) ? 1 : 0;
			ln_set[i] = ln_vswing[i] | (ln_vswing_max[i] << 2) |
				(ln_preemp[i] << 3) | (ln_preemp_max[i] << 5);

			DRM_DEBUG_DRIVER("ln_set[%d] = %x \n", i, ln_set[i]);
			aux_data.addr = (0x103 + i);
			aux_data.size = 0;
			aux_data.data_low = ln_set[i];
			aux_data.data_high = 0;
			aux_config(adev, WRITE, aux_data, intf);
		}

		DRM_DEBUG_DRIVER("wait a second!\n");
		udelay(40000);

		aux_data.addr = 0x202;
		aux_data.size = 5;
		aux_data.data_low = 0;
		aux_data.data_high = 0;
		aux_config(adev, READ, aux_data, intf);
		tmp = dc_readl(adev,  gdc_reg->dp_reg[intf].aux_monitor1);

		for (i = 0; i < 4; i++) {
			ln_cr_done[i] = (tmp >> (i * 4)) & 0x1;
			ln_eq_done[i] = (tmp >> (1 + i * 4)) & 0x1;
			ln_sl_done[i] = (tmp >> (2 + i * 4)) & 0x1;
		}

		interlane_align_done = (tmp >> 16) & 0x1;

		lt_done = (dp_param.dp_link_xlane == 4) ?
			(ln_cr_done[0] & ln_cr_done[1] & ln_cr_done[2] & ln_cr_done[3] &
			 ln_eq_done[0] & ln_eq_done[1] & ln_eq_done[2] & ln_eq_done[3] &
			 ln_sl_done[0] & ln_sl_done[1] & ln_sl_done[2] & ln_sl_done[3] & interlane_align_done) :
			(dp_param.dp_link_xlane == 2) ?
			(ln_cr_done[0] & ln_cr_done[1] &
			 ln_eq_done[0] & ln_eq_done[1] &
			 ln_sl_done[0] & ln_sl_done[1] & interlane_align_done) :
			(ln_cr_done[0] & ln_eq_done[0] & ln_sl_done[0] & interlane_align_done);

		DRM_DEBUG_DRIVER("lt_done = 0x%x\n", lt_done);

		if (lt_done == 1)
			break;

		for (i = 0; i < 4; i++) {
			DRM_DEBUG_DRIVER("ln_cr_done[%d] = 0x%x\n", i, ln_cr_done[i]);
			DRM_DEBUG_DRIVER("ln_eq_done[%d] = 0x%x\n", i, ln_eq_done[i]);
			DRM_DEBUG_DRIVER("ln_sl_done[%d] = 0x%x\n", i, ln_sl_done[i]);
		}
		DRM_DEBUG_DRIVER("interlane_align_done = 0x%x\n", interlane_align_done);

		lt_cnt++;
	}

	aux_data.addr = 0x102;
	aux_data.size = 0;
	aux_data.data_low = 0;
	aux_data.data_high = 0;
	aux_config(adev, WRITE, aux_data, intf);
	udelay(10000);

end:
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, 0x3830038b);
	value = dc_readl(adev, gdc_reg->dp_reg[intf].link_cfg0);
	value |= (0x3 << 20);
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, value);

	return;
}

static int dp_aux_init(struct loonggpu_device *adev, int intf)
{
	unsigned int value;

	value = dc_readl(adev, gdc_reg->dp_reg[intf].link_cfg0);
	value |= (0x1 << 31);
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, value);
	msleep(100);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].aux_channel0);
	value |= (0x1 << 1);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel0, value);

	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel8, 0x1a0ffff8);
	dc_writel(adev, gdc_reg->dp_reg[intf].aux_channel9, 0x0004aeae);

	value = dc_readl(adev, gdc_reg->dp_reg[intf].link_cfg0);
	value |= 0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, value);

	msleep(50);

	return 0;
}

void ls2k3000_dp_pll_set(struct loonggpu_dc_crtc *crtc, int intf, struct dc_timing_info *timing)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	dp_feature_t recheck_dp_param;
	dp_feature_t dp_param;
	u32 link_cfg0;
	bool ret;

	ret = dp_check_bandwidth(timing->clock, &dp_param);
	if (!ret)
		return;

	link_cfg0 = dc_readl(adev, gdc_reg->dp_reg[intf].link_cfg0);
	link_cfg0 &= ~0x1;
	dc_writel(adev, gdc_reg->dp_reg[intf].link_cfg0, link_cfg0);
	dc_writel(adev, gdc_reg->crtc_reg[intf].cfg, 0);

	dp_recheck_bandwidth(adev, timing->clock, &recheck_dp_param, intf);
	dp_phy_init(adev, recheck_dp_param, intf, 0, 0);
	dp_link_init(adev, recheck_dp_param, intf, timing);
	dp_soft_training(adev, recheck_dp_param, intf);

	return;
}

bool ls2k3000_dp_enable(struct loonggpu_dc_crtc *crtc, int intf, bool enable)
{
	struct loonggpu_device *adev = crtc->dc->adev;
        aux_msg_t aux_data;
        u32 ret;

	if (!crtc->intf[intf].connected)
		return false;

	if ((enable && crtc->intf[intf].enabled) ||
		(!enable && !crtc->intf[intf].enabled))
		return true;

	if (enable) {
		aux_data.addr = 0x600;
		aux_data.size = 0;
		aux_data.data_low = 0x1;
		aux_data.data_high = 0;
		ret = aux_config(adev,  WRITE, aux_data, intf);
		if (!ret)
			crtc->intf[intf].enabled = true;
	} else {
		aux_data.addr = 0x600;
		aux_data.size = 0;
		aux_data.data_low = 2;
		aux_data.data_high = 0;
		ret = aux_config(adev,  WRITE, aux_data, intf);
		if (!ret)
			crtc->intf[intf].enabled = false;
	}

	DRM_INFO("SWITCH DISPLAY THROUGH AUX: %d-%d\n", enable, ret);
	return true;
}

void ls2k3000_dp_suspend(struct loonggpu_dc_crtc *crtc, int intf)
{
}

int ls2k3000_dp_resume(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	struct loonggpu_dc *dc = adev->dc;

	dc->hw_ops->first_hpd_detect(crtc, intf);
	return 0;
}

static int ls2k3000_dp_aux_detect_status(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	struct connector_resource *connector_res;
	unsigned int detect_flag = 1;
	aux_msg_t aux_data;
	int i;

	connector_res = adev->dc->link_info[intf].connector;
	if (connector_res) {
		/**
		" When the hotplug mode is set to interrupt mode, the initial hotplug status during first boot cannot be
		* retrieved via interrupt when only a DP display is connected. Therefore, the first hotplug connection status
		* is probed via aux. Additionally, when both DP and EDP are connected simultaneously, the initial connection
		* status for both EDP and DP cannot be obtained through interrupts either, necessitating AUX probing for their
		* first connection states as well. However, when the hotplug mode is configured to FORCE_ON, AUX probing for
		* hotplug status is not required.
		*/
		if (connector_res->hotplug != FORCE_ON) {
			aux_data.addr = 0;
			aux_data.size = 5;
			aux_data.data_low = 0;
			aux_data.data_high = 0;
			detect_flag = aux_config(adev, READ, aux_data, intf);

			for (i = 0; i < crtc->interfaces; i++) {
				switch (crtc->intf[i].type) {
				case INTERFACE_DP:
					if (!detect_flag) {
						adev->dp_status[1] = 0x2;
					} else {
						adev->dp_status[1] = 0x8;
					}
					break;
				case INTERFACE_EDP:
					if (!detect_flag) {
						adev->dp_status[0] = 0x1;
					} else {
						adev->dp_status[0] = 0x4;
					}
					break;
				}
			}
		}
	}

	return 0;
}

int ls2k3000_dp_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	dp_aux_init(adev, intf);
	return ls2k3000_dp_aux_detect_status(crtc, intf);
}

void l2k3000_hpd_irq_handler(struct loonggpu_device *adev, struct loonggpu_iv_entry *entry)
{
	u32 int_reg1 = 0;

	if (gdc_reg->global_reg.intr_1) {
		int_reg1 = dc_readl(adev, gdc_reg->global_reg.intr_1);
		if (int_reg1)
			dc_writel(adev, gdc_reg->global_reg.intr_1, int_reg1);
	}

	int_reg1 &= 0xffff;
	switch (int_reg1) {
	case LS2K3000_EDP_IN:
	case LS2K3000_EDP_OUT:
		adev->dp_status[0] = int_reg1;
		entry->src_id = DC_INT_ID_HPD_EDP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	case LS2K3000_DP_IN:
	case LS2K3000_DP_OUT:
		adev->dp_status[1] = int_reg1;
		entry->src_id = DC_INT_ID_HPD_DP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	case LS2K3000_EDP_IN_DP_IN:
		/*
		* When both EDP and DP displays are plugged in simultaneously, their hotplug interrupts
		* are asserted together. The value read from the interrupt status register 1 is 0x3, so
		* the corresponding hotplug events need to be serviced separately.
		**/
		adev->dp_status[0] = LS2K3000_EDP_IN;
		entry->src_id = DC_INT_ID_HPD_EDP;
		loonggpu_irq_dispatch(adev, entry);

		adev->dp_status[1] = LS2K3000_DP_IN;
		entry->src_id = DC_INT_ID_HPD_DP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	case LS2K3000_EDP_OUT_DP_IN:
		/*
		* When EDP unplug and DP plug-in events occur simultaneously, their hotplug interrupts
		* are asserted together. The value read from interrupt status register 1 is 0x6, requiring
		* separate handling of the corresponding hotplug events.
		*/
		adev->dp_status[0] = LS2K3000_EDP_OUT;
		entry->src_id = DC_INT_ID_HPD_EDP;
		loonggpu_irq_dispatch(adev, entry);

		adev->dp_status[1] = LS2K3000_DP_IN;
		entry->src_id = DC_INT_ID_HPD_DP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	case LS2K3000_EDP_IN_DP_OUT:
		/*
		* When EDP plug-in and DP unplug events occur simultaneously, their hotplug interrupts
		* are asserted together. The value read from interrupt status register 1 is 0x9, requiring
		* separate handling of the corresponding hotplug events.
		*/
		adev->dp_status[0] = LS2K3000_EDP_IN;
		entry->src_id = DC_INT_ID_HPD_EDP;
		loonggpu_irq_dispatch(adev, entry);

		adev->dp_status[1] = LS2K3000_DP_OUT;
		entry->src_id = DC_INT_ID_HPD_DP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	case LS2K3000_EDP_OUT_DP_OUT:
		/*
		* When both EDP and DP displays are unplugged simultaneously, their hotplug interrupts
		* are asserted together. The value read from interrupt status register 1 is 0xc, so
		* the corresponding hotplug events need to be serviced separately.
		**/
		adev->dp_status[0] = LS2K3000_EDP_OUT;
		entry->src_id = DC_INT_ID_HPD_EDP;
		loonggpu_irq_dispatch(adev, entry);

		adev->dp_status[1] = LS2K3000_DP_OUT;
		entry->src_id = DC_INT_ID_HPD_DP;
		loonggpu_irq_dispatch(adev, entry);
		break;
	default:
		break;
	}
}

void l2k3000_dp_first_hdp_detect(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	dp_feature_t dp_param;
	aux_msg_t aux_msg;

	/*
	* When entering S3 or S4 states with no EDP or DP connected, the enable bits of the aux_channel0and
	* link_cfg0registers are cleared. Therefore, prior to wake-up, a dp_aux_initoperation must be performed
	* to ensure the aux_channel0register is enabled for correct EDID reading and AUX connection state probing.
	*/
	dp_aux_init(adev, intf);
	dp_param.dp_phy_rate = DP_PHY_1P62G;
	dp_param.dp_phy_xlane = DP_PHY_X4;
	dp_param.dp_link_rate = DP_LINK_1P62G;
	dp_param.dp_link_xlane = DP_LINK_X4;
	dp_param.dp_pixclk = 148500;

	/*
	* The AUX functionality depends on the EDP/DP PHY. Therefore, the EDP/DP PHY must be
	* initialized prior to probing the link status via AUX
	*/
	dp_phy_init(adev, dp_param, intf, 0x7, 0);

	/*
	* Prior to entering S3 or S4 states, if no EDP/DP display is connected, a power off and on
	* cycle must be performed via the AUX channel. Otherwise, certain displays may fail to report
	* the correct connection status
	*/
	aux_msg.addr = 0x600;
	aux_msg.size = 0;
	aux_msg.data_low = 0x2;
	aux_msg.data_high = 0;
	aux_config(adev,  WRITE, aux_msg, intf);
	udelay(10000);

	aux_msg.addr = 0x600;
	aux_msg.size = 0;
	aux_msg.data_low = 0x1;
	aux_msg.data_high = 0;
	aux_config(adev,  WRITE, aux_msg, intf);
	udelay(10000);

	ls2k3000_dp_aux_detect_status(crtc, intf);
}

int ls2k3000_dp_audio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	return 0;
}

int ls2k3000_dp_noaudio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	return 0;
}
