#include <linux/delay.h>
#include "loonggpu.h"
#include "loonggpu_dc.h"
#include "loonggpu_dc_hdmi.h"
#include "loonggpu_dc_crtc.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_helper.h"

void ls7a2000_hdmi_i2c_set(struct loonggpu_dc_crtc *crtc, int intf, bool use_gpio_i2c)
{
	u32 value;
	struct loonggpu_device *adev = crtc->dc->adev;

	value = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
	if (use_gpio_i2c)
		value &= ~(1 << 8);
	else
		value |= (1 << 8);
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, value);
}

void ls7a2000_hdmi_pll_set(struct loonggpu_dc_crtc *crtc, int intf, int clock)
{
	int val;
	int count = 0;
	struct loonggpu_device *adev = crtc->dc->adev;
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].phy_pllcfg, 0x0);

	if (clock >= 170000)
		val = (0x0 << 13) | (0x28 << 6) | (0x10 << 1) | (0 << 0);
	else if (clock >= 85000  && clock < 170000)
		val = (0x1 << 13) | (0x28 << 6) | (0x8 << 1) | (0 << 0);
	else if (clock >= 42500 && clock < 85000)
		val = (0x2 << 13) | (0x28 << 6) | (0x4 << 1) | (0 << 0);
	else if (clock >= 21250 && clock < 42500)
		val = (0x3 << 13) | (0x28 << 6) | (0x2 << 1) | (0 << 0);

	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].phy_pllcfg, val);
	mdelay(5);
	val |= (1 << 0);
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].phy_pllcfg, val);

	/* wait pll lock */
	while (!(dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].phy_pllcfg) & 0x10000)) {
		count++;
		if (count >= 1000) {
			DRM_ERROR("LOONGGPU HDMI PHY PLL lock failed\n");
			return;
		}
	}
}

bool ls7a2000_hdmi_enable(struct loonggpu_dc_crtc *crtc, int intf, bool enable)
{
	u32 hdmi_ctrl, hdmi_phy;
	struct loonggpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(adev))
		return false;

	if (intf >= MAX_DC_INTERFACES)
		return false;

	hdmi_phy = 0xf02;
	hdmi_phy |= dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].phy_ctrl);
	hdmi_ctrl = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
	if (enable && (hdmi_phy & HDMI_PHY_CTRL_ENABLE) && (hdmi_ctrl & HDMI_CTRL_ENABLE))
		return true;

	if (!enable && (!(hdmi_phy & HDMI_PHY_CTRL_ENABLE))&& (!(hdmi_ctrl & HDMI_CTRL_ENABLE)))
		return true;

	if (enable) {
		hdmi_phy |= HDMI_PHY_CTRL_ENABLE;
		hdmi_ctrl = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
		hdmi_ctrl |= HDMI_CTRL_ENABLE;
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, hdmi_ctrl);
	} else {
		hdmi_ctrl = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
		hdmi_ctrl &= ~HDMI_CTRL_ENABLE;
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, hdmi_ctrl);
		msleep(50);
		hdmi_phy &= ~HDMI_PHY_CTRL_ENABLE;
	}

	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].phy_ctrl, hdmi_phy);

	return true;
}

void ls7a2000_hdmi_suspend(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	crtc->hdmi_ctrl[intf] = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
}

int ls7a2000_hdmi_resume(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, crtc->hdmi_ctrl[intf]);
	if (crtc->hdmi_ctrl[intf] & HDMI_CTRL_AUDIO_ENABLE) {
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].zoneidle, 0x00400040);
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_ncfg, 6272);
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_ctscfg, 0x80000000);
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_infoframe, 0x15);
		dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_sample, 0x1);
	}

	return 0;
}

int ls7a2000_hdmi_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, 0x280);
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].phy_ctrl, 0xf02);
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].zoneidle, 0x00400040);

	return 0;
}

int ls7a2000_hdmi_noaudio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	u32 val;
	struct loonggpu_device *adev = crtc->dc->adev;

	val = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, 0x282 | (val & 1));

	return 0;
}

int ls7a2000_hdmi_audio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	u32 val;
	struct loonggpu_device *adev = crtc->dc->adev;

	val = dc_readl(adev, gdc_reg->hdmi_reg_v1[intf].ctrl);
	if ((val & 0x2) && (val & 0x4))
		return 0;

	/* enable hdmi audio, but don't touch EN bit which will be
	 * update in loonggpu_hdmi_enable() */
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].ctrl, 0x286 | (val & 0x1));

	/* Audio N 
	 * 44.1KHz * 4, dynamic update N && CTS value */
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_ncfg, 6272);

	/* Enable Send CTS */
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_ctscfg, 0x80000000);

	/* Audio AIF
	 * enable AIF,set freq,and set CC = 1, CA = 0
	 * Update AIF */
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_infoframe, 0x15);

	/* Audio Sample Packet */
	dc_writel(adev, gdc_reg->hdmi_reg_v1[intf].audio_sample, 0x1);

	return 0;
}

void ls2k3000_hdmi_i2c_set(struct loonggpu_dc_crtc *crtc, int intf, bool use_gpio_i2c)
{
	u32 value;
	struct loonggpu_device *adev = crtc->dc->adev;

	value = dc_readl(adev, gdc_reg->hdmi_reg_v2[0].ctrl);
	if (use_gpio_i2c)
		value &= ~(1 << 8);
	else
		value |= (1 << 8);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[0].ctrl, value);
}

#define PCLK_PRECISION_INDICATOR 10000
static struct mpll {
	uint32_t multiplier;
	uint32_t tx_clkdiv;
	uint32_t tx_rate;
} mpllcfg;

static uint32_t get_tx_clkdiv(void)
{
	uint32_t tx_clkdiv;

	switch (mpllcfg.tx_clkdiv) {
  	  case 1:
  	    tx_clkdiv = 0;
  	    break;
  	  case 2:
  	    tx_clkdiv = 1;
  	    break;
  	  case 3:
  	    tx_clkdiv = 4;
  	    break;
  	  case 4:
  	    tx_clkdiv = 2;
  	    break;
  	  case 5:
  	    tx_clkdiv = 5;
  	    break;
  	  case 6:
  	    tx_clkdiv = 6;
  	    break;
  	  case 8:
  	    tx_clkdiv = 3;
  	    break;
  	  case 10:
  	    tx_clkdiv = 7;
  	    break;
  	  default:
  	    tx_clkdiv = 0;
  	    break;
  	}

	return tx_clkdiv;
}

static uint32_t get_tx_rate(void)
{
	uint32_t tx_rate;

	switch (mpllcfg.tx_rate) {
  	  case 1:
  	    tx_rate = 0;
  	    break;
  	  case 2:
  	    tx_rate = 1;
  	    break;
  	  case 4:
  	    tx_rate = 2;
  	    break;
  	  case 8:
  	    tx_rate = 3;
  	    break;
  	  default:
  	    tx_rate = 0;
  	    break;
  	}

	return tx_rate;
}

static uint32_t hdmi_phy_cr_access(struct loonggpu_device *adev, uint32_t cr_wr_en, uint32_t cr_wdata, uint32_t cr_addr)
{
	uint32_t tmp;
	void __iomem *io_base = adev->io_base;
	if (io_base == NULL)
		return false;

	writel(cr_wdata, io_base + 0x790);
	/* cr_wr_en 0: read, 1: write */
	writel(cr_addr | (cr_wr_en << 26) | (0x1 << 27), io_base + 0x794);

	/* clear en */
	tmp = readl(io_base + 0x794);
	writel(tmp & (~(0x1 << 27)), io_base + 0x794);

	/* busy */
	while ((readl(io_base + 0x79c) & (0x1 << 31)) == (0x1 << 31));

	if (cr_wr_en == 0)
		tmp = readl(io_base + 0x798) & 0xffff;
	else
		tmp = 0;

	return tmp;
}

static void dc_global_reg_config(struct loonggpu_device *adev, uint32_t *dc_global_reg_save, uint8_t flag)
{
	if (flag == 1) {
		dc_global_reg_save[0] = dc_readl(adev, gdc_reg->global_reg.gpio_cfg_off);
		dc_writel(adev, gdc_reg->global_reg.gpio_cfg_off, 0xffff);
	} else if (flag == 0) {
		/* restore global reg value */
		dc_writel(adev, gdc_reg->global_reg.gpio_cfg_off, dc_global_reg_save[0]);
	}
}

static void configmpll(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	uint32_t tx_rate, tx_clkdiv, phy_clk, high_phy_clk, tmds_clk_ratio;
	uint32_t dc_global_reg_save[6];
	u32 val;

	tx_clkdiv = get_tx_clkdiv();
	tx_rate   = get_tx_rate();
	phy_clk   = ((2 * mpllcfg.multiplier * 27) / mpllcfg.tx_clkdiv / 5 / mpllcfg.tx_rate) * 10;
	high_phy_clk = 3400;

	tmds_clk_ratio = ((phy_clk <= high_phy_clk) ? 0 : 1);

	DRM_INFO( "[2K3000] PhyClk %d Mhz\n", phy_clk);
	DRM_INFO( "[2K3000] Multiplier %d\n", mpllcfg.multiplier);
	DRM_INFO( "[2K3000] TxClkDiv %d\n", mpllcfg.tx_clkdiv);
	DRM_INFO( "[2K3000] TxRate %d\n", mpllcfg.tx_rate);

	/* dc global reg config save
	 *
	 * !!Notice
	 *
	 * This call is used to workaround hw bug.
	 *
	 * Any global reg read access with non-zero returned value will
	 * destroy hdmi phy registers, so the call save int reg and write
	 * it to zero and read back befor hdmi phy configuration. After
	 * config done, restore saved int reg. */
	dc_global_reg_config(adev, dc_global_reg_save, 1);

	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg1, 0x33330000);

	/* wait sram init done */
	while ((dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].phy_mon0) & (0x1 << 14)) == 0);

	/* ext ld done */
	val = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg2);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg2, val & (~(0x1 << 29)));
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg2, val | (0x1 << 29));

	/* wait sram boot done */
	while ((dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].phy_mon0) & (0xf << 4)) != 0);
	DRM_INFO("hdmi phy boot up done...\n");
	hdmi_phy_cr_access(adev, 1, 0x1f, 0x4000);
	hdmi_phy_cr_access(adev, 1, mpllcfg.multiplier << 8, 0x4001);
	hdmi_phy_cr_access(adev, 1, (tx_clkdiv << 12) | 0x800, 0x4002);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x4003);
	hdmi_phy_cr_access(adev, 1, tx_rate << 11, 0x4006);
	hdmi_phy_cr_access(adev, 1, 0x226, 0x4042);
	hdmi_phy_cr_access(adev, 1, tmds_clk_ratio, 0x5011);
	hdmi_phy_cr_access(adev, 1, 0x718, 0x5000);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x5001);
	hdmi_phy_cr_access(adev, 1, tmds_clk_ratio, 0x5111);
	hdmi_phy_cr_access(adev, 1, 0x718, 0x5100);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x5101);
	hdmi_phy_cr_access(adev, 1, tmds_clk_ratio, 0x5211);
	hdmi_phy_cr_access(adev, 1, 0x718, 0x5200);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x5201);
	hdmi_phy_cr_access(adev, 1, tmds_clk_ratio, 0x5311);
	hdmi_phy_cr_access(adev, 1, 0x718, 0x5300);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x5301);
	hdmi_phy_cr_access(adev, 1, 0x5811, 0x4041);
	hdmi_phy_cr_access(adev, 1, 0x0, 0x4040);
	hdmi_phy_cr_access(adev, 1, 0x224, 0x4042);

	/* power up */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg1, 0);

	/* enable tx valid */
	val = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg0);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg0, val & (~(0x1 << 29)));
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg0, val | (0x1 << 29));

	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg1, 0x33330000);
	msleep(10);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg1, 0);
	DRM_INFO("hdmi mpll config done......\n");

	/* dc global reg config restore */
	dc_global_reg_config(adev, dc_global_reg_save, 0);
}

static int ls2k3000_calc_phy_clk(unsigned int pixclock_khz)
{
	unsigned int tx_clkdiv_set[] = {1, 2, 3, 4, 5, 6, 8, 10};
	unsigned int prec_set[] = {1, 5, 10, 50, 100, 1000};	//in 1/PCLK_PRECISION_INDICATOR
	unsigned int tx_rate, multiplier, tx_clkdiv;
	int i, j;
	unsigned int precision_req, precision;
	unsigned int multiplier_min, multiplier_max, multiplier_mid;
	unsigned long long real_dvo, req_dvo;
	unsigned int pixclock_mhz;
	int offset;

	pixclock_mhz = pixclock_khz / 1000;
	//try precsion from high to low
	for (j = 0; j < sizeof(prec_set) / sizeof(uint32_t); j++) {
		precision_req = prec_set[j];
		//try each refc
		for (i = 0; i < sizeof(tx_clkdiv_set) / sizeof(uint32_t); i++) {
			tx_clkdiv = tx_clkdiv_set[i];
			multiplier_min = (4500 / 27) / 2;  //1200 / (PLL_REF_CLK_MHZ / refc)
			multiplier_max = (6500 / 27) / 2;  //3200 / (PLL_REF_CLK_MHZ / refc)
			multiplier_mid = (5500 / 27) / 2;  //(loopc_min + loopc_max) / 2;

			offset = 0;
			//try each loopc
			for (multiplier = multiplier_mid; (multiplier <= multiplier_max) && (multiplier >= multiplier_min); multiplier += offset) {
				if (offset < 0) {
					offset = -(offset - 1);
				} else {
					offset = -(offset + 1);
				}

				tx_rate = (2 * multiplier * 27) / tx_clkdiv / 5 / pixclock_mhz;
				if ((tx_rate != 1) && (tx_rate != 2) && (tx_rate != 4) && (tx_rate != 8)) continue;

				real_dvo = (2 * multiplier * 27) / tx_clkdiv / 5 / tx_rate;
				req_dvo  = pixclock_mhz;
				precision = abs(real_dvo * PCLK_PRECISION_INDICATOR / req_dvo - PCLK_PRECISION_INDICATOR);

				if (precision < precision_req) {
					mpllcfg.multiplier = multiplier;
					mpllcfg.tx_clkdiv = tx_clkdiv;
					mpllcfg.tx_rate   = tx_rate;
					DRM_INFO("for pixclock = %d mhz, found: multiplier = %d, "
							"tx_clkdiv = %d, tx_rate = %d, precision = %d / %d.\n", pixclock_mhz,
							multiplier, tx_clkdiv, tx_rate, precision+1, PCLK_PRECISION_INDICATOR);
					if (j > 1) {
						DRM_INFO("Warning: PIX clock precision degraded to %d / %d\n", precision_req, PCLK_PRECISION_INDICATOR);
					}

					return 1;
				}
			}
		}
	}
	return 0;
}

void ls2k3000_hdmi_pll_set(struct loonggpu_dc_crtc *crtc, int intf, int clock)
{
	if (ls2k3000_calc_phy_clk((unsigned int)(clock))) {
  		configmpll(crtc, intf);
	} else {
		DRM_INFO("\n\nError: Fail to find a proper PLL configuration.\n\n");
	}
}

bool ls2k3000_hdmi_enable(struct loonggpu_dc_crtc *crtc, int intf, bool enable)
{
	u32 hdmi_ctrl, phy_cfg0;
	struct loonggpu_device *adev = crtc->dc->adev;
	uint32_t dc_global_reg_save[6];

	if (IS_ERR_OR_NULL(adev))
		return false;

	if (intf >= MAX_DC_INTERFACES)
		return false;

	dc_global_reg_config(adev, dc_global_reg_save, 1);

	phy_cfg0 = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg0);
	if (enable && (phy_cfg0 & (0x1 << 29)))
		goto out;

	if (!enable && (!(phy_cfg0 & (0x1 << 29))))
		goto out;

	if (enable) {
		phy_cfg0 |= (0x1 << 29);
		hdmi_ctrl = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].ctrl);
                /* fixme: the hdmi ctrl reg bit 0 do not work, but it can workaround purple border issue. */
		hdmi_ctrl &= ~HDMI_CTRL_ENABLE;
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, hdmi_ctrl);
	} else {
		hdmi_ctrl = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].ctrl);
		hdmi_ctrl &= ~HDMI_CTRL_ENABLE;
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, hdmi_ctrl);
		phy_cfg0 &= ~(0x1 << 29);
	}

	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].phy_cfg0, phy_cfg0);

out:
	dc_global_reg_config(adev, dc_global_reg_save, 0);
	return true;
}

void ls2k3000_hdmi_suspend(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	crtc->hdmi_ctrl[intf] = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].ctrl);
}

int ls2k3000_hdmi_resume(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, crtc->hdmi_ctrl[intf]);
	if (crtc->hdmi_ctrl[intf] & HDMI_CTRL_AUDIO_ENABLE) {
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].zoneidle, 0x00400040);
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_ncfg, 6272);
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_ctscfg, 0x80000000);
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_infoframe, 0x15);
		dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_sample, 0x1);
	}

	return 0;
}

int ls2k3000_hdmi_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;

	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, 0x388);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].zoneidle, 0x00400040);
	return 0;
}

int ls2k3000_hdmi_audio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	u32 val;

	val = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].ctrl);
	if ((val & 0x2) && (val & 0x4))
		return 0;

	/* enable hdmi audio, but don't touch EN bit which will be
	 * update in loonggpu_hdmi_enable() */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, 0x386 | (val & 0x1));

	/* Audio N 
	 * 44.1KHz * 4, dynamic update N && CTS value */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_ncfg, 6272);

	/* Enable Send CTS */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_ctscfg, 0x80000000);

	/* Audio AIF
	 * enable AIF,set freq,and set CC = 1, CA = 0
	 * Update AIF */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_infoframe, 0x15);

	/* Audio Sample Packet */
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].audio_sample, 0x1);

	val = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].avi_ctrl);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].avi_ctrl, val | 0x5);
	return 0;
}

int ls2k3000_hdmi_noaudio_init(struct loonggpu_dc_crtc *crtc, int intf)
{
	u32 val;
	struct loonggpu_device *adev = crtc->dc->adev;

	val = dc_readl(adev, gdc_reg->hdmi_reg_v2[intf].ctrl);
	dc_writel(adev, gdc_reg->hdmi_reg_v2[intf].ctrl, 0x388 | (val & 1));

	return 0;
}
