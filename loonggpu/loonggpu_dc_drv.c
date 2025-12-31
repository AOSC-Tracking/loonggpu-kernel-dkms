#include <linux/pm_runtime.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_edid.h>

#include "loonggpu.h"
#include "loonggpu_dc_plane.h"
#include "loonggpu_dc_crtc.h"
#include "loonggpu_dc.h"
#include "loonggpu_display.h"
#include "loonggpu_dc_encoder.h"
#include "loonggpu_dc_vbios.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_hdmi.h"
#include "loonggpu_dc_dp.h"
#include "bridge_phy.h"
#include "loonggpu_helper.h"
#if defined(DRM_DRM_ATOMIC_UAPI_H_PRESENT)
#include <drm/drm_atomic_uapi.h>
#endif
#if defined(LG_DRM_DISPLAY_DRM_DP_MST_HELPER_H_PRESENT)
#include <drm/display/drm_dp_mst_helper.h>
#else
#include <drm/drm_dp_mst_helper.h>
#endif

extern resource_size_t g_vram_base;
extern resource_size_t g_vram_size;
struct dc_reg *gdc_reg;

static const struct loonggpu_display_funcs dc_display_funcs = {
	.vblank_get_counter = dc_vblank_get_counter,
	.page_flip_get_scanoutpos = dc_crtc_get_scanoutpos,
};

/* DC register offsets definition */
/* For 7A2000, 2K2000 */
struct dc_reg ls7a2000_dc_reg = {
	.global_reg = {
		0x1570, /* DI_INT_REG */
		0x1BA0, /* DI_HDMI_HOTPLUG_STATUS */
		0x1BB0, /* DI_VGA_HOTPULG_CFG */
		0x1F00, /* DI_I2C_ADDR */
		0x1660, /* DI_GPIO_CFG_OFFSET */
		0x1650, /* DI_GPIO_IN_OFFSET */
		0x1650, /* DI_GPIO_OUT_OFFSET */
	},
	.crtc_reg[0] = {
		0x1240, /* DI_CRTC_CFG_REG */
		0x1260, /* DI_CRTC_FBADDR0_LO_REG */
		0x1580, /* DI_CRTC_FBADDR1_LO_REG */
		0x15A0, /* DI_CRTC_FBADDR0_HI_REG */
		0x15C0, /* DI_CRTC_FBADDR1_HI_REG */
		0x1280, /* DI_CRTC_STRIDE_REG */
		0x1300, /* DI_CRTC_FBORIGIN_REG */
		0x1360, /* DI_CRTC_DITCFG_REG */
		0x1380, /* DI_CRTC_DITTAB_LO_REG */
		0x13A0, /* DI_CRTC_DITTAB_HI_REG */
		0x13C0, /* DI_CRTC_PANELCFG_REG */
		0x13E0, /* DI_CRTC_PANELTIM_REG */
		0x1400, /* DI_CRTC_HDISPLAY_REG */
		0x1420, /* DI_CRTC_HSYNC_REG */
		0x1480, /* DI_CRTC_VDISPLAY_REG */
		0x14A0, /* DI_CRTC_VSYNC_REG */
		0x14E0, /* DI_CRTC_GAMINDEX_REG */
		0x1500, /* DI_CRTC_GAMDATA_REG */
		0x1B80, /* DI_CRTC_SYNCDEV_REG */
		0x14C0, /* DI_CRTC_DISPLAY_POS_REG */
		0x1B00, /* DI_CRTC_META0_REG_L */
		0x1B20, /* DI_CRTC_META0_REG_H */
		0x1B40, /* DI_CRTC_META1_REG_L */
		0x1B60, /* DI_CRTC_META1_REG_H */
		0x1A00, /* DI_VSYNC_COUNTER_REG */
	},
	.crtc_reg[1] = {
		0x1250, /* DI_CRTC_CFG_REG */
		0x1270, /* DI_CRTC_FBADDR0_LO_REG */
		0x1590, /* DI_CRTC_FBADDR1_LO_REG */
		0x15B0, /* DI_CRTC_FBADDR0_HI_REG */
		0x15D0, /* DI_CRTC_FBADDR1_HI_REG */
		0x1290, /* DI_CRTC_STRIDE_REG */
		0x1310, /* DI_CRTC_FBORIGIN_REG */
		0x1370, /* DI_CRTC_DITCFG_REG */
		0x1390, /* DI_CRTC_DITTAB_LO_REG */
		0x13B0, /* DI_CRTC_DITTAB_HI_REG */
		0x13D0, /* DI_CRTC_PANELCFG_REG */
		0x13F0, /* DI_CRTC_PANELTIM_REG */
		0x1410, /* DI_CRTC_HDISPLAY_REG */
		0x1430, /* DI_CRTC_HSYNC_REG */
		0x1490, /* DI_CRTC_VDISPLAY_REG */
		0x14B0, /* DI_CRTC_VSYNC_REG */
		0x14F0, /* DI_CRTC_GAMINDEX_REG */
		0x1510, /* DI_CRTC_GAMDATA_REG */
		0x1B90, /* DI_CRTC_SYNCDEV_REG */
		0x14D0, /* DI_CRTC_DISPLAY_POS_REG */
		0x1B10, /* DI_CRTC_META0_REG_L */
		0x1B30, /* DI_CRTC_META0_REG_H */
		0x1B50, /* DI_CRTC_META1_REG_L */
		0x1B70, /* DI_CRTC_META1_REG_H */
		0x1A10, /* DI_VSYNC_COUNTER_REG */
	},
	.cursor_reg[0] = {
		0x1520, /* DI_CURSOR0_CFG_REG */
		0x1530, /* DI_CURSOR0_LADDR_REG */
		0x15E0, /* DI_CURSOR0_HADDR_REG */
		0x1540, /* DI_CURSOR0_POSITION_REG */
		0x1550, /* DI_CURSOR0_BACK_REG */
		0x1560, /* DI_CURSOR0_FORE_REG */
	},
	.cursor_reg[1] = {
		0x1670, /* DI_CURSOR1_CFG_REG */
		0x1680, /* DI_CURSOR1_LADDR_REG */
		0x16E0, /* DI_CURSOR1_HADDR_REG */
		0x1690, /* DI_CURSOR1_POSITION_REG */
		0x16A0, /* DI_CURSOR1_BACK_REG */
		0x16B0, /* DI_CURSOR1_FORE_REG */
	},
	.hdmi_reg_v1[0] = {
		0x1700, /* DI_HDMI_ZONEIDLE_REG */
		0x1720, /* DI_HDMI_CTRL_REG */
		0x1800, /* DI_HDMI_PHY_CTRL_REG */
		0x1820, /* DI_HDMI_PHY_PLLCFG_REG */
		0x1840, /* DI_HDMI_PHY_PEC0_REG */
		0x1860, /* DI_HDMI_PHY_PEC1_REG */
		0x1880, /* DI_HDMI_PHY_PEC2_REG */
		0x18A0, /* DI_HDMI_PHY_RSVR_REG */
		0x18C0, /* DI_HDMI_PHY_CALCTRL_REG */
		0x1AC0, /* DI_HDMI_PHY_CALOUT_REG */
		0x18E0, /* DI_HDMI_AVI_CONT0_REG */
		0x1900, /* DI_HDMI_AVI_CONT1_REG */
		0x1920, /* DI_HDMI_AVI_CONT2_REG */
		0x1940, /* DI_HDMI_AVI_CONT3_REG */
		0x1960, /* DI_HDMI_AVI_CTRL_REG */
		0x1980, /* DI_HDMI_VSI_CFG_REG */
		0x1740, /* DI_HDMI_AUDIO_BUF_REG */
		0x1760, /* DI_HDMI_AUDIO_NCFG_REG */
		0x1780, /* DI_HDMI_AUDIO_CTSCFG_REG */
		0x17A0, /* DI_HDMI_AUDIO_CTSCALCFG_REG */
		0x17C0, /* DI_HDMI_AUDIO_INFOFRAME_REG */
		0x17E0, /* DI_HDMI_AUDIO_SAMPLE_REG */
	},
	.hdmi_reg_v1[1] = {
		0x1710, /* DI_HDMI_ZONEIDLE_REG */
		0x1730, /* DI_HDMI_CTRL_REG */
		0x1810, /* DI_HDMI_PHY_CTRL_REG */
		0x1830, /* DI_HDMI_PHY_PLLCFG_REG */
		0x1850, /* DI_HDMI_PHY_PEC0_REG */
		0x1870, /* DI_HDMI_PHY_PEC1_REG */
		0x1890, /* DI_HDMI_PHY_PEC2_REG */
		0x18B0, /* DI_HDMI_PHY_RSVR_REG */
		0x18D0, /* DI_HDMI_PHY_CALCTRL_REG */
		0x1AD0, /* DI_HDMI_PHY_CALOUT_REG */
		0x18F0, /* DI_HDMI_AVI_CONT0_REG */
		0x1910, /* DI_HDMI_AVI_CONT1_REG */
		0x1930, /* DI_HDMI_AVI_CONT2_REG */
		0x1950, /* DI_HDMI_AVI_CONT3_REG */
		0x1970, /* DI_HDMI_AVI_CTRL_REG */
		0x1990, /* DI_HDMI_VSI_CFG_REG */
		0x1750, /* DI_HDMI_AUDIO_BUF_REG */
		0x1770, /* DI_HDMI_AUDIO_NCFG_REG */
		0x1790, /* DI_HDMI_AUDIO_CTSCFG_REG */
		0x17B0, /* DI_HDMI_AUDIO_CTSCALCFG_REG */
		0x17D0, /* DI_HDMI_AUDIO_INFOFRAME_REG */
		0x17F0, /* DI_HDMI_AUDIO_SAMPLE_REG */
	},
};

/* For 2K3000 */
struct dc_reg ls2k3000_dc_reg = {
	.global_reg = {
		0x7C, /* DI_INT_REG */
		0xD0, /* DI_HDMI_HOTPLUG_STATUS */
		0xD4, /* DI_VGA_HOTPULG_CFG */
		0x10B8, /* DI_I2C_ADDR */
		0x98, /* DI_GPIO_CFG_OFFSET */
		0x94, /* DI_GPIO_IN_OFFSET */
		0x94, /* DI_GPIO_OUT_OFFSET */
		0xdc,	/*INT_REG1*/
		0xe0,	/*INT_EN_REG*/
	},
	.crtc_reg[0] = {
		0x0, /* DI_CRTC_CFG_REG */
		0x4, /* DI_CRTC_FBADDR0_LO_REG */
		0x8, /* DI_CRTC_FBADDR1_LO_REG */
		0xC, /* DI_CRTC_FBADDR0_HI_REG */
		0x10, /* DI_CRTC_FBADDR1_HI_REG */
		0x24, /* DI_CRTC_STRIDE_REG */
		0x28, /* DI_CRTC_FBORIGIN_REG */
		0x40, /* DI_CRTC_DITCFG_REG */
		0x44, /* DI_CRTC_DITTAB_LO_REG */
		0x48, /* DI_CRTC_DITTAB_HI_REG */
		0x4C, /* DI_CRTC_PANELCFG_REG */
		0x50, /* DI_CRTC_PANELTIM_REG */
		0x54, /* DI_CRTC_HDISPLAY_REG */
		0x58, /* DI_CRTC_HSYNC_REG */
		0x68, /* DI_CRTC_VDISPLAY_REG */
		0x6C, /* DI_CRTC_VSYNC_REG */
		0x74, /* DI_CRTC_GAMINDEX_REG */
		0x78, /* DI_CRTC_GAMDATA_REG */
		0x5C, /* DI_CRTC_SYNCDEV_REG */
		0x70, /* DI_CRTC_DISPLAY_POS_REG */
		0x14, /* DI_CRTC_META0_REG_L */
		0x1C, /* DI_CRTC_META0_REG_H */
		0x18, /* DI_CRTC_META1_REG_L */
		0x20, /* DI_CRTC_META1_REG_H */
		0xD8, /* DI_VSYNC_COUNTER_REG */
	},
	.crtc_reg[1] = {
		0x400, /* DI_CRTC_CFG_REG */
		0x404, /* DI_CRTC_FBADDR0_LO_REG */
		0x408, /* DI_CRTC_FBADDR1_LO_REG */
		0x40C, /* DI_CRTC_FBADDR0_HI_REG */
		0x410, /* DI_CRTC_FBADDR1_HI_REG */
		0x424, /* DI_CRTC_STRIDE_REG */
		0x428, /* DI_CRTC_FBORIGIN_REG */
		0x440, /* DI_CRTC_DITCFG_REG */
		0x444, /* DI_CRTC_DITTAB_LO_REG */
		0x448, /* DI_CRTC_DITTAB_HI_REG */
		0x44C, /* DI_CRTC_PANELCFG_REG */
		0x450, /* DI_CRTC_PANELTIM_REG */
		0x454, /* DI_CRTC_HDISPLAY_REG */
		0x458, /* DI_CRTC_HSYNC_REG */
		0x468, /* DI_CRTC_VDISPLAY_REG */
		0x46C, /* DI_CRTC_VSYNC_REG */
		0x474, /* DI_CRTC_GAMINDEX_REG */
		0x478, /* DI_CRTC_GAMDATA_REG */
		0x45C, /* DI_CRTC_SYNCDEV_REG */
		0x470, /* DI_CRTC_DISPLAY_POS_REG */
		0x414, /* DI_CRTC_META0_REG_L */
		0x41C, /* DI_CRTC_META0_REG_H */
		0x418, /* DI_CRTC_META1_REG_L */
		0x420, /* DI_CRTC_META1_REG_H */
		0x4D8, /* DI_VSYNC_COUNTER_REG */
	},
	.cursor_reg[0] = {
		0xA0, /* DI_CURSOR0_CFG_REG */
		0xA4, /* DI_CURSOR0_LADDR_REG */
		0xA8, /* DI_CURSOR0_HADDR_REG */
		0xAC, /* DI_CURSOR0_POSITION_REG */
		0xB0, /* DI_CURSOR0_BACK_REG */
		0xB4, /* DI_CURSOR0_FORE_REG */
	},
	.cursor_reg[1] = {
		0xB8, /* DI_CURSOR1_CFG_REG */
		0xBC, /* DI_CURSOR1_LADDR_REG */
		0xC0, /* DI_CURSOR1_HADDR_REG */
		0xC4, /* DI_CURSOR1_POSITION_REG */
		0xC8, /* DI_CURSOR1_BACK_REG */
		0xCC, /* DI_CURSOR1_FORE_REG */
	},
	.hdmi_reg_v2[0] = {
		0x1000, /* DI_HDMI_ZONEIDLE_REG */
		0x1004, /* DI_HDMI_CTRL_REG */
		0x1008, /* DI_HDMI_AUDIO_BUF_REG */
		0x100C, /* DI_HDMI_AUDIO_NCFG_REG */
		0x1010, /* DI_HDMI_AUDIO_CTSCFG_REG */
		0x1014, /* DI_HDMI_AUDIO_CTSCALCFG_REG */
		0x1018, /* DI_HDMI_AUDIO_INFOFRAME_REG */
		0x101C, /* DI_HDMI_AUDIO_SAMPLE_REG */
		0x1020, /* DI_HDMI_TIMING0 */
		0x1024, /* DI_HDMI_TIMING1 */
		0x1028, /* DI_HDMI_TIMING2 */
		0x102c, /* DI_HDMI_TIMING3 */
		0x1030, /* DI_HDMI_TIMING4 */
		0x1034, /* DI_HDMI_INT */
		0x1038, /* DI_HDMI_INT_EN */
		0x1040, /* DI_HDMI_AVI_INFOFRAME0 */
		0x1044, /* DI_HDMI_AVI_INFOFRAME1 */
		0x1048, /* DI_HDMI_AVI_INFOFRAME2 */
		0x104c, /* DI_HDMI_AVI_INFOFRAME3 */
		0x1050, /* DI_HDMI_AVI_INFOFRAME_CTRL */
		0x1054, /* DI_HDMI_VENDOR_SPECIFIC_INFOFRAME */
		0x10b8, /* DI_HDMI_I2C0 */
		0x10bc, /* DI_HDMI_I2C1 */
		0x3000, /* DI_HDMI_PHY_CTRL0 */
		0x3004, /* DI_HDMI_PHY_CTRL1 */
		0x3008, /* DI_HDMI_PHY_CTRL2 */
		0x3080, /* DI_HDMI_PHY_MONITOR */
	},
	.dp_reg[0] = {
		0x2000, /*DP_LINK_CFG_0*/
		0x2004, /*DP_LINK_CFG_1*/
		0x2008, /*DP_LINK_CFG_2*/
		0x200c, /*DP_LINK_CFG_3*/
		0x2010, /*DP_LINK_CFG_4*/
		0x2014, /*DP_LINK_CFG_5*/
		0x2018, /*DP_LINK_CFG_6*/
		0x2040, /*DP_LINK_MONITOR_0*/
		0x2044, /*DP_LINK_MONITOR_1*/
		0x2048, /*DP_LINK_MONITOR_2*/
		0x204c, /*DP_LINK_MONITOR_3*/
		0x2050, /*DP_LINK_MONITOR_4*/
		0x2054, /*DP_LINK_MONITOR_5*/
		0x2058, /*DP_LINK_MONITOR_6*/
		0x205c, /*DP_LINK_MONITOR_7*/
		0x2060, /*DP_LINK_MONITOR_8*/
		0x2064, /*DP_LINK_MONITOR_9*/
		0x2068, /*DP_LINK_MONITOR_10*/
		0x206c, /*DP_LINK_MONITOR_11*/
		0x2070, /*DP_LINK_MONITOR_12*/
		0x2074, /*DP_LINK_MONITOR_13*/
		0x2078, /*DP_LINK_MONITOR_14*/
		0x207c, /*DP_LINK_MONITOR_15*/
		0x2080,	/*DP_AUX_CHANNEL_CFG_0*/
		0x2084,	/*DP_AUX_CHANNEL_CFG_1*/
		0x2088,	/*DP_AUX_CHANNEL_CFG_2*/
		0x208c,	/*DP_AUX_CHANNEL_CFG_3*/
		0x2090,	/*DP_AUX_CHANNEL_CFG_4*/
		0x2094,	/*DP_AUX_CHANNEL_CFG_5*/
		0x2098,	/*DP_AUX_CHANNEL_CFG_6*/
		0x209c,	/*DP_AUX_CHANNEL_CFG_7*/
		0x20a0,	/*DP_AUX_CHANNEL_CFG_8*/
		0x20a4,	/*DP_AUX_CHANNEL_CFG_9*/
		0x20c0,	/*DP_AUX_MONITOR_0*/
		0x20c4,	/*DP_AUX_MONITOR_1*/
		0x20c8,	/*DP_AUX_MONITOR_2*/
		0x20cc,	/*DP_AUX_MONITOR_3*/
		0x20d0,	/*DP_AUX_MONITOR_4*/
		0x20d4,	/*DP_AUX_MONITOR_5*/
		0x20d8,	/*DP_AUX_MONITOR_6*/
		0x20dc, /*DP_AUX_MONITOR_7*/
		0x2034,	/*DP_HPD_STATUS*/
		0x2038,	/*DP_HPD_ENABLE*/
		0x2100,	/*DP_SDP_CFG_0*/
		0x2104,	/*DP_SDP_CFG_1*/
		0x2108,	/*DP_SDP_CFG_2*/
		0x210c,	/*DP_SDP_CFG_3*/
		0x2110,	/*DP_SDP_CFG_4*/
		0x2114,	/*DP_SDP_CFG_5*/
		0x2118,	/*DP_SDP_CFG_6*/
		0x211c,	/*DP_SDP_CFG_7*/
		0x2120,	/*DP_SDP_CFG_8*/
		0x2124,	/*DP_SDP_CFG_9*/
		0x2128,	/*DP_SDP_CFG_10*/
		0x212c,	/*DP_SDP_CFG_11*/
		0x2130,	/*DP_SDP_CFG_12*/
		0x2134,	/*DP_SDP_CFG_13*/
		0x21f4, /*DP_SDP_CFG_61*/
		0x21f8, /*DP_SDP_CFG_62*/
		0x21fc, /*DP_SDP_CFG_63*/
		0x2200, /*DP_SDP_CFG_64*/
		0x2204, /*DP_SDP_CFG_65*/

		/* TODO: DP_SDP */
		0x4000,	/*DP_PHY_CFG_0*/
		0x4004,	/*DP_PHY_CFG_1*/
		0x4008,	/*DP_PHY_CFG_2*/
		0x400C,	/*DP_PHY_CFG_3*/
		0x4010,	/*DP_PHY_CFG_4*/
		0x4014,	/*DP_PHY_CFG_5*/
		0x4018,	/*DP_PHY_CFG_6*/
		0x401C,	/*DP_PHY_CFG_7*/
		0x4020,	/*DP_PHY_CFG_8*/
		0x4030,	/*DP_PHY_CFG_12*/
		0x4080,	/*DP_PHY_MONITOR_0*/
		0x4084,	/*DP_PHY_MONITOR_1*/
		0x4088,	/*DP_PHY_MONITOR_2*/
	},
	.dp_reg[1] = {
		0x2400, /*DP_LINK_CFG_0*/
		0x2404, /*DP_LINK_CFG_1*/
		0x2408, /*DP_LINK_CFG_2*/
		0x240c, /*DP_LINK_CFG_3*/
		0x2410, /*DP_LINK_CFG_4*/
		0x2414, /*DP_LINK_CFG_5*/
		0x2418, /*DP_LINK_CFG_6*/
		0x2440, /*DP_LINK_MONITOR_0*/
		0x2444, /*DP_LINK_MONITOR_1*/
		0x2448, /*DP_LINK_MONITOR_2*/
		0x244c, /*DP_LINK_MONITOR_3*/
		0x2450, /*DP_LINK_MONITOR_4*/
		0x2454, /*DP_LINK_MONITOR_5*/
		0x2458, /*DP_LINK_MONITOR_6*/
		0x245c, /*DP_LINK_MONITOR_7*/
		0x2460, /*DP_LINK_MONITOR_8*/
		0x2464, /*DP_LINK_MONITOR_9*/
		0x2468, /*DP_LINK_MONITOR_10*/
		0x246c, /*DP_LINK_MONITOR_11*/
		0x2470, /*DP_LINK_MONITOR_12*/
		0x2474, /*DP_LINK_MONITOR_13*/
		0x2478, /*DP_LINK_MONITOR_14*/
		0x247c, /*DP_LINK_MONITOR_15*/
		0x2480,	/*DP_AUX_CHANNEL_CFG_0*/
		0x2484,	/*DP_AUX_CHANNEL_CFG_1*/
		0x2488,	/*DP_AUX_CHANNEL_CFG_2*/
		0x248c,	/*DP_AUX_CHANNEL_CFG_3*/
		0x2490,	/*DP_AUX_CHANNEL_CFG_4*/
		0x2494,	/*DP_AUX_CHANNEL_CFG_5*/
		0x2498,	/*DP_AUX_CHANNEL_CFG_6*/
		0x249c,	/*DP_AUX_CHANNEL_CFG_7*/
		0x24a0,	/*DP_AUX_CHANNEL_CFG_8*/
		0x24a4,	/*DP_AUX_CHANNEL_CFG_9*/
		0x24c0,	/*DP_AUX_MONITOR_0*/
		0x24c4,	/*DP_AUX_MONITOR_1*/
		0x24c8,	/*DP_AUX_MONITOR_2*/
		0x24cc,	/*DP_AUX_MONITOR_3*/
		0x24d0,	/*DP_AUX_MONITOR_4*/
		0x24d4,	/*DP_AUX_MONITOR_5*/
		0x24d8,	/*DP_AUX_MONITOR_6*/
		0x24dc, /*DP_AUX_MONITOR_7*/
		0x2434,	/*DP_HPD_STATUS*/
		0x2438,	/*DP_HPD_ENABLE*/
		0x2500,	/*DP_SDP_CFG_0*/
		0x2504,	/*DP_SDP_CFG_1*/
		0x2508,	/*DP_SDP_CFG_2*/
		0x250c,	/*DP_SDP_CFG_3*/
		0x2510,	/*DP_SDP_CFG_4*/
		0x2514,	/*DP_SDP_CFG_5*/
		0x2518,	/*DP_SDP_CFG_6*/
		0x251c,	/*DP_SDP_CFG_7*/
		0x2520,	/*DP_SDP_CFG_8*/
		0x2524,	/*DP_SDP_CFG_9*/
		0x2528,	/*DP_SDP_CFG_10*/
		0x252c,	/*DP_SDP_CFG_11*/
		0x2530,	/*DP_SDP_CFG_12*/
		0x2534,	/*DP_SDP_CFG_13*/
		0x25f4, /*DP_SDP_CFG_61*/
		0x25f8, /*DP_SDP_CFG_62*/
		0x25fc, /*DP_SDP_CFG_63*/
		0x2600, /*DP_SDP_CFG_64*/
		0x2604, /*DP_SDP_CFG_65*/
		/* TODO: DP_SDP */
		0x4400,	/*DP_PHY_CFG_0*/
		0x4404,	/*DP_PHY_CFG_1*/
		0x4408,	/*DP_PHY_CFG_2*/
		0x440C,	/*DP_PHY_CFG_3*/
		0x4410,	/*DP_PHY_CFG_4*/
		0x4414,	/*DP_PHY_CFG_5*/
		0x4418,	/*DP_PHY_CFG_6*/
		0x441C,	/*DP_PHY_CFG_7*/
		0x4420,	/*DP_PHY_CFG_8*/
		0x4430,	/*DP_PHY_CFG_12*/
		0x4480,	/*DP_PHY_MONITOR_0*/
		0x4484,	/*DP_PHY_MONITOR_1*/
		0x4488,	/*DP_PHY_MONITOR_2*/
	},
};

/* For 7A1000, it has two DVOs and has no hdmi interface */
struct dc_reg ls7a1000_dc_reg = {
	.global_reg = {
		0x1570, /* DI_INT_REG */
		0x1BA0, /* DI_HDMI_HOTPLUG_STATUS */
		0x1BB0, /* DI_VGA_HOTPULG_CFG */
		0x1F00, /* DI_I2C_ADDR */
		0x1660, /* DI_GPIO_CFG_OFFSET */
		0x1650, /* DI_GPIO_IN_OFFSET */
		0x1650, /* DI_GPIO_OUT_OFFSET */
	},
	.crtc_reg[0] = {
		0x1240, /* DI_CRTC_CFG_REG */
		0x1260, /* DI_CRTC_FBADDR0_LO_REG */
		0x1580, /* DI_CRTC_FBADDR1_LO_REG */
		0x15A0, /* DI_CRTC_FBADDR0_HI_REG */
		0x15C0, /* DI_CRTC_FBADDR1_HI_REG */
		0x1280, /* DI_CRTC_STRIDE_REG */
		0x1300, /* DI_CRTC_FBORIGIN_REG */
		0x1360, /* DI_CRTC_DITCFG_REG */
		0x1380, /* DI_CRTC_DITTAB_LO_REG */
		0x13A0, /* DI_CRTC_DITTAB_HI_REG */
		0x13C0, /* DI_CRTC_PANELCFG_REG */
		0x13E0, /* DI_CRTC_PANELTIM_REG */
		0x1400, /* DI_CRTC_HDISPLAY_REG */
		0x1420, /* DI_CRTC_HSYNC_REG */
		0x1480, /* DI_CRTC_VDISPLAY_REG */
		0x14A0, /* DI_CRTC_VSYNC_REG */
		0x14E0, /* DI_CRTC_GAMINDEX_REG */
		0x1500, /* DI_CRTC_GAMDATA_REG */
		0x1B80, /* DI_CRTC_SYNCDEV_REG */
		0x14C0, /* DI_CRTC_DISPLAY_POS_REG */
		0x1B00, /* DI_CRTC_META0_REG_L */
		0x1B20, /* DI_CRTC_META0_REG_H */
		0x1B40, /* DI_CRTC_META1_REG_L */
		0x1B60, /* DI_CRTC_META1_REG_H */
		0x1A00, /* DI_VSYNC_COUNTER_REG */
	},
	.crtc_reg[1] = {
		0x1250, /* DI_CRTC_CFG_REG */
		0x1270, /* DI_CRTC_FBADDR0_LO_REG */
		0x1590, /* DI_CRTC_FBADDR1_LO_REG */
		0x15B0, /* DI_CRTC_FBADDR0_HI_REG */
		0x15D0, /* DI_CRTC_FBADDR1_HI_REG */
		0x1290, /* DI_CRTC_STRIDE_REG */
		0x1310, /* DI_CRTC_FBORIGIN_REG */
		0x1370, /* DI_CRTC_DITCFG_REG */
		0x1390, /* DI_CRTC_DITTAB_LO_REG */
		0x13B0, /* DI_CRTC_DITTAB_HI_REG */
		0x13D0, /* DI_CRTC_PANELCFG_REG */
		0x13F0, /* DI_CRTC_PANELTIM_REG */
		0x1410, /* DI_CRTC_HDISPLAY_REG */
		0x1430, /* DI_CRTC_HSYNC_REG */
		0x1490, /* DI_CRTC_VDISPLAY_REG */
		0x14B0, /* DI_CRTC_VSYNC_REG */
		0x14F0, /* DI_CRTC_GAMINDEX_REG */
		0x1510, /* DI_CRTC_GAMDATA_REG */
		0x1B90, /* DI_CRTC_SYNCDEV_REG */
		0x14D0, /* DI_CRTC_DISPLAY_POS_REG */
		0x1B10, /* DI_CRTC_META0_REG_L */
		0x1B30, /* DI_CRTC_META0_REG_H */
		0x1B50, /* DI_CRTC_META1_REG_L */
		0x1B70, /* DI_CRTC_META1_REG_H */
		0x1A10, /* DI_VSYNC_COUNTER_REG */
	},
	.cursor_reg[0] = {
		0x1520, /* DI_CURSOR0_CFG_REG */
		0x1530, /* DI_CURSOR0_LADDR_REG */
		0x15E0, /* DI_CURSOR0_HADDR_REG */
		0x1540, /* DI_CURSOR0_POSITION_REG */
		0x1550, /* DI_CURSOR0_BACK_REG */
		0x1560, /* DI_CURSOR0_FORE_REG */
	},
	.cursor_reg[1] = {
		0x1670, /* DI_CURSOR1_CFG_REG */
		0x1680, /* DI_CURSOR1_LADDR_REG */
		0x16E0, /* DI_CURSOR1_HADDR_REG */
		0x1690, /* DI_CURSOR1_POSITION_REG */
		0x16A0, /* DI_CURSOR1_BACK_REG */
		0x16B0, /* DI_CURSOR1_FORE_REG */
	},
};


u32 dc_readl(struct loonggpu_device *adev, u32 reg)
{
	return readl(adev->loongson_dc_rmmio + reg);
}

void dc_writel(struct loonggpu_device *adev, u32 reg, u32 val)
{
	writel(val, adev->loongson_dc_rmmio + reg);
}

void dc_writel_check(struct loonggpu_device *adev, u32 reg, u32 val)
{
	u32 val_reg, count = 50;

	writel(val, adev->loongson_dc_rmmio + reg);
	do {
		msleep(1);
		val_reg = readl(adev->loongson_dc_rmmio + reg);
		val_reg &= CRTC_CFG_MASK;
	} while ((val_reg != val) && (count--));
	if (val_reg != val)
		DRM_INFO("LOONGGPU DC: write reg %x failed, target value is %x, but read-back value is %x\n", reg, val, val_reg);
}

u32 dc_readl_locked(struct loonggpu_device *adev, u32 reg)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&adev->dc_mmio_lock, flags);
	val = readl(adev->loongson_dc_rmmio + reg);
	spin_unlock_irqrestore(&adev->dc_mmio_lock, flags);

	return val;
}

void dc_writel_locked(struct loonggpu_device *adev, u32 reg, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&adev->dc_mmio_lock, flags);
	writel(val, adev->loongson_dc_rmmio + reg);
	spin_unlock_irqrestore(&adev->dc_mmio_lock, flags);
}

static bool dc_links_init(struct loonggpu_dc *dc)
{
	struct header_resource *header_res = NULL;
	struct crtc_resource *crtc_resource;
	struct encoder_resource *encoder_resource;
	struct connector_resource *connector_resource;
	struct loonggpu_link_info *link_info;
	s32 links, i;

	if (IS_ERR_OR_NULL(dc))
		return false;

	header_res = dc_get_vbios_resource(dc->vbios, 0, LOONGGPU_RESOURCE_HEADER);
	links = header_res->links;

	link_info = kzalloc(sizeof(*link_info) * links, GFP_KERNEL);
	if (IS_ERR_OR_NULL(link_info))
		return false;

	dc->link_info = link_info;

	for (i = 0; i < links; i++) {
		crtc_resource = dc_get_vbios_resource(dc->vbios, i, LOONGGPU_RESOURCE_CRTC);
		link_info[i].crtc = dc_crtc_construct(dc, crtc_resource);
		if (!link_info[i].crtc) {
			DRM_ERROR("link-%d  crtc construct failed \n", i);
			continue;
		}

		encoder_resource = dc_get_vbios_resource(dc->vbios, i, LOONGGPU_RESOURCE_ENCODER);
		link_info[i].encoder = dc_encoder_construct(dc, encoder_resource);
		if (!link_info[i].encoder) {
			DRM_ERROR("link-%d  encoder construct failed \n", i);
			continue;
		}

		connector_resource = dc_get_vbios_resource(dc->vbios,
						i, LOONGGPU_RESOURCE_CONNECTOR);
		link_info[i].bridge = dc_bridge_construct(dc,
					encoder_resource, connector_resource);
		if (!link_info[i].bridge) {
			DRM_ERROR("link-%d bridge construct failed\n", i);
			continue;
		}
		link_info[i].connector = connector_resource;
		if (!link_info[i].connector) {
			DRM_ERROR("link-%d  encoder construct failed \n", i);
			continue;
		}

		link_info[i].fine = true;
		dc->links++;
	}

	return true;
}

static void dc_link_exit(struct loonggpu_dc *dc)
{
	u32 i;
	struct loonggpu_link_info *link_info;

	if (IS_ERR_OR_NULL(dc))
		return;

	link_info = dc->link_info;

	for (i = 0; i < dc->links; i++) {
		dc_crtc_destroy(link_info[i].crtc);

		link_info[i].fine = false;
	}

	kfree(link_info);
	dc->link_info = NULL;
	dc->links = 0;
}

static struct loonggpu_dc *dc_construct(struct loonggpu_device *adev)
{
	struct loonggpu_dc *dc;
	bool status;

	if (IS_ERR_OR_NULL(adev))
		return false;

	dc = kzalloc(sizeof(*dc), GFP_KERNEL);

	if (IS_ERR_OR_NULL(dc))
		return ERR_PTR(-ENOMEM);

	dc->adev = adev;

	INIT_LIST_HEAD(&dc->crtc_list);
	INIT_LIST_HEAD(&dc->encoder_list);

	status = dc_vbios_init(dc);
	if (!status) {
		kfree(dc);
		DRM_ERROR("LOONGGPU dc init vbios failed\n");
		return NULL;
	}

	if (dc_links_init(dc) == false) {
		DRM_ERROR("LOONGGPU dc init links failed\n");
		kfree(dc);
		dc = NULL;
	}


	return dc;
}

static void dc_destruct(struct loonggpu_dc *dc)
{
	if (IS_ERR_OR_NULL(dc))
		return;

	dc_link_exit(dc);
	dc_vbios_exit(dc->vbios);

	kfree(dc);
	dc = NULL;
}

bool dc_submit_timing_update(struct loonggpu_dc *dc, u32 link, struct dc_timing_info *timing)
{
	if (IS_ERR_OR_NULL(dc) || (link >= dc->links))
		return false;

	return dc->hw_ops->crtc_timing_set(dc->link_info[link].crtc, timing);
}

bool dc_submit_plane_update(struct loonggpu_dc *dc, u32 link, struct dc_plane_update *update)
{
	if (IS_ERR_OR_NULL(dc) || (link >= dc->links))
		return false;

	return dc_crtc_plane_update(dc->link_info[link].crtc, update);
}

static void manage_dc_interrupts(struct loonggpu_device *adev,
				 struct loonggpu_crtc *acrtc,
				 bool enable)
{
	if (enable) {
		drm_crtc_vblank_on(&acrtc->base);
		loonggpu_irq_get(adev, &adev->vsync_irq, acrtc->crtc_id);
	} else {
		loonggpu_irq_put(adev, &adev->vsync_irq, acrtc->crtc_id);
		drm_crtc_vblank_off(&acrtc->base);
	}
}

static int loonggpu_dc_meta_enable(struct loonggpu_device *adev, bool clear)
{
	uint64_t meta_gpu_addr;
	int r, i;

	if (adev->dc->meta_bo == NULL) {
		dev_err(adev->dev, "No VRAM object for PCIE DC_META.\n");
		return -EINVAL;
	}

	r = loonggpu_bo_reserve(adev->dc->meta_bo, false);
	if (unlikely(r != 0))
		return r;

	r = loonggpu_bo_pin(adev->dc->meta_bo, LOONGGPU_GEM_DOMAIN_VRAM);
	if (r) {
		loonggpu_bo_unreserve(adev->dc->meta_bo);
		return r;
	}

	r = loonggpu_bo_kmap(adev->dc->meta_bo, &adev->dc->meta_cpu_addr);
	if (r)
		loonggpu_bo_unpin(adev->dc->meta_bo);
	loonggpu_bo_unreserve(adev->dc->meta_bo);

	adev->dc->meta_gpu_addr = loonggpu_bo_gpu_offset(adev->dc->meta_bo);

	if (clear)
		memset(adev->dc->meta_cpu_addr, 0x00, 0x200000);

	meta_gpu_addr = adev->dc->meta_gpu_addr;
	for (i = 0; i < adev->mode_info.num_crtc; i++) {
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_l, (u32)meta_gpu_addr);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_h, (meta_gpu_addr >> 32));
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_l, (u32)meta_gpu_addr);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_h, (meta_gpu_addr >> 32));
	}

	DRM_INFO("DC META of 2M enabled (table at 0x%016llX).\n",
		 (unsigned long long)adev->dc->meta_gpu_addr);

	return 0;
}

static int loonggpu_dc_meta_disable(struct loonggpu_device *adev)
{
	int r, i;

	if (adev->dc->meta_bo == NULL) {
		dev_err(adev->dev, "No VRAM object for PCIE DC_META.\n");
		return -EINVAL;
	}

	for (i = 0; i < adev->mode_info.num_crtc; i++) {
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_l, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_h, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_l, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_h, 0);
	}

	r = loonggpu_bo_reserve(adev->dc->meta_bo, true);
	if (likely(r == 0)) {
		loonggpu_bo_kunmap(adev->dc->meta_bo);
		loonggpu_bo_unpin(adev->dc->meta_bo);
		loonggpu_bo_unreserve(adev->dc->meta_bo);
		adev->dc->meta_cpu_addr = NULL;
	}

	return 0;
}

static void loonggpu_dc_meta_set(struct loonggpu_device *adev)
{
	uint64_t meta_gpu_addr;
	int dc_meta_size = 0x200000;
	int i;
	int r;

	r = loonggpu_bo_create_kernel(adev, dc_meta_size,
				   LOONGGPU_GEM_COMPRESSED_SIZE,
				   LOONGGPU_GEM_DOMAIN_VRAM, &adev->dc->meta_bo,
				   &meta_gpu_addr, &adev->dc->meta_cpu_addr);
	if (r) {
		dev_warn(adev->dev, "(%d) create dc meta bo failed\n", r);
		return;
	}

	adev->dc->meta_gpu_addr = meta_gpu_addr;
	memset(adev->dc->meta_cpu_addr, 0x00, dc_meta_size);
	for (i = 0; i < adev->mode_info.num_crtc; i++) {
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_l, (u32)meta_gpu_addr);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_h, (meta_gpu_addr >> 32));
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_l, (u32)meta_gpu_addr);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_h, (meta_gpu_addr >> 32));
	}

	DRM_INFO("Config crtc number:%d meta addr 0x%llx\n", i, meta_gpu_addr);
}

static void loonggpu_dc_meta_free(struct loonggpu_device *adev)
{
	int i;

	loonggpu_bo_free_kernel(&adev->dc->meta_bo,
			     &adev->dc->meta_gpu_addr,
			     &adev->dc->meta_cpu_addr);

	for (i = 0; i < adev->mode_info.num_crtc; i++) {
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_l, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta0_h, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_l, 0);
		dc_writel(adev, gdc_reg->crtc_reg[i].meta1_h, 0);
	}
	adev->dc->meta_bo = NULL;

	DRM_INFO("Free crtc number:%d meta addr\n", i);
}

static int loonggpu_dc_atomic_commit(struct drm_device *dev,
				  struct drm_atomic_state *state,
				  bool nonblock)
{
	return drm_atomic_helper_commit(dev, state, nonblock);
}

static void loonggpu_output_poll_changed(struct drm_device *dev)
{
       struct loonggpu_device *adev = dev->dev_private;
       int i;

       for (i = 0; i < 2; i++) {
               struct loonggpu_dc_crtc *dc_crtc = adev->dc->link_info[i].crtc;
               dc_crtc->timing->clock = 0;
       }
       drm_fb_helper_hotplug_event(dev->fb_helper);
}

static const struct drm_mode_config_funcs loonggpu_dc_mode_funcs = {
	.fb_create = loonggpu_display_user_framebuffer_create,
	lg_drm_mode_cfg_func_set_output_poll_changed
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = loonggpu_dc_atomic_commit,
};

static bool modeset_required(struct drm_crtc_state *crtc_state)
{
	if (!drm_atomic_crtc_needs_modeset(crtc_state))
		return false;

	if (!crtc_state->enable)
		return false;

	return crtc_state->active;
}

static bool modereset_required(struct drm_crtc_state *crtc_state)
{
	if (!drm_atomic_crtc_needs_modeset(crtc_state))
		return false;

	return !crtc_state->enable || !crtc_state->active;
}

static void prepare_flip_isr(struct loonggpu_crtc *acrtc)
{
	assert_spin_locked(&acrtc->base.dev->event_lock);
	WARN_ON(acrtc->event);

	acrtc->event = acrtc->base.state->event;

	/* Set the flip status */
	acrtc->pflip_status = LOONGGPU_FLIP_SUBMITTED;

	/* Mark this event as consumed */
	acrtc->base.state->event = NULL;

	DRM_DEBUG_DRIVER("crtc:%d, pflip_stat:LOONGGPU_FLIP_SUBMITTED\n",
						 acrtc->crtc_id);
}

static void loonggpu_dc_do_flip(struct drm_crtc *crtc,
			      struct drm_framebuffer *fb,
			      uint32_t target)
{
	unsigned long flags;
	uint32_t target_vblank;
	int r, vpos, hpos;
	struct dc_plane_update plane;
	struct loonggpu_crtc *acrtc = to_loonggpu_crtc(crtc);
	struct loonggpu_framebuffer *afb = to_loonggpu_framebuffer(fb);
	struct loonggpu_bo *abo = gem_to_loonggpu_bo(fb->obj[0]);
	struct loonggpu_device *adev = crtc->dev->dev_private;
	lg_dma_resv_t *resv;
	uint64_t crtc_array_mode, crtc_address;
	int align = 64;

	/* Prepare wait for target vblank early - before the fence-waits */
	target_vblank = target - (uint32_t)drm_crtc_vblank_count(crtc) +
			loonggpu_get_vblank_counter_kms(crtc->dev, acrtc->crtc_id);

	/* TODO This might fail and hence better not used, wait
	 * explicitly on fences instead
	 * and in general should be called for
	 * blocking commit to as per framework helpers
	 */
	r = loonggpu_bo_reserve(abo, true);
	if (unlikely(r != 0)) {
		DRM_ERROR("failed to reserve buffer before flip\n");
		WARN_ON(1);
	}

	/* Wait for all fences on this FB */
	resv = to_dma_resv(abo);
	WARN_ON(lg_dma_resv_wait_timeout_rcu(resv, LG_DMA_RESV_USAGE_READ, true, false,
					     MAX_SCHEDULE_TIMEOUT) < 0);

	loonggpu_bo_unreserve(abo);

	/* Wait until we're out of the vertical blank period before the one
	 * targeted by the flip
	 */
	while ((acrtc->enabled &&
		(loonggpu_display_get_crtc_scanoutpos(adev->ddev, acrtc->crtc_id,
						    0, &vpos, &hpos, NULL,
						    NULL, &crtc->hwmode)
		 & (DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_IN_VBLANK)) ==
		(DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_IN_VBLANK) &&
		(int)(target_vblank -
		  loonggpu_get_vblank_counter_kms(adev->ddev, acrtc->crtc_id)) > 0)) {
		usleep_range(1000, 1100);
	}

	/* Flip */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	WARN_ON(acrtc->pflip_status != LOONGGPU_FLIP_NONE);

	if (acrtc->base.state->event)
		prepare_flip_isr(acrtc);

	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	crtc_array_mode = LOONGGPU_TILING_GET(abo->tiling_flags, ARRAY_MODE);

	if (crtc_array_mode == 0 && crtc->x) {
		if (!(fb->width % 64))
			align = 64;
		else if (!(fb->width % 32))
			align = 32;

		if (!(crtc->x % 64) && align >= 64)
			align = 64;
		else if (!(crtc->x % 32) && align >= 32)
			align = 32;

		if (crtc->x % 8)
			DRM_INFO("Flipping with unaligned screen x: %d\n", crtc->x);
		if (fb->width % 8)
			DRM_INFO("Flipping with unaligned fb width x: %d\n", fb->width);
	}

	switch (crtc_array_mode) {
	case 0:
		crtc_address = afb->address + crtc->y * afb->base.pitches[0] + ALIGN(crtc->x, align) * 4;
		break;
	case 2:
		crtc_address = afb->address + crtc->y * afb->base.pitches[0] + ALIGN(crtc->x, 8) * 4 * 4;
		break;
	}

	plane.type = DC_PLANE_PRIMARY;
	plane.primary.address.low_part = lower_32_bits(crtc_address);
	plane.primary.address.high_part = upper_32_bits(crtc_address);

	dc_submit_plane_update(adev->dc, acrtc->crtc_id, &plane);

	DRM_DEBUG_DRIVER("%s Flipping to hi: 0x%x, low: 0x%x \n",
			 __func__,
			 upper_32_bits(afb->address),
			 lower_32_bits(afb->address));
}

static void loonggpu_dc_commit_planes(struct drm_atomic_state *state,
				   struct drm_device *dev,
				   struct drm_crtc *pcrtc,
				   bool *wait_for_vblank)
{
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	struct loonggpu_crtc *acrtc = to_loonggpu_crtc(pcrtc);
	struct drm_crtc_state *new_pcrtc_state =
			drm_atomic_get_new_crtc_state(state, pcrtc);
	struct loonggpu_device *adev = dev->dev_private;
	struct loonggpu_dc_crtc *dc_crtc = adev->dc->link_info[acrtc->crtc_id].crtc;
	struct drm_framebuffer *modeset_fbs[1];
	int planes_count = 0;
	uint64_t tiling_flags = 0;
	unsigned long flags;
	uint32_t i;
	int x, y, cpp, ret;

	/* update planes when needed */
	for_each_oldnew_plane_in_state(state, plane, old_plane_state, new_plane_state, i) {
		struct drm_crtc *crtc = new_plane_state->crtc;
		struct drm_crtc_state *new_crtc_state;
		struct drm_framebuffer *fb = new_plane_state->fb;
		bool pflip_needed;

		if (plane->type == DRM_PLANE_TYPE_CURSOR) {
			handle_cursor_update(plane, old_plane_state);
			continue;
		}

		if (!fb || !crtc || pcrtc != crtc)
			continue;

                new_crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
                if (!new_crtc_state->active)
                        continue;

		x = plane->state->crtc->x;
		y = plane->state->crtc->y;
		pflip_needed = !state->allow_modeset;

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		if (acrtc->pflip_status != LOONGGPU_FLIP_NONE) {
			DRM_ERROR("%s: acrtc %d, already busy\n",
				  __func__, acrtc->crtc_id);
			/* In commit tail framework this cannot happen */
			WARN_ON(1);
		}
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

		if (!pflip_needed) {
			planes_count++;
			modeset_fbs[0] = fb;
		} else if (new_crtc_state->planes_changed) {
			/* Assume even ONE crtc with immediate flip means
			 * entire can't wait for VBLANK
			 * TODO Check if it's correct
			 */
			*wait_for_vblank = lg_drm_crtc_state_async_flip(
						new_pcrtc_state) ? false : true;

			/* TODO: Needs rework for multiplane flip */
			if (plane->type == DRM_PLANE_TYPE_PRIMARY)
				drm_crtc_vblank_get(crtc);

			loonggpu_dc_do_flip(crtc, fb,
				(uint32_t)drm_crtc_vblank_count(crtc) + *wait_for_vblank);
		}
	}

	if (planes_count) {
		unsigned long flags;
		struct dc_timing_info timing;
		struct dc_plane_update plane;
		struct drm_display_mode *mode = &pcrtc->mode;
		struct loonggpu_bridge_phy *bridge = NULL;
		struct drm_framebuffer *drm_fb;
		int align = 64;
		uint64_t address;

		struct loonggpu_framebuffer *afb =
				to_loonggpu_framebuffer(modeset_fbs[0]);

		struct loonggpu_bo *bo = gem_to_loonggpu_bo(afb->base.obj[0]);
		ret = loonggpu_bo_reserve(bo, false);
		if (unlikely(ret)) {
			if (ret != -ERESTARTSYS)
				DRM_ERROR("Unable to reserve buffer: %d\n", ret);
			return;
		}
		loonggpu_bo_get_tiling_flags(bo, &tiling_flags);
		loonggpu_bo_unreserve(bo);
		dc_crtc->array_mode = LOONGGPU_TILING_GET(tiling_flags, ARRAY_MODE);

		if (new_pcrtc_state->event) {
			drm_crtc_vblank_get(pcrtc);
			spin_lock_irqsave(&pcrtc->dev->event_lock, flags);
			prepare_flip_isr(acrtc);
			spin_unlock_irqrestore(&pcrtc->dev->event_lock, flags);
		}

		cpp = afb->base.format->cpp[0];
		timing.depth = afb->base.format->cpp[0] << 3;
		timing.stride = afb->base.pitches[0];
		timing.clock = mode->clock;
		timing.hdisplay = mode->hdisplay;
		timing.htotal = mode->htotal;
		timing.hsync_start = mode->hsync_start;
		timing.hsync_end  = mode->hsync_end;
		timing.vdisplay = mode->vdisplay;
		timing.vtotal = mode->vtotal;
		timing.vsync_start = mode->vsync_start;
		timing.vsync_end = mode->vsync_end;
		timing.use_dma = 0;


		/* The width of FB is used to calculate the DMA length when
		 * the screen is rotated */
		mutex_lock(&dev->mode_config.fb_lock);
		drm_for_each_fb(drm_fb, dev) {
			struct loonggpu_framebuffer *lfb = to_loonggpu_framebuffer(drm_fb);
			if (drm_fb->width < 480 || !strcmp(drm_fb->comm, "fbcon"))
				continue;
			if (x != 0 && (lfb != afb) && dc_crtc->array_mode == 0) {
				if (!(drm_fb->width % 64)) {
					align = 64;
					timing.use_dma = CRTC_CFG_DMA_256;
				} else if (!(drm_fb->width % 32)) {
					align = 32;
					timing.use_dma = CRTC_CFG_DMA_128;
				} else
					DRM_INFO("Setting with unaligned fb width x: %d\n", drm_fb->width);
			}
		}
		mutex_unlock(&dev->mode_config.fb_lock);

		/* x is used to calculate the DMA length when the dual screen
		 * is arranged horizontally */
		if (x != 0 && dc_crtc->array_mode == 0) {
			if (!(x % 64) && align >= 64) {
				align = 64;
				timing.use_dma = CRTC_CFG_DMA_256;
			} else if (!(x % 32) && align >= 32) {
				align = 32;
				timing.use_dma = CRTC_CFG_DMA_128;
			} else
				DRM_INFO("Setting with unaligned screen x: %d\n", x);
		}

		DRM_DEBUG_DRIVER("loonggpu crtc-%d hdisplay %d vdisplay %d x %d y %d cpp %d stride %d\n",
				 acrtc->crtc_id, timing.hdisplay, timing.vdisplay,
				 x, y, cpp, timing.stride);

		plane.type = DC_PLANE_PRIMARY;
		switch (dc_crtc->array_mode) {
		case 0:
			address = afb->address + y * timing.stride + ALIGN(x, align) * cpp;
			break;
		case 2:
			y = (y + 3) & ~3;
			x = ALIGN(x, 8);
			address = afb->address + y * timing.stride + x * cpp * 4;
			break;
		}

		plane.primary.address.low_part = lower_32_bits(address);
		plane.primary.address.high_part = upper_32_bits(address);
		dc_submit_plane_update(adev->dc, acrtc->crtc_id, &plane);

		dc_submit_timing_update(adev->dc, acrtc->crtc_id, &timing);

		bridge = adev->mode_info.encoders[acrtc->crtc_id]->bridge;
		if (bridge)
			bridge_phy_mode_set(bridge, mode, NULL);

	}
}

static void loonggpu_dc_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;
	struct loonggpu_device *adev = dev->dev_private;
	struct loonggpu_dc *dc = adev->dc;
	uint32_t i, j;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	unsigned long flags;
	bool wait_for_vblank = true;
	int crtc_disable_count = 0;
	struct loonggpu_dc_crtc *dc_crtc;

	/*
	 * We evade vblanks and pflips on crtc that
	 * should be changed. We do it here to flush & disable
	 * interrupts before drm_swap_state is called in drm_atomic_helper_commit
	 * it will update crtc->dm_crtc_state->stream pointer which is used in
	 * the ISRs.
	 */
	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		struct loonggpu_crtc *acrtc = to_loonggpu_crtc(crtc);
		dc_crtc = adev->dc->link_info[acrtc->crtc_id].crtc;

		if (old_crtc_state->active && !new_crtc_state->active) {
			if (drm_atomic_crtc_needs_modeset(new_crtc_state)) {
				dc_crtc->timing->clock = 0;
				dc->hw_ops->crtc_enable(dc_crtc, false);
				dc_interface_enable(dc_crtc, false);
			}
		}

		if (drm_atomic_crtc_needs_modeset(new_crtc_state)) {
			manage_dc_interrupts(adev, acrtc, false);
		}
	}

	drm_atomic_helper_update_legacy_modeset_state(dev, state);

	/* update changed items */
	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		struct loonggpu_crtc *acrtc = to_loonggpu_crtc(crtc);

		DRM_DEBUG_DRIVER(
			"loonggpu_crtc id:%d crtc_state_flags: enable:%d, active:%d, "
			"planes_changed:%d, mode_changed:%d,active_changed:%d,"
			"connectors_changed:%d\n",
			acrtc->crtc_id,
			new_crtc_state->enable,
			new_crtc_state->active,
			new_crtc_state->planes_changed,
			new_crtc_state->mode_changed,
			new_crtc_state->active_changed,
			new_crtc_state->connectors_changed);

		/* handles headless hotplug case, updating new_state and
		 * aconnector as needed
		 */
		if (modeset_required(new_crtc_state)) {
			DRM_DEBUG_DRIVER("Atomic commit: SET crtc id %d: [%p]\n", acrtc->crtc_id, acrtc);
			pm_runtime_get_noresume(dev->dev);
			acrtc->enabled = true;
			acrtc->hw_mode = new_crtc_state->mode;
			crtc->hwmode = new_crtc_state->mode;
		} else if (modereset_required(new_crtc_state)) {
			DRM_DEBUG_DRIVER("loonggpu_crtc id:%d enable:%d, active:%d\n", acrtc->crtc_id,
				new_crtc_state->enable,	new_crtc_state->active);
			acrtc->enabled = false;
		}
	} /* for_each_crtc_in_state() */

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		/*
		 * loop to enable interrupts on newly arrived crtc
		 */
		struct loonggpu_crtc *acrtc = to_loonggpu_crtc(crtc);
		bool modeset_needed;

		if (old_crtc_state->active && !new_crtc_state->active)
			crtc_disable_count++;

		modeset_needed = modeset_required(new_crtc_state);

		if (!modeset_needed)
			continue;

		manage_dc_interrupts(adev, acrtc, true);
	}

	/* update planes when needed per crtc */
	for_each_new_crtc_in_state(state, crtc, new_crtc_state, j) {
		struct loonggpu_crtc *acrtc = to_loonggpu_crtc(crtc);
		dc_crtc = adev->dc->link_info[acrtc->crtc_id].crtc;

		if (!new_crtc_state->active) {
			dc_crtc->timing->clock = 0;
			dc->hw_ops->crtc_enable(dc_crtc, false);
			dc_interface_enable(dc_crtc, false);
		} else {
			dc->hw_ops->crtc_enable(dc_crtc, true);
			dc_interface_enable(dc_crtc, true);
		}
		loonggpu_dc_commit_planes(state, dev, crtc, &wait_for_vblank);
	}

	/*
	 * send vblank event on all events not handled in flip and
	 * mark consumed event for drm_atomic_helper_commit_hw_done
	 */
	spin_lock_irqsave(&adev->ddev->event_lock, flags);
	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {

		if (new_crtc_state->event)
			drm_send_event_locked(dev, &new_crtc_state->event->base);

		new_crtc_state->event = NULL;
	}
	spin_unlock_irqrestore(&adev->ddev->event_lock, flags);

	drm_atomic_helper_commit_hw_done(state);

	if (wait_for_vblank)
		drm_atomic_helper_wait_for_flip_done(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);

	/* Finally, drop a runtime PM reference for each newly disabled CRTC,
	 * so we can put the GPU into runtime suspend if we're not driving any
	 * displays anymore
	 */
	for (i = 0; i < crtc_disable_count; i++)
		pm_runtime_put_autosuspend(dev->dev);
	pm_runtime_mark_last_busy(dev->dev);
}

static struct drm_mode_config_helper_funcs loonggpu_dc_mode_config_helper = {
	.atomic_commit_tail = loonggpu_dc_atomic_commit_tail
};

static int dc_mode_config_init(struct loonggpu_device *adev)
{
	int r;

	adev->mode_info.mode_config_initialized = true;

	adev->ddev->mode_config.funcs = (void *)&loonggpu_dc_mode_funcs;
	adev->ddev->mode_config.helper_private = &loonggpu_dc_mode_config_helper;

	adev->ddev->mode_config.max_width = 16384;
	adev->ddev->mode_config.max_height = 16384;

	adev->ddev->mode_config.preferred_depth = 24;
	adev->ddev->mode_config.prefer_shadow = 1;
	/* indicate support of immediate flip */
	adev->ddev->mode_config.async_page_flip = true;

	lg_dc_mode_config_init_fb_base(adev);

	r = loonggpu_display_modeset_create_props(adev);
	if (r)
		return r;

	return 0;
}

static void loonggpu_attach_encoder_connector(struct loonggpu_device *adev)
{
	struct loonggpu_connector *lconnector;
	struct loonggpu_encoder *lencoder;
	int i;

	for (i = 0; i < adev->mode_info.num_crtc; i++) {
		lconnector = adev->mode_info.connectors[i];
		lencoder = adev->mode_info.encoders[i];
		drm_connector_attach_encoder(&lconnector->base, &lencoder->base);
	}
}

static void loonggpu_display_print_display_setup(struct drm_device *dev)
{
	struct loonggpu_device *adev = dev->dev_private;
	struct drm_crtc *crtc;
	struct loonggpu_crtc *loonggpu_crtc;
	struct drm_connector *connector;
	struct loonggpu_connector *loonggpu_connector;
	struct drm_encoder *encoder;
	struct loonggpu_encoder *loonggpu_encoder;

	DRM_DEBUG_DRIVER("LOONGGPU DC revision %d\n", adev->dc_revision);
	DRM_INFO("LOONGGPU Display Crtcs\n");
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		loonggpu_crtc = to_loonggpu_crtc(crtc);
		DRM_INFO("Crtc %d: name:%s\n", crtc->index, crtc->name);
	}

	DRM_INFO("LOONGGPU Display Connectors\n");
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		loonggpu_connector = to_loonggpu_connector(connector);
		DRM_INFO("Connector %d: name:%s\n", connector->index, connector->name);
	}

	DRM_INFO("LOONGGPU Display Encoders\n");
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		loonggpu_encoder = to_loonggpu_encoder(encoder);
		DRM_INFO("Encoder %d: name:%s\n", encoder->index, encoder->name);
	}
}

static int dc_initialize_drm_device(struct loonggpu_device *adev)
{
	struct loonggpu_mode_info *mode_info = &adev->mode_info;
	int32_t i;
	s32 links = adev->dc->links;

	if (dc_mode_config_init(adev)) {
		DRM_ERROR("Failed to initialize mode config\n");
		return -1;
	}

	for (i = (links - 1); i >= 0; i--) {
		if (initialize_plane(adev, mode_info, i)) {
			DRM_ERROR("KMS: Failed to initialize primary plane\n");
			goto fail;
		}
	}

	for (i = 0; i < links; i++) {
		if (loonggpu_dc_crtc_init(adev, &mode_info->planes[i]->base, i)) {
			DRM_ERROR("KMS: Failed to initialize crtc\n");
			goto fail;
		}
	}

	for (i = 0; i < links; i++) {
		adev->dc->hw_ops->dc_i2c_init(adev, i);
		if (loonggpu_dc_encoder_init(adev, i)) {
			DRM_ERROR("KMS: Failed to initialize encoder\n");
			goto fail;
		}

		if (loonggpu_dc_bridge_init(adev, i)) {
			DRM_ERROR("KMS: Failed to initialize bridge\n");
			goto fail;
		}
	}

	loonggpu_attach_encoder_connector(adev);

	return 0;

fail:
	for (i = 0; i < 2/*max_planes*/; i++)
		kfree(mode_info->planes[i]);
	return -1;
}

static void loonggpu_dc_fini(struct loonggpu_device *adev)
{
	drm_mode_config_cleanup(adev->ddev);
	/*
	 * TODO: pageflip, vlank interrupt
	 *
	 * loonggpu_dc_irq_fini(adev);
	 */

	return;
}

#define DC_MAX_PLANES 4
static const enum drm_plane_type plane_type[DC_MAX_PLANES] = {
	DRM_PLANE_TYPE_PRIMARY,
	DRM_PLANE_TYPE_PRIMARY,
};

static int dc_early_init(void *handle)
{
	struct loonggpu_device *adev = (struct loonggpu_device *)handle;

	adev->mode_info.num_crtc = 2;
	adev->mode_info.num_i2c = 2;
	adev->mode_info.num_hpd = 3;
	adev->mode_info.plane_type = plane_type;

	if (adev->mode_info.funcs == NULL)
		adev->mode_info.funcs = &dc_display_funcs;

	return 0;
}

static int dc_late_init(void *handle)
{
	return 0;
}

static void ls7a2000_dc_topology_init(struct loonggpu_dc_crtc *crtc)
{
	crtc->cursor = crtc->resource->base.link;
	crtc->intf[0].type = INTERFACE_HDMI;
	crtc->intf[0].index = crtc->resource->base.link;
	crtc->interfaces = 1;
}

static void ls7a1000_dc_topology_init(struct loonggpu_dc_crtc *crtc)
{
	crtc->cursor = crtc->resource->base.link;
	crtc->intf[0].type = INTERFACE_DVO;
	crtc->intf[0].index = crtc->resource->base.link;
	crtc->interfaces = 1;
}

static void ls2k2000_dc_topology_init(struct loonggpu_dc_crtc *crtc)
{
	crtc->cursor = crtc->resource->base.link;
	crtc->interfaces = 1;
	crtc->intf[0].index = 0;

	switch (crtc->resource->base.link) {
	case 0:
		crtc->intf[0].type = INTERFACE_HDMI;
		break;
	case 1:
		crtc->intf[0].type = INTERFACE_DVO;
		break;
	}
}

static void ls2k3000_dc_topology_init(struct loonggpu_dc_crtc *crtc)
{
	crtc->cursor = crtc->resource->base.link;
	switch (crtc->resource->base.link) {
	case 0:
		crtc->intf[0].index = crtc->resource->base.link;
		crtc->intf[0].type = INTERFACE_EDP;
		crtc->interfaces = 1;
		break;
	case 1:
		crtc->intf[0].index = 0;
		crtc->intf[0].type = INTERFACE_HDMI;
		crtc->intf[1].index = crtc->resource->base.link;
		crtc->intf[1].type = INTERFACE_DP;
		crtc->interfaces = 2;
		break;
	}
}

static u32 loonggpu_dc_vblank_get_counter(struct loonggpu_device *adev, int crtc_num)
{
	return dc_readl(adev, gdc_reg->crtc_reg[crtc_num].vsync_counter);
}

static u32 ls7a1000_dc_vblank_get_counter(struct loonggpu_device *adev, int crtc_num)
{
	return drm_crtc_vblank_count(&adev->mode_info.crtcs[crtc_num]->base);
}

static const struct dc_hw_ops ls7a2000_dc_hw_ops = {
	.dc_pll_set = ls7a2000_dc_pll_set,
	.dc_irq_hw_init = dc_irq_hw_init,
	.dc_irq_hw_uninit = dc_irq_hw_uninit,
	.dc_irq_fini = dc_irq_fini,
	.dc_i2c_init = ls7a2000_dc_i2c_init,
	.dc_hpd_enable = ls7a2000_dc_hpd_enable,

	.crtc_enable = dc_crtc_enable,
	.crtc_timing_set = dc_crtc_timing_set,

	.cursor_move = dc_cursor_move,
	.cursor_set = dc_cursor_set,

	.hdmi_pll_set = ls7a2000_hdmi_pll_set,
	.hdmi_enable = ls7a2000_hdmi_enable,
	.hdmi_init = ls7a2000_hdmi_init,
	.hdmi_audio_init = ls7a2000_hdmi_audio_init,
	.hdmi_noaudio_init = ls7a2000_hdmi_noaudio_init,
	.hdmi_resume = ls7a2000_hdmi_resume,
	.hdmi_suspend = ls7a2000_hdmi_suspend,
	.hdmi_i2c_set = ls7a2000_hdmi_i2c_set,
	.dc_hpd_ack = dc_hpd_ack,
	.dc_i2c_ack = dc_i2c_ack,
	.dc_crtc_vblank_ack = dc_crtc_vblank_ack,
	.dc_vblank_get_counter = loonggpu_dc_vblank_get_counter,
};

static const struct dc_hw_ops ls7a1000_dc_hw_ops = {
	.dc_pll_set = ls7a2000_dc_pll_set,
	.dc_irq_hw_init = dc_irq_hw_init,
	.dc_irq_hw_uninit = dc_irq_hw_uninit,
	.dc_irq_fini = dc_irq_fini,
	.dc_i2c_init = ls7a1000_dc_i2c_init,
	.dc_hpd_enable = ls7a2000_dc_hpd_enable,

	.crtc_enable = dc_crtc_enable,
	.crtc_timing_set = dc_crtc_timing_set,

	.cursor_move = dc_cursor_move,
	.cursor_set = dc_cursor_set,

	/* 7a1000 has two DVOs and has no hdmi interface */
	.hdmi_i2c_set = ls7a1000_hdmi_i2c_set,
	.dc_hpd_ack = ls7a1000_dc_hpd_ack,
	.dc_i2c_ack = ls7a1000_dc_i2c_ack,
	.dc_crtc_vblank_ack = ls7a1000_dc_crtc_vblank_ack,
	.dc_vblank_get_counter = ls7a1000_dc_vblank_get_counter,
};

static const struct dc_sw_ops ls7a2000_dc_sw_ops = {
	.dc_topology_init = ls7a2000_dc_topology_init,
	.dc_irq_sw_init = dc_irq_sw_init,
};

static const struct dc_sw_ops ls7a1000_dc_sw_ops = {
	.dc_topology_init = ls7a1000_dc_topology_init,
	.dc_irq_sw_init = dc_irq_sw_init,
};

static const struct dc_hw_ops ls2k2000_dc_hw_ops = {
	.dc_pll_set = ls7a2000_dc_pll_set,
	.dc_irq_hw_init = dc_irq_hw_init,
	.dc_irq_hw_uninit = dc_irq_hw_uninit,
	.dc_irq_fini = dc_irq_fini,
	.dc_i2c_init = ls7a2000_dc_i2c_init,
	.dc_hpd_enable = ls7a2000_dc_hpd_enable,

	.crtc_enable = dc_crtc_enable,
	.crtc_timing_set = dc_crtc_timing_set,
	.crtc_cfg_adjust = ls2k2000_dc_crtc_cfg_adjust,

	.cursor_move = dc_cursor_move,
	.cursor_set = dc_cursor_set,

	.hdmi_pll_set = ls7a2000_hdmi_pll_set,
	.hdmi_enable = ls7a2000_hdmi_enable,
	.hdmi_init = ls7a2000_hdmi_init,
	.hdmi_audio_init = ls7a2000_hdmi_audio_init,
	.hdmi_noaudio_init = ls7a2000_hdmi_noaudio_init,
	.hdmi_resume = ls7a2000_hdmi_resume,
	.hdmi_suspend = ls7a2000_hdmi_suspend,
	.hdmi_i2c_set = ls7a2000_hdmi_i2c_set,
	.dc_hpd_ack = dc_hpd_ack,
	.dc_i2c_ack = dc_i2c_ack,
	.dc_crtc_vblank_ack = dc_crtc_vblank_ack,
	.dc_vblank_get_counter = loonggpu_dc_vblank_get_counter,
};

static const struct dc_sw_ops ls2k2000_dc_sw_ops = {
	.dc_topology_init = ls2k2000_dc_topology_init,
	.dc_irq_sw_init = dc_irq_sw_init,
};

static const struct dc_hw_ops ls2k3000_dc_hw_ops = {
	.dc_pll_set = ls2k3000_dc_pll_set,
	.dc_irq_hw_init = dc_irq_hw_init,
	.dc_irq_hw_uninit = dc_irq_hw_uninit,
	.dc_irq_fini = dc_irq_fini,
	.dc_i2c_init = ls2k3000_dc_i2c_init,
	.dc_i2c_resume = ls2k3000_dc_i2c_resume,
	.dc_hpd_enable = ls2k3000_dc_hpd_enable,

	.crtc_enable = dc_crtc_enable,
	.crtc_timing_set = dc_crtc_timing_set,

	.cursor_move = dc_cursor_move,
	.cursor_set = dc_cursor_set,

	.hdmi_pll_set = ls2k3000_hdmi_pll_set,
	.hdmi_enable = ls2k3000_hdmi_enable,
	.hdmi_init = ls2k3000_hdmi_init,
	.hdmi_audio_init = ls2k3000_hdmi_audio_init,
	.hdmi_noaudio_init = ls2k3000_hdmi_noaudio_init,
	.hdmi_resume = ls2k3000_hdmi_resume,
	.hdmi_suspend = ls2k3000_hdmi_suspend,
	.hdmi_i2c_set = ls2k3000_hdmi_i2c_set,

	.first_hpd_detect = l2k3000_dp_first_hdp_detect,
	.dp_hpd_handler = l2k3000_hpd_irq_handler,
	.dp_pll_set = ls2k3000_dp_pll_set,
	.dp_enable = ls2k3000_dp_enable,
	.dp_init = ls2k3000_dp_init,
	.dp_audio_init = ls2k3000_dp_audio_init,
	.dp_resume = ls2k3000_dp_resume,
	.dp_noaudio_init = ls2k3000_dp_noaudio_init,
	.dp_suspend = ls2k3000_dp_suspend,
	.dc_hpd_ack = dc_hpd_ack,
	.dc_i2c_ack = dc_i2c_ack,
	.dc_crtc_vblank_ack = dc_crtc_vblank_ack,
	.dc_vblank_get_counter = loonggpu_dc_vblank_get_counter,
};

static const struct dc_sw_ops ls2k3000_dc_sw_ops = {
	.dc_topology_init = ls2k3000_dc_topology_init,
	.dc_irq_sw_init = dc_irq_sw_init,
};

void dc_set_dma_consistent(struct loonggpu_device *adev)
{
	int r;
	int dma_bits;

	/* set DMA mask + need_dma32 flags.
	 * PCIE - can handle 40-bits.
	 * PCI - dma32 for legacy pci gart, 40 bits on newer asics
	 */
	adev->need_dma32 = false;
	dma_bits = 40;
	r = lg_pci_set_dma_mask(adev->pdev, &adev->pdev->dev, DMA_BIT_MASK(dma_bits));
	if (r) {
		adev->need_dma32 = true;
		dma_bits = 32;
		pr_warn("loonggpu: No suitable DMA available\n");
	}
	r = lg_pci_set_consistent_dma_mask(adev->pdev, &adev->pdev->dev, DMA_BIT_MASK(dma_bits));
	if (r) {
		lg_pci_set_consistent_dma_mask(adev->pdev, &adev->pdev->dev, DMA_BIT_MASK(32));
		pr_warn("loonggpu: No coherent DMA available\n");
	}

	adev->gmc.dma_bits = dma_bits;
}

static int dc_sw_init(void *handle)
{
	struct loonggpu_device *adev = (struct loonggpu_device *)handle;
	const struct dc_sw_ops * sw_ops;
	u64 base = 0;
	int i, r;
	unsigned long size = 0;

	if (adev->family_type == CHIP_NO_GPU) {
		dc_set_dma_consistent(adev);

		adev->need_swiotlb = lg_drm_need_swiotlb(adev->gmc.dma_bits);

		if (g_vram_base && g_vram_size) {
			adev->gmc.aper_base = g_vram_base;
			adev->gmc.aper_size = g_vram_size;
			base = adev->gmc.aper_base & 0xffffffffff;
		} else {
			adev->gmc.aper_base = (unsigned long)loonggpu_get_vram_info(adev->dev, &size);
			if (!adev->gmc.aper_base)
				goto error;
			base = adev->gmc.aper_base;
			adev->gmc.aper_size = size;
		}

		DRM_INFO("VRAM START:%llx!\n", (u64)adev->gmc.aper_base);

		adev->gmc.mc_vram_size = adev->gmc.aper_size;
		adev->gmc.real_vram_size = adev->gmc.aper_size;
		adev->gmc.visible_vram_size = adev->gmc.aper_size;

		loonggpu_device_vram_location(adev, &adev->gmc, base);

		/* Memory manager via ttm*/
		r = loonggpu_bo_init(adev);
		if (r) {
			DRM_INFO("bo init failed!\n");
			goto error;
		}
		/* Adjust VM size here.
		 * Currently set to 4GB ((1 << 20) 4k pages).
		 * Max GPUVM size for cayman and SI is 40 bits.
		 */
		loonggpu_vm_adjust_size(adev, 64, 2, 40);

		/*
		 * number of VMs
		 * VMID 0 is reserved for System
		 * loonggpu graphics/compute will use VMIDs 1-7
		 */
		adev->vm_manager.id_mgr.num_ids = 4;
		adev->vm_manager.first_kcd_vmid = 4;
		loonggpu_vm_manager_init(adev);

		adev->vm_manager.vram_base_offset = adev->gmc.aper_base;

		loonggpu_hw_sema_mgr_init(adev);
	}

	adev->dc = dc_construct(adev);
	if (adev->dc) {
		DRM_INFO("LOONGGPU Display Core initialized with v%s!\n", DC_VER);
	} else {
		DRM_INFO("LOONGGPU Display Core failed to init with v%s!\n", DC_VER);
		goto error;
	}

	if (adev->chip == dev_7a2000)
		adev->dc->sw_ops = &ls7a2000_dc_sw_ops;
	else if (adev->chip == dev_2k2000)
		adev->dc->sw_ops = &ls2k2000_dc_sw_ops;
	else if (adev->chip == dev_2k3000)
		adev->dc->sw_ops = &ls2k3000_dc_sw_ops;
	else if (adev->chip == dev_7a1000) {
		adev->dc->sw_ops = &ls7a1000_dc_sw_ops;
	}
	sw_ops = adev->dc->sw_ops;
	for (i = 0; i < adev->dc->links; i++) {
		sw_ops->dc_topology_init(adev->dc->link_info[i].crtc);
	}
	DRM_INFO("LOONGGPU DC construct links:%d", adev->dc->links);

	adev->mode_info.num_crtc = adev->dc->links;
	DRM_INFO("LOONGGPU DC construct links:%d", adev->dc->links);
	DRM_DEBUG_DRIVER("LOONGGPU DC sw init success!\n");

	return 0;

error:
	loonggpu_dc_fini(adev);
	return -1;
}

static int dc_sw_fini(void *handle)
{
	return 0;
}

static int dc_hw_init(void *handle)
{
	struct loonggpu_device *adev = (struct loonggpu_device *)handle;
	int i;

	adev->ddev->mode_config.cursor_width = 64;
	adev->ddev->mode_config.cursor_height = 64;

	if (adev->chip == dev_7a2000)
		adev->dc->hw_ops = &ls7a2000_dc_hw_ops;
	else if (adev->chip == dev_2k2000)
		adev->dc->hw_ops = &ls2k2000_dc_hw_ops;
	else if (adev->chip == dev_2k3000)
		adev->dc->hw_ops = &ls2k3000_dc_hw_ops;
	else if (adev->chip == dev_7a1000) {
		adev->dc->hw_ops = &ls7a1000_dc_hw_ops;
		adev->ddev->mode_config.cursor_width = 32;
		adev->ddev->mode_config.cursor_height = 32;
	}

	for (i = 0; i < adev->dc->links; i++) {
		dc_interface_init(adev->dc->link_info[i].crtc);
	}

	if (dc_initialize_drm_device(adev)) {
		DRM_ERROR("Failed to initialize sw for display support.\n");
		goto error;
	}

	if (drm_vblank_init(adev->ddev, adev->dc->links)) {
		DRM_ERROR("Failed to initialize vblank.\n");
		goto error;
	}

	if (dc_irq_sw_init(adev)) {
		DRM_ERROR("Failed to initialize IRQ support.\n");
		goto error;
	}

	loonggpu_display_print_display_setup(adev->ddev);

	drm_mode_config_reset(adev->ddev);
	drm_kms_helper_poll_init(adev->ddev);
	loonggpu_dc_meta_set(adev);
	adev->dc->hw_ops->dc_irq_hw_init(adev);

	DRM_DEBUG_DRIVER("LOONGGPU DC hw init success!\n");

	return 0;
error:
	loonggpu_dc_fini(adev);
	return -1;
}

static int dc_hw_fini(void *handle)
{
	struct loonggpu_device *adev = (struct loonggpu_device *)handle;

	dc_irq_fini(adev);
	loonggpu_dc_meta_free(adev);
	loonggpu_dc_fini(adev);
	return 0;
}

static void loonggpu_dc_crtc_suspend(struct loonggpu_dc *dc)
{
	struct loonggpu_dc_crtc *dc_crtc;

	list_for_each_entry(dc_crtc, &dc->crtc_list, node) {
		memset(dc_crtc->timing, 0, sizeof(struct dc_timing_info));
	}
}

static int dc_suspend(void *handle)
{
	struct loonggpu_device *adev = handle;
	struct loonggpu_dc *dc = adev->dc;
	struct loonggpu_dc_crtc *dc_crtc;

	WARN_ON(adev->dc->cached_state);

	adev->dc->cached_state = drm_atomic_helper_suspend(adev->ddev);

	list_for_each_entry(dc_crtc, &dc->crtc_list, node) {
		dc_interface_suspend(dc_crtc);
	}
	loonggpu_dc_meta_disable(adev);
	dc_irq_hw_uninit(adev);
	loonggpu_bridge_suspend(adev);
	loonggpu_dc_crtc_suspend(adev->dc);

	return 0;
}

static int dc_resume(void *handle)
{
	struct loonggpu_device *adev = handle;
	struct drm_device *ddev = adev->ddev;
	struct loonggpu_dc *dc = adev->dc;
	struct loonggpu_dc_crtc *dc_crtc;
	struct drm_crtc *crtc;
	struct drm_plane *plane;
	struct drm_crtc_state *new_crtc_state;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	struct drm_framebuffer *fb;
	struct loonggpu_framebuffer *afb;
	struct loonggpu_bo *abo;
	void *kptr;
	int i;

	for_each_new_crtc_in_state(dc->cached_state, crtc, new_crtc_state, i)
		new_crtc_state->active_changed = true;

	for_each_oldnew_plane_in_state(dc->cached_state, plane, old_plane_state, new_plane_state, i) {
		fb = new_plane_state->fb;
		if (!fb)
			continue;
		afb = to_loonggpu_framebuffer(fb);
		if (!afb)
			continue;
		abo = gem_to_loonggpu_bo(fb->obj[0]);
		if (!abo)
			continue;
		kptr = loonggpu_bo_kptr(abo);
		if (!kptr)
			continue;
		memset(kptr, 0, afb->base.height * afb->base.pitches[0]);
	}

	for (i = 0; i < dc->links; i++) {
		dc_crtc = dc->link_info[i].crtc;
		dc_interface_resume(dc_crtc);
	}

	drm_atomic_helper_resume(ddev, dc->cached_state);

	dc->cached_state = NULL;

	loonggpu_dc_meta_enable(adev, true);
	dc_irq_hw_init(adev);
	loonggpu_bridge_resume(adev);

	return 0;
}

static bool dc_is_idle(void *handle)
{
	return true;
}

static int dc_wait_for_idle(void *handle)
{
	return 0;
}

static bool dc_check_soft_reset(void *handle)
{
	return false;
}

static int dc_soft_reset(void *handle)
{
	return 0;
}

static const struct loonggpu_ip_funcs loonggpu_dc_funcs = {
	.name = "display",
	.early_init = dc_early_init,
	.late_init = dc_late_init,
	.sw_init = dc_sw_init,
	.sw_fini = dc_sw_fini,
	.hw_init = dc_hw_init,
	.hw_fini = dc_hw_fini,
	.suspend = dc_suspend,
	.resume = dc_resume,
	.is_idle = dc_is_idle,
	.wait_for_idle = dc_wait_for_idle,
	.check_soft_reset = dc_check_soft_reset,
	.soft_reset = dc_soft_reset,
};

const struct loonggpu_ip_block_version dc_ip_block = {
	.type = LOONGGPU_IP_BLOCK_TYPE_DCE,
	.major = 1,
	.minor = 0,
	.rev = 0,
	.funcs = &loonggpu_dc_funcs,
};
