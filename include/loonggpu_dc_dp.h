#ifndef __LOONGGPU_DP_H__
#define __LOONGGPU_DP_H__

#define DP_LINK_1P62G                       6
#define DP_LINK_2P16G                       8
#define DP_LINK_2P43G                       9
#define DP_LINK_2P7G                        10
#define DP_LINK_3P24G                       12
#define DP_LINK_4P32G                       16
#define DP_LINK_5P4G                        20
#define DP_LINK_8P1G                        30
#define DP_LINK_X1                          0x1
#define DP_LINK_X2                          0x2
#define DP_LINK_X4                          0x4

#define DP_PHY_1P62G                        0x0
#define DP_PHY_2P16G                        0x1
#define DP_PHY_2P43G                        0x2
#define DP_PHY_2P7G                         0x3
#define DP_PHY_3P24G                        0x4
#define DP_PHY_4P32G                        0x5
#define DP_PHY_5P4G                         0x6
#define DP_PHY_8P1G                         0x7
#define DP_PHY_X1                           0x0
#define DP_PHY_X2                           0x1
#define DP_PHY_X4                           0x2


typedef struct aux_msg {
	unsigned int addr;
	unsigned int size;
	unsigned long data_low;
	unsigned long data_high;
} aux_msg_t;

typedef struct dp_feature {
	unsigned int dp_phy_rate;
	unsigned int dp_phy_xlane;
	unsigned int dp_pixclk;
	unsigned int dp_link_rate;
	unsigned int dp_link_xlane;
} dp_feature_t;

typedef struct {
	int link_rate;
	int phy_rate;
	int clk;
} dp_rate_info_t;

typedef struct {
	unsigned int bw;
	unsigned int phy_rate;
	unsigned int phy_lane;
	unsigned int link_rate;
	unsigned int link_lane;
} dp_bandwidth_entry_t;

unsigned int aux_config(struct loonggpu_device *adev, unsigned int rd_wr, aux_msg_t aux_msg, int intf);
int ls2k3000_dp_audio_init(struct loonggpu_dc_crtc *crtc, int intf);
int ls2k3000_dp_noaudio_init(struct loonggpu_dc_crtc *crtc, int intf);
int ls2k3000_dp_init(struct loonggpu_dc_crtc *crtc, int intf);
void ls2k3000_dp_suspend(struct loonggpu_dc_crtc *crtc, int intf);
int ls2k3000_dp_resume(struct loonggpu_dc_crtc *crtc, int intf);
bool ls2k3000_dp_enable(struct loonggpu_dc_crtc *crtc, int intf, bool enable);
void ls2k3000_dp_pll_set(struct loonggpu_dc_crtc *crtc, int intf, struct dc_timing_info *timing);

#endif /* __LOONGGPU_DP_H__ */
