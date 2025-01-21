#ifndef __LT9721_H__
#define __LT9721_H__

#include "bridge_phy.h"

typedef  unsigned char u8;
typedef  unsigned int u32;

#define LT9721_CHIP_NAME    "LT9721"
#define LT9721_CHIP_ADDR    0X66

#define LT9721_REG_START    0x0000
#define LT9721_REG_END      0x90FE
#define LT9721_REG_PAGE     0x100
#define LT9721_REG_PAGE_SELECT  0xFF

#define LT9721_REG_LINK_STATUS  0x9006
#define LINK_STATUS_OUTPUT_DC_POS 3U
#define LINK_STATUS_STABLE 1U

/* General Registers */
#define LT9721_REG_CHIP_VERSION_BASE 0x6000
#define CHIP_VERSION_LEN 2U
#define LT9721_REG_CHIP_VERSION(x) (LT9721_REG_CHIP_VERSION_BASE + (x))

enum lt9721_chip_version {
	LT9721_VER_Unknown = 0,
	LT9721_VER_1,
};

enum lt9721_pll_level {
	LT9721_PLL_LEVEL_LOW = 0,
	LT9721_PLL_LEVEL_MIDDLE,
	LT9721_PLL_LEVEL_HIGH,
};

struct lt9721_device {
	enum lt9721_chip_version ver;
	enum lt9721_pll_level pll_level;
};

static const struct reg_sequence lt9721_pll_cfg_seq[] = {
	{ 0x8040, 0x22 },
	{ 0x8041, 0x36 },
	{ 0x8042, 0x00 },
	{ 0x8043, 0x80 },
	{ 0x701c, 0x18 },
	{ 0x701d, 0x42 },
	{ 0x701e, 0x00 },
	{ 0x701e, 0x01 },
	{ 0x701f, 0x3b },
	{ 0x804a, 0x0e },
	{ 0x804b, 0x10 },
	{ 0x8045, 0x83 },
	{ 0x601e, 0xbf },
	{ 0x601e, 0xff },
	{ 0x701f, 0x3b },
	{ 0x8044, 0x41 },
	{ 0x8045, 0x03 },
	{ 0x8046, 0x0a },
	{ 0x8048, 0x0a },
	{ 0x8040, 0x22 },
};

static const struct reg_sequence lt9711_rx_pll_cfg[] = {
	{ 0x7016, 0x30 },
	{ 0x7018, 0x03 },
	{ 0x7019, 0x1e },
	{ 0x701a, 0x24 },
};

static const struct reg_sequence lt9721_HDMIRxCDR[] = {
	{ 0x9005, 0x00 },
	{ 0x901d, 0x04 },
	{ 0x901e, 0x1f },
	{ 0x901f, 0x20 },
	{ 0x9024, 0xe0 },
	{ 0x9025, 0xf0 },
	{ 0x9027, 0x01 },
	{ 0x902b, 0x40 },
	{ 0x902c, 0x65 },
};

static const struct reg_sequence lt9721_tx_phy_cfg[] = {
	{ 0x7021, 0x0d },
	{ 0x7021, 0x0f },
	{ 0x7022, 0x77 },
	{ 0x7023, 0x77 },
	{ 0x7024, 0x80 },
	{ 0x7025, 0x00 },
	{ 0x7026, 0x80 },
	{ 0x7027, 0x00 },
	{ 0x7028, 0x80 },
	{ 0x7029, 0x00 },
	{ 0x702a, 0x80 },
	{ 0x702b, 0x00 },
	{ 0x702c, 0xb0 },
	{ 0x702c, 0xf0 },
	{ 0x702f, 0x70 },
	{ 0x7030, 0x24 },
	{ 0x7031, 0xFC },
	{ 0x8030, 0x0e },
};

#endif
