// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM panel driver for the Samsung EA8076 AMOLED panel (1080x2340)
 * found on the Xiaomi Redmi K20 Pro (raphael).
 *
 * The panel is a command mode AMOLED driven over a 4-lane MIPI-DSI
 * link; the DPU paces transfers on the panel's TE signal in hardware.
 * The backlight brightness is a 16-bit DCS 0x51 write.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* Manufacturer Command Set */
#define MCS_PASSWD1		0xf0
#define MCS_PASSWD2		0xfc
#define MCS_ADDR_SHIFT		0xb0
#define MCS_UNKNOWN_B7		0xb7
#define MCS_BIAS_CURRENT_CTRL	0xd1
#define MCS_UNKNOWN_FF		0xff
#define MCS_OFC			0xe9
#define MCS_UNKNOWN_D9		0xd9
#define MCS_UNKNOWN_C0		0xc0
#define MCS_ERR_FG		0xe1
#define MCS_ERR_FG_MASK		0xe2

struct ea8076 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
};

static const struct regulator_bulk_data ea8076_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vcie" },
};

/* OFC setting */
static const u8 ea8076_ofc[] = {
	0xe9, 0x11, 0x55, 0xa6, 0x75, 0xa3, 0xa6, 0x37, 0xbe, 0x00, 0x1a,
	0xb8,
};

/* Panel timing setup */
static const u8 ea8076_d9[] = {
	0xd9, 0x14, 0x00, 0x00, 0x8f, 0x6e, 0x00, 0x00, 0x8f, 0x2e, 0x6e,
	0x34,
};

/* Panel driving mode setup */
static const u8 ea8076_c0[] = {
	0xc0, 0x31, 0x01, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x40,
	0x3d, 0x00, 0x5e, 0x00, 0x00, 0x00, 0x20, 0x07, 0x20, 0x30, 0x58,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x0b, 0x75, 0xd8, 0x00,
	0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0xf6, 0x97, 0x0f, 0x6d,
	0xe6, 0xcd, 0x05, 0x21, 0x8b, 0xec, 0xed, 0xed, 0xed, 0x0d, 0x3f,
	0x60, 0x00, 0x67, 0xfd, 0x6d, 0x51, 0xbc, 0xdd, 0xdd, 0x00, 0x00,
	0x07, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x2f, 0x00, 0x00, 0x03, 0x2e,
	0x3c, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x8c,
};

static inline struct ea8076 *to_ea8076(struct drm_panel *panel)
{
	return container_of(panel, struct ea8076, panel);
}

static void ea8076_reset(struct ea8076 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ea8076_on(struct ea8076 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	/* Delay 10ms for VCIE power */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD2, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ADDR_SHIFT, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_UNKNOWN_FF, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ADDR_SHIFT, 0x2f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_BIAS_CURRENT_CTRL, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD2, 0xa5, 0xa5);

	/* Sleep Out */
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);

	/* TE OUT (Vsync On) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0x5a, 0x5a);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	/* DBV Smooth Transition */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_UNKNOWN_B7, 0x01, 0x4b);

	/* Edge Dimming Speed Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ADDR_SHIFT, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_UNKNOWN_B7, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0xa5, 0xa5);

	/* Page Address Set */
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x0923);

	/* Set DDIC internal HFP */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD2, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ADDR_SHIFT, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_BIAS_CURRENT_CTRL, 0x33);

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, ea8076_ofc,
					sizeof(ea8076_ofc));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, ea8076_d9,
					sizeof(ea8076_d9));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, ea8076_c0,
					sizeof(ea8076_c0));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_UNKNOWN_C0, 0x31);

	/* Err_FG Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ERR_FG, 0x00, 0x00, 0xc3, 0x00, 0x22, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_ERR_FG_MASK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD1, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MCS_PASSWD2, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);

	/* Brightness Control */
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);

	/* Adaptive Control */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 67);

	/* Display On */
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static void ea8076_off(struct ea8076 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
}

static int ea8076_prepare(struct drm_panel *panel)
{
	struct ea8076 *ctx = to_ea8076(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ea8076_supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	ea8076_reset(ctx);

	ret = ea8076_on(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ea8076_supplies), ctx->supplies);
		return ret;
	}

	return 0;
}

static int ea8076_unprepare(struct drm_panel *panel)
{
	struct ea8076 *ctx = to_ea8076(panel);

	ea8076_off(ctx);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ea8076_supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode ea8076_mode = {
	.clock = (1080 + 64 + 20 + 64) * (2340 + 64 + 20 + 64) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 64,
	.hsync_end = 1080 + 64 + 20,
	.htotal = 1080 + 64 + 20 + 64,
	.vdisplay = 2340,
	.vsync_start = 2340 + 64,
	.vsync_end = 2340 + 64 + 20,
	.vtotal = 2340 + 64 + 20 + 64,
	.width_mm = 68,
	.height_mm = 147,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ea8076_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ea8076_mode);
}

static const struct drm_panel_funcs ea8076_panel_funcs = {
	.prepare = ea8076_prepare,
	.unprepare = ea8076_unprepare,
	.get_modes = ea8076_get_modes,
};

static int ea8076_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int ea8076_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops ea8076_bl_ops = {
	.update_status = ea8076_bl_update_status,
	.get_brightness = ea8076_bl_get_brightness,
};

static struct backlight_device *
ea8076_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 2047,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ea8076_bl_ops, &props);
}

static int ea8076_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ea8076 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ea8076, panel,
				   &ea8076_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(&dsi->dev,
					    ARRAY_SIZE(ea8076_supplies),
					    ea8076_supplies, &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = ea8076_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ea8076_remove(struct mipi_dsi_device *dsi)
{
	struct ea8076 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ea8076_of_match[] = {
	{ .compatible = "samsung,ea8076" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ea8076_of_match);

static struct mipi_dsi_driver ea8076_driver = {
	.probe = ea8076_probe,
	.remove = ea8076_remove,
	.driver = {
		.name = "panel-samsung-ea8076",
		.of_match_table = ea8076_of_match,
	},
};
module_mipi_dsi_driver(ea8076_driver);

MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
MODULE_DESCRIPTION("DRM driver for the Samsung EA8076 FHD AMOLED DSI panel");
MODULE_LICENSE("GPL");
