# SPDX-License-Identifier: GPL-2.0-only
#
# Goodix GTx8 touchscreen driver.
#
# The vendor tree splits this into a composite core module plus one module
# per optional feature.  The firmware update code cannot stay separate: the
# core calls goodix_do_fw_update() from the config-download path and from the
# reset path, so it has to be linked into the same module.
#
# gesture and tools really are optional and keep their own modules.
#
obj-$(CONFIG_TOUCHSCREEN_GOODIX_GTX8) += goodix_gtx8.o
goodix_gtx8-y := goodix_ts_i2c.o goodix_ts_core.o goodix_cfg_bin.o

goodix_gtx8-$(CONFIG_TOUCHSCREEN_GOODIX_GTX8_UPDATE) += goodix_gtx8_update.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_GTX8_GESTURE) += goodix_ts_gesture.o
obj-$(CONFIG_TOUCHSCREEN_GOODIX_GTX8_TOOLS) += goodix_ts_tools.o
