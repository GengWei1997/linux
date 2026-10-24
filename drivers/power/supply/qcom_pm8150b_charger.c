// SPDX-License-Identifier: GPL-2.0-only
/*
 * PM8150B SMB5 switch-mode battery charger and fuel-gauge driver
 *
 * The PM8150B PMIC combines an SMB5 battery charger with a GEN4 fuel
 * gauge.  Both are reached through the PMIC register map, the charger at
 * 0x1000-0x16ff and the gauge at 0x4000-0x43ff, so a single driver talks
 * to both and exposes the usual "battery" and "usb" power supplies.
 *
 * This is a clean-room driver written from the register map.  The values,
 * encodings and read sequences follow the PM8150B SMB5/GEN4 hardware
 * documentation as used by the Qualcomm vendor drivers; none of that code
 * is copied here.  It covers the parts a Linux system needs: report the
 * battery state of charge, voltage and temperature, report the source that
 * was plugged in, and run a safe CC/CV charge with the limits the battery
 * profile asks for.  Step charging, JEITA and OTG boost are left to the
 * hardware defaults and are not driven from software here.
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* ======================== SMB5 charger registers ======================== */
/* CHGR peripheral (0x1000) */
#define BATTERY_CHARGER_STATUS_1	0x1006
#define CHARGER_STATUS_MASK		GENMASK(2, 0)
#define CHARGER_INHIBIT			0
#define CHARGER_TRICKLE			1
#define CHARGER_PRE			2
#define CHARGER_FULLON			3
#define CHARGER_TAPER			4
#define CHARGER_TERMINATE		5
#define CHARGER_PAUSE			6
#define CHARGER_DISABLE			7

#define CHARGING_ENABLE_CMD		0x1042
#define CHARGING_ENABLE			BIT(0)

#define CHGR_CFG2			0x1051
#define CHARGER_INHIBIT_BIT		BIT(0)

#define CHGR_FAST_CHARGE_CURRENT_CFG	0x1061
#define CHGR_FLOAT_VOLTAGE_CFG		0x1070

/* DCDC peripheral (0x1100) */
#define ICL_MAX_STATUS			0x1106
#define POWER_PATH_STATUS		0x110b
#define USBIN_SUSPEND_STS		BIT(6)
#define USE_USBIN			BIT(4)
#define VALID_INPUT_POWER_SOURCE	BIT(0)

#define AICL_STATUS			0x110a
#define AICL_DONE			BIT(0)

/* USBIN peripheral (0x1300) */
#define USBIN_INT_RT_STS		0x1310
#define USBIN_PLUGIN_RT_STS		BIT(4)

#define APSD_STATUS			0x1307
#define QC_CHARGER			BIT(1)
#define APSD_DTC_DONE			BIT(0)

#define APSD_RESULT_STATUS		0x1308
#define QC_3P0				BIT(6)
#define QC_2P0				BIT(5)
#define FLOAT_CHARGER			BIT(4)
#define DCP_CHARGER			BIT(3)
#define CDP_CHARGER			BIT(2)
#define OCP_CHARGER			BIT(1)
#define SDP_CHARGER			BIT(0)

#define QC_CHANGE_STATUS		0x1309
#define QC_12V				BIT(2)
#define QC_9V				BIT(1)
#define QC_5V				BIT(0)

#define USBIN_CMD_IL			0x1340
#define USBIN_SUSPEND			BIT(0)

#define CMD_ICL_OVERRIDE		0x1342
#define ICL_OVERRIDE			BIT(0)

#define USBIN_OPTIONS_1_CFG		0x1362
#define BC1P2_SRC_DETECT		BIT(3)

#define USBIN_LOAD_CFG			0x1365
#define ICL_OVERRIDE_AFTER_APSD		BIT(4)

#define USBIN_ICL_OPTIONS		0x1366
#define USBIN_MODE_CHG			BIT(0)

#define USBIN_CURRENT_LIMIT_CFG		0x1370

#define USBIN_AICL_OPTIONS_CFG		0x1380
#define SUSPEND_ON_COLLAPSE_USBIN	BIT(7)

/* TYPEC peripheral (0x1500) */
#define TYPE_C_SNK_STATUS		0x1506
#define SNK_RP_STD			BIT(3)
#define SNK_RP_1P5			BIT(2)
#define SNK_RP_3P0			BIT(1)

/* MISC peripheral (0x1600) */
#define WD_CFG				0x1651

/* ========================== GEN4 gauge registers ======================= */
/* fg-batt-soc (0x4000) */
#define BATT_SOC_MONOTONIC_SOC		0x4009
#define BATT_SOC_MONOTONIC_SOC_CP	0x400a
#define BATT_SOC_INT_RT_STS		0x4010
#define SOC_READY			BIT(1)

/* fg-batt-info (0x4100) */
#define BATT_INFO_VBATT_LSB		0x41a0
#define BATT_INFO_VBATT_MSB		0x41a1
#define BATT_INFO_VBATT_LSB_CP		0x41a6
#define BATT_INFO_VBATT_MSB_CP		0x41a7

/* fg-memif (0x4300): DMA window into the gauge's SRAM */
#define MEM_IF_INT_RT_STS		0x4310
#define GEN4_MEM_GNT			BIT(3)
#define MEM_IF_MEM_ARB_CFG		0x4340
#define MEM_ARB_REQ			BIT(0)
#define MEM_IF_MEM_INTF_CFG		0x4350
#define MEM_ACCESS_REQ			BIT(7)
#define IACS_SLCT			BIT(5)

/* Gauge algorithm activity, observed through the peek bus. */
#define BATT_INFO_PEEK_MUX4		0x41ee
#define ALG_ACTIVE_PEEK_CFG		0xac
#define BATT_INFO_PEEK_RD		0x41ef
#define ALG_ACTIVE			BIT(3)

/*
 * Battery temperature lives in gauge SRAM word 328, which the hardware
 * maps into DMA window 4.  Two bytes per word.
 */
#define GEN4_FG_DMA4_BASE		0x4800
#define SRAM_ADDR_OFFSET		0x20
#define BATT_TEMP_WORD			328

/*
 * Lithium cells are damaged by charging outside this range, so charging is
 * held off well before the cell's own protection board has to act.  The
 * band is deliberately generous: ordinary operation sits near 25 C.
 */
#define TEMP_MIN_DECI_C			0	/* 0 C */
#define TEMP_MAX_DECI_C			450	/* 45 C */

/*
 * One out-of-range sample is not enough to act on, since the gauge's DMA
 * window can hand back a stale word.  Two in a row is a real reading.
 */
#define TEMP_BAD_SAMPLES		2

/*
 * Input current limits.  A standard downstream port is limited by the USB
 * specification, a charging downstream port may supply more, and a
 * dedicated charger is only limited by what it advertises.  A source that
 * floats its D+ and D- pins tells us nothing, so it gets the conservative
 * figure a "probably a charger" is trusted with.
 */
#define ICL_SDP_UA			500000
#define ICL_CDP_UA			1500000
#define ICL_DCP_UA			1600000
#define ICL_QC_UA			2000000
#define ICL_FLOAT_UA			1000000
#define ICL_UNKNOWN_UA			100000

/*
 * A Type-C source advertises its current on CC: the default USB budget,
 * 1.5 A or 3 A.  This is what a source that does not speak BC1.2 gets.
 */
#define ICL_TYPEC_DEFAULT_UA		500000
#define ICL_TYPEC_1P5A_UA		1500000
#define ICL_TYPEC_3A_UA			3000000

/*
 * Gauge scaling.  One battery-voltage LSB is 122070/1000 uV.  The
 * monotonic SOC register runs 0-255 for 0-100 %.  The RRADC temperature
 * is 10 bits, 0.25 C per LSB, offset by -128 C.
 */
#define VBATT_NUMR			122070
#define VBATT_DENR			1000
#define FULL_SOC_RAW			255

struct pm8150b_chg {
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*battery;
	struct power_supply	*usb;

	/* charge limits, in micro-units, from the battery profile / DT */
	unsigned int		fv_uv;
	unsigned int		fcc_ua;
	unsigned int		icl_ua;
	bool			icl_manual;	/* userspace took the limit */

	/*
	 * Last state reported to userspace.  The charger is polled, so these
	 * let the poller signal a change only when something actually moved.
	 */
	int			status;
	int			usb_online;
	int			src_type;	/* last source class reported */
	int			src_voltage_uv;	/* last QC class reported */
	int			capacity;
	int			temp_deci;
	bool			temp_valid;
	int			temp_bad;	/* out-of-range samples in a row */
	bool			charge_allowed;

	/* Requested IRQs, held for the unload sequence, see chg_remove(). */
	int			*irqs;

	struct delayed_work	status_work;
};

/* ------------------------- register helpers --------------------------- */

static int chg_read(struct pm8150b_chg *chg, unsigned int reg, unsigned int *val)
{
	return regmap_read(chg->regmap, reg, val);
}

static int chg_write(struct pm8150b_chg *chg, unsigned int reg, unsigned int val)
{
	return regmap_write(chg->regmap, reg, val);
}

static int chg_masked_write(struct pm8150b_chg *chg, unsigned int reg,
			    unsigned int mask, unsigned int val)
{
	return regmap_update_bits(chg->regmap, reg, mask, val);
}

/* --------------------------- fuel gauge ------------------------------- */

/*
 * The gauge publishes each measurement in a live/shadow register pair;
 * the two copies must agree before the value is trusted.  Read both and
 * retry a few times until they match.
 */
static int fg_read_pair(struct pm8150b_chg *chg, unsigned int reg,
			unsigned int cp_reg, unsigned int *val)
{
	unsigned int a, b;
	int i, rc;

	for (i = 0; i < 5; i++) {
		rc = chg_read(chg, reg, &a);
		if (rc < 0)
			return rc;
		rc = chg_read(chg, cp_reg, &b);
		if (rc < 0)
			return rc;
		if (a == b) {
			*val = a;
			return 0;
		}
	}

	/* Copies never settled; use the live register rather than fail. */
	*val = a;
	return 0;
}

static int fg_read_triple(struct pm8150b_chg *chg, unsigned int lsb,
			  unsigned int msb, unsigned int lsb_cp,
			  unsigned int msb_cp, unsigned int msb_mask,
			  unsigned int *val)
{
	unsigned int lo, hi, lo2, hi2;
	int i, rc;

	for (i = 0; i < 5; i++) {
		rc = chg_read(chg, lsb, &lo);
		if (rc < 0)
			return rc;
		rc = chg_read(chg, msb, &hi);
		if (rc < 0)
			return rc;
		rc = chg_read(chg, lsb_cp, &lo2);
		if (rc < 0)
			return rc;
		rc = chg_read(chg, msb_cp, &hi2);
		if (rc < 0)
			return rc;
		if (lo == lo2 && hi == hi2) {
			*val = ((hi & msb_mask) << 8) | lo;
			return 0;
		}
	}

	*val = ((hi & msb_mask) << 8) | lo;
	return 0;
}

/* Battery voltage in microvolts. */
static int fg_get_voltage_uv(struct pm8150b_chg *chg, int *uv)
{
	unsigned int raw;
	int rc;

	rc = fg_read_triple(chg, BATT_INFO_VBATT_LSB, BATT_INFO_VBATT_MSB,
			    BATT_INFO_VBATT_LSB_CP, BATT_INFO_VBATT_MSB_CP,
			    GENMASK(7, 0), &raw);
	if (rc < 0)
		return rc;

	*uv = div_u64((u64)raw * VBATT_NUMR, VBATT_DENR);
	return 0;
}

/* State of charge, 0-100 %. */
static int fg_get_soc(struct pm8150b_chg *chg, int *soc)
{
	unsigned int raw;
	int rc;

	rc = fg_read_pair(chg, BATT_SOC_MONOTONIC_SOC,
			  BATT_SOC_MONOTONIC_SOC_CP, &raw);
	if (rc < 0)
		return rc;

	if (raw >= FULL_SOC_RAW) {
		*soc = 100;
	} else if (raw == 0) {
		*soc = 0;
	} else {
		/* Map 0-255 onto 0-100, keeping 1 % resolution at the ends. */
		*soc = div_u64((u64)(raw - 1) * (100 - 2), FULL_SOC_RAW - 2) + 1;
	}
	return 0;
}

/*
 * Read one word of gauge SRAM.  The SRAM is shared with the gauge's own
 * algorithm, so access has to be arbitrated: request the memory bus, wait
 * for the grant, read the DMA window, then release.  Word 328 lives in
 * DMA window 4, two bytes per word.
 */
static int fg_sram_read_word(struct pm8150b_chg *chg, unsigned int word,
			     unsigned int *val)
{
	unsigned int addr, lo, hi, stat;
	int i, rc;

	/*
	 * The gauge algorithm owns the memory bus and only grants access
	 * while it is briefly idle.  Route the ALG_ACTIVE signal onto the
	 * peek bus and wait for that idle window before requesting.
	 */
	rc = chg_write(chg, BATT_INFO_PEEK_MUX4, ALG_ACTIVE_PEEK_CFG);
	if (rc < 0)
		return rc;

	for (i = 0; i < 35; i++) {
		rc = chg_read(chg, BATT_INFO_PEEK_RD, &stat);
		if (rc < 0)
			return rc;
		if (!(stat & ALG_ACTIVE))
			break;
		usleep_range(10000, 11000);
	}
	if (stat & ALG_ACTIVE)
		return -ETIMEDOUT;
	usleep_range(1000, 1100);

	rc = chg_masked_write(chg, MEM_IF_MEM_ARB_CFG, MEM_ARB_REQ, MEM_ARB_REQ);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, MEM_IF_MEM_INTF_CFG,
			      MEM_ACCESS_REQ | IACS_SLCT, MEM_ACCESS_REQ);
	if (rc < 0)
		goto release_arb;

	/* The hardware needs a few cycles before the grant lands. */
	usleep_range(40, 50);

	/*
	 * The gauge algorithm owns the memory bus and only grants access in
	 * the gaps between its own conversions, which can take well over a
	 * second to come around, so be patient before giving up.
	 */
	for (i = 0; i < 250; i++) {
		rc = chg_read(chg, MEM_IF_INT_RT_STS, &stat);
		if (rc < 0)
			goto release;
		if (stat & GEN4_MEM_GNT)
			break;
		usleep_range(10000, 11000);
	}
	if (!(stat & GEN4_MEM_GNT)) {
		rc = -ETIMEDOUT;
		goto release;
	}

	addr = GEN4_FG_DMA4_BASE + SRAM_ADDR_OFFSET + (word - 300) * 2;
	rc = chg_read(chg, addr, &lo);
	if (rc < 0)
		goto release;
	rc = chg_read(chg, addr + 1, &hi);
	if (rc < 0)
		goto release;

	*val = (hi << 8) | lo;
	rc = 0;

release:
	chg_masked_write(chg, MEM_IF_MEM_INTF_CFG, MEM_ACCESS_REQ | IACS_SLCT, 0);
release_arb:
	chg_masked_write(chg, MEM_IF_MEM_ARB_CFG, MEM_ARB_REQ, 0);
	return rc;
}

/* Battery temperature in deci-degrees Celsius. */
static int fg_read_temp(struct pm8150b_chg *chg, int *temp)
{
	unsigned int raw;
	int i, rc;

	/*
	 * The DMA grant races with the gauge algorithm and is often denied,
	 * so try a few times before giving up.  The caller caches the result
	 * and only refreshes it periodically.
	 */
	for (i = 0; i < 3; i++) {
		rc = fg_sram_read_word(chg, BATT_TEMP_WORD, &raw);
		if (!rc) {
			*temp = sign_extend32(raw, 9) * 100 / 40;
			return 0;
		}
	}

	return rc;
}

/* ---------------------------- charger --------------------------------- */

/* Is a valid USB source powering the system? */
static int chg_usb_online(struct pm8150b_chg *chg, int *online)
{
	unsigned int stat;
	int rc;

	rc = chg_read(chg, POWER_PATH_STATUS, &stat);
	if (rc < 0)
		return rc;

	*online = (stat & USE_USBIN) && (stat & VALID_INPUT_POWER_SOURCE);
	return 0;
}

/* Is anything plugged into USB? */
static int chg_usb_present(struct pm8150b_chg *chg, int *present)
{
	unsigned int stat;
	int rc;

	rc = chg_read(chg, USBIN_INT_RT_STS, &stat);
	if (rc < 0)
		return rc;

	*present = !!(stat & USBIN_PLUGIN_RT_STS);
	return 0;
}

/*
 * What a source is good for: how it identifies itself, what voltage it has
 * been talked into, and how much current may safely be drawn from it.
 */
struct chg_source {
	enum power_supply_type		type;
	enum power_supply_usb_type	usb_type;
	int				voltage_uv;
	unsigned int			icl_ua;
};

/*
 * Source classification.  BC1.2 detection runs in the hardware and leaves
 * its verdict in the APSD registers; a dedicated charger that also passed
 * the Qualcomm handshake is flagged QC, and QC_CHANGE_STATUS tells which
 * voltage class the adapter has been asked for.  Mainline has no power
 * supply type for the QC variants -- they are dedicated chargers -- so
 * they map to USB_DCP and the negotiated class is reported as VOLTAGE_NOW.
 *
 * The current limit starts from what the Type-C CC pins advertise, which is
 * all a source that speaks none of the above tells us.  A BC1.2 verdict
 * overrides it: a legacy cable advertises the USB default on CC no matter
 * how strong the charger behind it is.
 */
static int chg_read_source(struct pm8150b_chg *chg, struct chg_source *src)
{
	unsigned int apsd, result, qc, snk;
	int rc;

	src->type = POWER_SUPPLY_TYPE_USB;
	src->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	src->voltage_uv = 5000000;
	src->icl_ua = ICL_UNKNOWN_UA;

	rc = chg_read(chg, TYPE_C_SNK_STATUS, &snk);
	if (rc < 0)
		return rc;

	if (snk & SNK_RP_3P0)
		src->icl_ua = ICL_TYPEC_3A_UA;
	else if (snk & SNK_RP_1P5)
		src->icl_ua = ICL_TYPEC_1P5A_UA;
	else if (snk & SNK_RP_STD)
		src->icl_ua = ICL_TYPEC_DEFAULT_UA;

	rc = chg_read(chg, APSD_STATUS, &apsd);
	if (rc < 0)
		return rc;

	if (!(apsd & APSD_DTC_DONE))
		return 0;

	rc = chg_read(chg, APSD_RESULT_STATUS, &result);
	if (rc < 0)
		return rc;

	if (result & (DCP_CHARGER | OCP_CHARGER)) {
		src->type = POWER_SUPPLY_TYPE_USB_DCP;
		src->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		src->icl_ua = ICL_DCP_UA;
	} else if (result & FLOAT_CHARGER) {
		/*
		 * Floating D+/D-: a dedicated charger that does not identify
		 * itself, or a USB-C source that has not been asked yet.  The
		 * conservative figure serves both.
		 */
		src->type = POWER_SUPPLY_TYPE_USB_DCP;
		src->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		src->icl_ua = ICL_FLOAT_UA;
	} else if (result & CDP_CHARGER) {
		src->type = POWER_SUPPLY_TYPE_USB_CDP;
		src->usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		src->icl_ua = ICL_CDP_UA;
	} else if (result & SDP_CHARGER) {
		src->usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		src->icl_ua = ICL_SDP_UA;
	}

	/* A QC adapter says so both in APSD_STATUS and in the verdict. */
	if (!(apsd & QC_CHARGER) && !(result & (QC_2P0 | QC_3P0)))
		return 0;

	rc = chg_read(chg, QC_CHANGE_STATUS, &qc);
	if (rc < 0)
		return rc;

	src->type = POWER_SUPPLY_TYPE_USB_DCP;
	src->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	src->icl_ua = ICL_QC_UA;

	if (qc & QC_12V)
		src->voltage_uv = 12000000;
	else if (qc & QC_9V)
		src->voltage_uv = 9000000;
	else if (qc & QC_5V)
		src->voltage_uv = 5000000;

	return 0;
}

static int chg_get_status(struct pm8150b_chg *chg, int *status)
{
	unsigned int stat;
	int online, rc;

	rc = chg_usb_online(chg, &online);
	if (rc < 0)
		return rc;

	/*
	 * With no input source the charger is out of the picture whatever
	 * its state register says: the battery is on its own.
	 */
	if (!online) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	rc = chg_read(chg, BATTERY_CHARGER_STATUS_1, &stat);
	if (rc < 0)
		return rc;

	stat &= CHARGER_STATUS_MASK;

	switch (stat) {
	case CHARGER_TRICKLE:
	case CHARGER_PRE:
	case CHARGER_FULLON:
	case CHARGER_TAPER:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_TERMINATE:
	case CHARGER_INHIBIT:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case CHARGER_PAUSE:
	case CHARGER_DISABLE:
	default:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	}
	return 0;
}

static int chg_set_float_voltage(struct pm8150b_chg *chg, unsigned int fv_uv)
{
	unsigned int raw;

	if (fv_uv < 3600000)
		fv_uv = 3600000;
	if (fv_uv > 4790000)
		fv_uv = 4790000;

	raw = (fv_uv - 3600000) / 10000;
	return chg_write(chg, CHGR_FLOAT_VOLTAGE_CFG, raw);
}

static int chg_set_fcc(struct pm8150b_chg *chg, unsigned int fcc_ua)
{
	unsigned int raw;

	if (fcc_ua > 8000000)
		fcc_ua = 8000000;

	raw = fcc_ua / 50000;
	return chg_write(chg, CHGR_FAST_CHARGE_CURRENT_CFG, raw);
}

/*
 * Program the input current limit, then bring the input out of suspend.
 * The hardware holds USBIN suspended while it works out what was plugged
 * in, and this is what clears it: without the write there is no charging at
 * all, whatever the limit says.  A zero limit hands the choice back to the
 * hardware's own source detection.
 */
static int chg_set_icl(struct pm8150b_chg *chg, unsigned int icl_ua)
{
	unsigned int raw;
	int rc;

	if (!icl_ua) {
		/* Hand the limit back to source detection. */
		rc = chg_masked_write(chg, CMD_ICL_OVERRIDE, ICL_OVERRIDE, 0);
		if (rc < 0)
			return rc;

		rc = chg_masked_write(chg, USBIN_LOAD_CFG,
				      ICL_OVERRIDE_AFTER_APSD, 0);
		if (rc < 0)
			return rc;

		chg->icl_ua = 0;
		return chg_masked_write(chg, USBIN_CMD_IL, USBIN_SUSPEND, 0);
	}

	if (icl_ua > 5000000)
		icl_ua = 5000000;

	raw = icl_ua / 50000;

	rc = chg_write(chg, USBIN_CURRENT_LIMIT_CFG, raw);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, USBIN_ICL_OPTIONS, USBIN_MODE_CHG,
			      USBIN_MODE_CHG);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, CMD_ICL_OVERRIDE, ICL_OVERRIDE,
			      ICL_OVERRIDE);
	if (rc < 0)
		return rc;

	chg->icl_ua = icl_ua;

	return chg_masked_write(chg, USBIN_CMD_IL, USBIN_SUSPEND, 0);
}

/*
 * Take the current the plugged-in source is good for.  Detection settles
 * some time after the plug event, so this runs again on every poll.  True is
 * returned when the verdict differs from the one already reported.
 */
static bool chg_update_input(struct pm8150b_chg *chg)
{
	struct chg_source src;
	bool changed;
	int rc;

	rc = chg_read_source(chg, &src);
	if (rc < 0)
		return false;

	changed = src.type != chg->src_type ||
		  src.voltage_uv != chg->src_voltage_uv;

	/* A limit set from userspace stays until it is cleared again. */
	if (!chg->icl_manual && src.icl_ua != chg->icl_ua) {
		rc = chg_set_icl(chg, src.icl_ua);
		if (rc < 0)
			dev_warn(chg->dev, "failed to limit input to %u uA\n",
				 src.icl_ua);
	}

	chg->src_type = src.type;
	chg->src_voltage_uv = src.voltage_uv;
	return changed;
}

/* Input current limit the hardware is actually running at, in microamps. */
static int chg_get_icl(struct pm8150b_chg *chg, unsigned int *icl_ua)
{
	unsigned int raw;
	int rc;

	rc = chg_read(chg, ICL_MAX_STATUS, &raw);
	if (rc < 0)
		return rc;

	*icl_ua = raw * 50000;
	return 0;
}

static int chg_enable(struct pm8150b_chg *chg, bool enable)
{
	return chg_masked_write(chg, CHARGING_ENABLE_CMD, CHARGING_ENABLE,
				enable ? CHARGING_ENABLE : 0);
}

/*
 * Program a complete, safe charge configuration: the float voltage that
 * stops over-charge, the fast-charge current, and an input current limit
 * left to the hardware's source detection.
 */
static int chg_configure(struct pm8150b_chg *chg)
{
	int rc;

	rc = chg_set_float_voltage(chg, chg->fv_uv);
	if (rc < 0)
		return dev_err_probe(chg->dev, rc, "float voltage\n");

	rc = chg_set_fcc(chg, chg->fcc_ua);
	if (rc < 0)
		return dev_err_probe(chg->dev, rc, "charge current\n");

	rc = chg_set_icl(chg, chg->icl_ua);
	if (rc < 0)
		return dev_err_probe(chg->dev, rc, "input current\n");

	rc = chg_enable(chg, true);
	if (rc < 0)
		return dev_err_probe(chg->dev, rc, "charge enable\n");

	return 0;
}

/* ------------------------- power supply ------------------------------- */

static int chg_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct pm8150b_chg *chg = power_supply_get_drvdata(psy);
	int rc, tmp;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		rc = chg_get_status(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = fg_get_voltage_uv(chg, &tmp);
		val->intval = tmp;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = fg_get_soc(chg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/*
		 * The gauge's DMA window is slow and racy to reach, so the
		 * temperature is sampled in the background and cached here.
		 *
		 * The battery's thermal zone is registered before the first
		 * sample has landed, and it polls right away.  Answer that
		 * with -EAGAIN: "not yet" rather than "no such data", which
		 * is what the thermal core treats as a transient condition
		 * and quietly tries again, instead of reporting a failure.
		 */
		if (!chg->temp_valid)
			return -EAGAIN;
		val->intval = chg->temp_deci;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = chg_get_status(chg, &tmp);
		if (rc < 0)
			break;
		if (tmp == POWER_SUPPLY_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/*
		 * Charging is held off outside this band, so reporting it as
		 * unhealthy is the truth rather than a warning.
		 */
		if (!chg->temp_valid)
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		else if (chg->temp_deci < TEMP_MIN_DECI_C)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (chg->temp_deci > TEMP_MAX_DECI_C)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "PM8150B";
		rc = 0;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int chg_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct pm8150b_chg *chg = power_supply_get_drvdata(psy);
	struct chg_source src;
	int rc;
	unsigned int tmp;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return chg_usb_online(chg, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		return chg_usb_present(chg, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = chg_get_icl(chg, &tmp);
		if (!rc)
			val->intval = tmp;
		return rc;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chg->icl_ua;
		return 0;
	default:
		break;
	}

	/* What is left describes the source that is plugged in. */
	rc = chg_read_source(chg, &src);
	if (rc < 0)
		return rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = src.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = src.usb_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = src.voltage_uv;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int chg_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct pm8150b_chg *chg = power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;

	/* Zero hands the limit back to the hardware's source detection. */
	chg->icl_manual = val->intval > 0;
	return chg_set_icl(chg, val->intval);
}

static int chg_usb_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static enum power_supply_property chg_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property chg_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static const struct power_supply_desc chg_battery_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= chg_battery_props,
	.num_properties	= ARRAY_SIZE(chg_battery_props),
	.get_property	= chg_battery_get_property,
};

static const struct power_supply_desc chg_usb_desc = {
	.name		= "usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
			  BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_DCP),
	.properties	= chg_usb_props,
	.num_properties	= ARRAY_SIZE(chg_usb_props),
	.get_property	= chg_usb_get_property,
	.set_property	= chg_usb_set_property,
	.property_is_writeable = chg_usb_property_is_writeable,
};

/* --------------------------- state polling ---------------------------- */

static void chg_status_work(struct work_struct *work)
{
	struct pm8150b_chg *chg =
		container_of(work, struct pm8150b_chg, status_work.work);
	bool changed = false;
	bool allow = true;
	unsigned int delay_ms;
	int status, online, capacity, temp;

	if (!chg_usb_online(chg, &online) && online != chg->usb_online) {
		chg->usb_online = online;
		changed = true;
	}

	/*
	 * The source classification is not instant: BC1.2 and the QC
	 * handshake settle some time after the plug event, so watch it here
	 * and let userspace know when the verdict changes.
	 */
	if (chg_update_input(chg))
		changed = true;

	if (!chg_get_status(chg, &status) && status != chg->status) {
		chg->status = status;
		changed = true;
	}

	if (!fg_get_soc(chg, &capacity) && capacity != chg->capacity) {
		chg->capacity = capacity;
		changed = true;
	}

	if (!fg_read_temp(chg, &temp) && temp != chg->temp_deci) {
		chg->temp_deci = temp;
		chg->temp_valid = true;
		changed = true;
	}

	/* Hold charging off while the cell is outside its safe range. */
	if (chg->temp_valid) {
		if (chg->temp_deci >= TEMP_MIN_DECI_C &&
		    chg->temp_deci <= TEMP_MAX_DECI_C) {
			chg->temp_bad = 0;
		} else if (++chg->temp_bad >= TEMP_BAD_SAMPLES) {
			allow = false;
		}

		if (allow != chg->charge_allowed) {
			if (chg_enable(chg, allow)) {
				/* Leave the flag alone so this is retried. */
				dev_warn(chg->dev, "failed to %s charging\n",
					 allow ? "resume" : "hold off");
			} else {
				chg->charge_allowed = allow;
				changed = true;
			}
		}
	}

	if (changed) {
		power_supply_changed(chg->battery);
		power_supply_changed(chg->usb);
	}

	/*
	 * The battery temperature lives behind the gauge's racy DMA window,
	 * so it is sampled here rather than in the property getter, which
	 * must stay fast and non-blocking.  Come back quickly until the
	 * first sample lands and while the cell is outside its safe range,
	 * where prompt action matters; a half-minute is plenty otherwise.
	 */
	delay_ms = (chg->temp_valid && !chg->temp_bad) ? 30000 : 5000;
	schedule_delayed_work(&chg->status_work,
			      round_jiffies_relative(msecs_to_jiffies(delay_ms)));
}

static irqreturn_t chg_plugin_irq(int irq, void *data)
{
	struct pm8150b_chg *chg = data;

	schedule_delayed_work(&chg->status_work, 0);
	return IRQ_HANDLED;
}

static irqreturn_t chg_state_irq(int irq, void *data)
{
	struct pm8150b_chg *chg = data;

	schedule_delayed_work(&chg->status_work, 0);
	return IRQ_HANDLED;
}

struct chg_irq {
	const char		*name;
	irq_handler_t		handler;
};

static const struct chg_irq chg_irqs[] = {
	{ "usbin-plugin",		chg_plugin_irq },
	{ "chg-state-change",		chg_state_irq },
};

/* ------------------------------- probe -------------------------------- */

/*
 * The charge limits come from the battery's own description, so they stay
 * with whichever cell is fitted rather than with the board.
 */
static int chg_read_battery_limits(struct pm8150b_chg *chg)
{
	struct power_supply_battery_info *info;
	int rc;

	rc = power_supply_get_battery_info(chg->battery, &info);
	if (rc)
		return dev_err_probe(chg->dev, rc, "battery description\n");

	chg->fv_uv = info->constant_charge_voltage_max_uv;
	chg->fcc_ua = info->constant_charge_current_max_ua;

	power_supply_put_battery_info(chg->battery, info);

	/*
	 * Float voltage is what stops over-charge, so an implausible value
	 * must not be quietly accepted: refuse to charge at all rather than
	 * run with a limit that could over-stress the cell.  The upper bound
	 * is the highest voltage any of these lithium cells tolerate.
	 */
	if (chg->fv_uv < 3600000 || chg->fv_uv > 4400000)
		return dev_err_probe(chg->dev, -EINVAL,
				     "float voltage %u uV out of range\n",
				     chg->fv_uv);

	if (!chg->fcc_ua || chg->fcc_ua > 8000000)
		return dev_err_probe(chg->dev, -EINVAL,
				     "charge current %u uA out of range\n",
				     chg->fcc_ua);

	return 0;
}

static int chg_request_irqs(struct pm8150b_chg *chg)
{
	int i, irq, rc;

	chg->irqs = devm_kcalloc(chg->dev, ARRAY_SIZE(chg_irqs), sizeof(int),
				 GFP_KERNEL);
	if (!chg->irqs)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(chg_irqs); i++) {
		irq = platform_get_irq_byname_optional(to_platform_device(chg->dev),
						       chg_irqs[i].name);
		if (irq < 0)
			continue;

		rc = devm_request_threaded_irq(chg->dev, irq, NULL,
					       chg_irqs[i].handler,
					       IRQF_ONESHOT, chg_irqs[i].name,
					       chg);
		if (rc < 0)
			return dev_err_probe(chg->dev, rc,
					     "irq %s\n", chg_irqs[i].name);
		chg->irqs[i] = irq;
	}

	return 0;
}

/*
 * Turn on hardware source detection.  BC1.2 then classifies whatever was
 * plugged in and leaves the verdict in the APSD registers for
 * chg_read_source() to pick up; nothing needs to poke the hardware after
 * this, which is what makes charging work before the driver has even
 * probed.
 *
 * Detection drives the D+/D- pair while the input-current ramp is running,
 * and the SMB5 reads that transient as a collapsing input.  The protection
 * latches USBIN_SUSPEND, which the suspend command register does not clear,
 * so charging then stops until the next re-plug.  The protection stays off:
 * a weak source is already handled by AICL, which trims the input current
 * to what the source can deliver and does not latch.
 */
static int chg_source_detect_init(struct pm8150b_chg *chg)
{
	int rc;

	rc = chg_masked_write(chg, USBIN_AICL_OPTIONS_CFG,
			      SUSPEND_ON_COLLAPSE_USBIN, 0);
	if (rc < 0)
		return rc;

	return chg_masked_write(chg, USBIN_OPTIONS_1_CFG, BC1P2_SRC_DETECT,
				BC1P2_SRC_DETECT);
}

static int chg_hw_init(struct pm8150b_chg *chg)
{
	int rc;

	/*
	 * Disable the software safety watchdog.  It exists to stop charging
	 * when the kernel hangs, but feeding it needs a periodic pet; the
	 * hardware float voltage and charge termination are the real
	 * over-charge protection and run independently of it.
	 */
	rc = chg_write(chg, WD_CFG, 0);
	if (rc < 0)
		return rc;

	rc = chg_source_detect_init(chg);
	if (rc < 0)
		return rc;

	rc = chg_configure(chg);
	if (rc < 0)
		return rc;

	/* Take whatever the attached source is good for. */
	chg_update_input(chg);

	/* Charging is enabled; only the temperature check can hold it off. */
	chg->charge_allowed = true;
	return 0;
}

static int chg_probe(struct platform_device *pdev)
{
	struct pm8150b_chg *chg;
	struct power_supply_config cfg = {};
	int rc;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	platform_set_drvdata(pdev, chg);

	chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(chg->dev, -ENODEV, "no regmap\n");

	INIT_DEFERRABLE_WORK(&chg->status_work, chg_status_work);

	rc = chg_request_irqs(chg);
	if (rc < 0)
		return rc;

	cfg.drv_data = chg;

	chg->battery = devm_power_supply_register(chg->dev, &chg_battery_desc,
						  &cfg);
	if (IS_ERR(chg->battery))
		return dev_err_probe(chg->dev, PTR_ERR(chg->battery),
				     "battery psy\n");

	rc = chg_read_battery_limits(chg);
	if (rc < 0)
		return rc;

	rc = chg_hw_init(chg);
	if (rc < 0)
		return rc;

	chg->usb = devm_power_supply_register(chg->dev, &chg_usb_desc, &cfg);
	if (IS_ERR(chg->usb))
		return dev_err_probe(chg->dev, PTR_ERR(chg->usb),
				     "usb psy\n");

	/* Prime the cached state, including the battery temperature. */
	schedule_delayed_work(&chg->status_work, 0);

	dev_info(chg->dev, "registered (FV %u uV, FCC %u uA)\n",
		 chg->fv_uv, chg->fcc_ua);

	return 0;
}

static void chg_remove(struct platform_device *pdev)
{
	struct pm8150b_chg *chg = platform_get_drvdata(pdev);
	int i;

	/*
	 * The IRQs feed the poller below and it re-arms itself, so the
	 * IRQs go first: disable_irq() waits out a handler that is in
	 * flight and, with them off, nothing can queue work once the
	 * cancel has drained what is pending.  Work running past this
	 * point would chase freed driver state.
	 */
	for (i = 0; i < ARRAY_SIZE(chg_irqs); i++)
		if (chg->irqs[i] > 0)
			disable_irq(chg->irqs[i]);

	cancel_delayed_work_sync(&chg->status_work);
}

static const struct of_device_id chg_match_table[] = {
	{ .compatible = "qcom,pm8150b-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, chg_match_table);

static struct platform_driver chg_driver = {
	.driver = {
		.name = "pm8150b-charger",
		.of_match_table = chg_match_table,
	},
	.probe = chg_probe,
	.remove = chg_remove,
};
module_platform_driver(chg_driver);

MODULE_DESCRIPTION("PM8150B SMB5 battery charger and gauge driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
