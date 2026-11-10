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
 * profile asks for, derated by the profile's JEITA and step-charging
 * tables.  On boards that fit an SMB1390 charge pump, the pump takes the
 * charge path over from the buck converter at the 9 V input class, see
 * chg_cp_enable().  OTG boost is left to the hardware defaults and is not
 * driven from software here.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
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
#define DCDC_FSW_SEL			0x1150
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
#define HVDCP_CHECK_TIMEOUT		BIT(6)
#define QC_AUTH_DONE			BIT(2)
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

#define CMD_APSD			0x1341
#define APSD_RERUN			BIT(0)

#define CMD_ICL_OVERRIDE		0x1342
#define ICL_OVERRIDE			BIT(0)

#define CMD_HVDCP_2			0x1343
#define HVDCP_FORCE_9V			BIT(4)
#define HVDCP_SINGLE_INC		BIT(0)

#define USBIN_OPTIONS_1_CFG		0x1362
#define HVDCP_AUTH_ALG_EN		BIT(6)
#define BC1P2_SRC_DETECT		BIT(3)
#define HVDCP_EN			BIT(2)

#define USBIN_LOAD_CFG			0x1365
#define ICL_OVERRIDE_AFTER_APSD		BIT(4)
#define USBIN_AICL_STEP_TIMING_SEL	GENMASK(3, 2)
#define USBIN_IN_COLLAPSE_GF_SEL	GENMASK(1, 0)

#define TYPE_C_CFG			0x1358
#define BC1P2_START_ON_CC		BIT(7)

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

/*
 * The SMB1390 charge pump's SMB_EN pin.  MISC_SMB_CFG routes the pin to
 * the pump and MISC_SMB_EN_CMD says who drives it: with EN_CP_CMD set the
 * hardware raises it while charging is active, and the override bits take
 * the pin over and park it low when the pump is not to run.  The pump is
 * armed underneath the pin separately, see chg_cp_enable().
 */
#define MISC_SMB_EN_CMD			0x1648
#define SMB_EN_OVERRIDE_VALUE		BIT(4)
#define SMB_EN_OVERRIDE			BIT(3)
#define EN_CP_CMD			BIT(0)

#define MISC_SMB_CFG			0x1690
#define SMB_EN_SEL			BIT(4)

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
 * Battery temperature lives in gauge SRAM word 328 and the battery current
 * in word 320; both are mapped into DMA window 4.  Two bytes per word.
 */
#define GEN4_FG_DMA4_BASE		0x4800
#define SRAM_ADDR_OFFSET		0x20
#define BATT_TEMP_WORD			328
#define BATT_CURRENT_WORD		320

/*
 * The gauge stores the battery current as a signed 16-bit word whose scale
 * is set by the sense resistor: 488282 in the denominator turns the raw
 * count into microamps, the same pair the vendor driver uses for
 * IBAT_FINAL.  The sign runs the other way round from the power supply
 * class: a positive word means current is leaving the cell.  The vendor
 * code negates it in fg-alg.c before using it as a charge current
 * (ibatt_avg = -ibatt_avg), and the class interface wants charging
 * positive, so do the same here.
 */
#define BATT_CURRENT_NUMR		488282
#define BATT_CURRENT_DENR		1000

/*
 * Lithium cells are damaged by charging outside this range, so charging is
 * held off well before the cell's own protection board has to act.  The
 * band is the union of the JEITA table's ranges: inside it the tables
 * derate the charge limits, outside it charging stops.  Ordinary
 * operation sits near 25 C, in the middle of the table's full band.
 */
#define TEMP_MIN_DECI_C			0	/* 0 C */
#define TEMP_MAX_DECI_C			580	/* 58 C */

/*
 * One out-of-range sample is not enough to act on, since the gauge's DMA
 * window can hand back a stale word.  Two in a row is a real reading.
 */
#define TEMP_BAD_SAMPLES		2

/*
 * The battery profile's charge tables are consulted against the battery
 * temperature and voltage, with a little hysteresis at the entry
 * boundaries so a reading sitting on a boundary settles where it first
 * landed instead of flapping between the two entries.
 */
#define JEITA_HYST_DECI_C		5	/* 0.5 C */
#define STEP_HYST_UV			10000	/* 10 mV */

/*
 * In the warm JEITA band the float voltage is derated below the cell's
 * maximum.  Charging is held off at the derated top and let back only
 * once the cell has fallen away from it by this much, so the charger
 * does not cycle on and off around the float.
 */
#define JEITA_HOLD_HYST_UV		70000	/* 70 mV */

/* A profile carries up to this many breakpoints per table. */
#define CHG_TABLE_MAX			8

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
#define ICL_QC2_UA			1500000
#define ICL_QC3_UA			2800000
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
 * A QC3 adapter has no classes to request: it steps 200 mV per D+ pulse
 * and the pulse count is the voltage.  Twenty pulses take the 5 V default
 * to the 9 V class, which is where this stops: the 4.4 V cell needs no
 * more at this power level and it keeps the charger switcher inside its
 * comfortable range.
 */
#define HVDCP3_STEP_UV			200000
#define QC3_PULSES_FOR_6V		5
#define QC3_PULSES_FOR_9V		20

/*
 * The input current is backed off while the adapter walks up the ladder.
 * A load step on top of the rising input is what sends a marginal adapter
 * back to 5 V mid-burst; the limit goes back to what the source is good
 * for once the burst is over, see chg_update_input().
 */
#define ICL_QC3_BOOST_UA		1000000

/*
 * The SMB1390 charge pump is a 2:1 switched-capacitor divider: it feeds
 * the system rail at half the input voltage and draws half the charge
 * current from the adapter.  Its input limit is therefore the smaller of
 * what the source may deliver and half the charge current, and below
 * 900 mA the limit falls outside the pump's accurate range, where the
 * buck charger carries the charge alone.  The pump only clears the cell
 * once its output is above the battery voltage, so it takes over at the
 * 9 V input class and comes off again below it.
 */
#define CP_INPUT_MIN_UV			9000000
#define CP_ILIM_MIN_UA			900000

/*
 * Starting the pump takes several register writes against the PMIC
 * charger, whose state-change interrupt re-runs the poll that is doing
 * the starting.  A pump that refused to start is therefore left alone
 * for a while instead of being hammered, and tried again on a later poll.
 */
#define CP_RETRY_MS			10000

/*
 * Arming and converting are separated by the switcher's soft-start, and
 * the writes that arm it wake the same poll that checks on it.  A verdict
 * passed inside that window would be about a switcher still starting up.
 */
#define CP_SS_GRACE_MS			2000

/*
 * What the QC class says was asked of the adapter and what the input
 * actually carries only disagree when a source has dropped out of its
 * class: a load step sends a marginal adapter back to 5 V, and the class
 * state goes stale with it.  A gap this wide is the wire telling the
 * truth against a register that is not.
 */
#define QC_CLASS_TOL_UV			1500000

/*
 * One gap between the class and the wire is not enough to act on: right
 * after the walk the adapter is still slewing up to the class.  Two in a
 * row is a source that is really not there.
 */
#define QC_CLASS_BAD_SAMPLES		2

/*
 * The charger switcher wants a frequency picked for the input voltage it
 * is stepping down: the vendor walks it up in the "5 V", "6-8 V" and
 * "9 V" steps of its frequency table as VBUS climbs.  DCDC_FSW_SEL holds
 * 19.2 MHz / FSW - 1, so those steps program 1200, 1600 and 2100 kHz.
 */
#define FSW_1200_KHZ			15
#define FSW_1600_KHZ			11
#define FSW_2100_KHZ			8

/*
 * Gauge scaling.  One battery-voltage LSB is 122070/1000 uV.  The
 * monotonic SOC register runs 0-255 for 0-100 %.  The RRADC temperature
 * is 10 bits, 0.25 C per LSB, offset by -128 C.
 */
#define VBATT_NUMR			122070
#define VBATT_DENR			1000
#define FULL_SOC_RAW			255

/*
 * A breakpoint of one of the battery profile's charge tables: the reading
 * in [low, high] asks for val.  The temperature tables carry decidegrees
 * Celsius, the voltage table microvolts, and all of them microamps.
 */
struct chg_range {
	int			low;
	int			high;
	int			val;
};

/*
 * A parsed profile table and the entry it last matched, kept so a
 * boundary crossing can be judged against where the reading just was,
 * see chg_table_lookup().
 */
struct chg_table {
	struct chg_range	entry[CHG_TABLE_MAX];
	int			count;
	int			index;
};

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
	 * The limits the charger is actually programmed with: the maxima
	 * above, lowered to whatever the profile's tables ask for at the
	 * battery's present temperature and voltage.
	 */
	unsigned int		eff_fv_uv;
	unsigned int		eff_fcc_ua;
	bool			warm_hold;	/* derated float reached */

	/* Charge tables from the battery profile, see chg_read_table(). */
	struct chg_table	jeita_fcc;
	struct chg_table	jeita_fv;
	struct chg_table	step_fcc;

	/*
	 * Last state reported to userspace.  The charger is polled, so these
	 * let the poller signal a change only when something actually moved.
	 */
	int			status;
	int			usb_online;
	int			usb_present;	/* last plug state seen */
	int			src_type;	/* last source class reported */
	int			src_voltage_uv;	/* last QC class reported */
	int			class_bad;	/* class/wire gaps in a row */
	int			qc3_pulses;	/* QC3 steps sent so far */
	bool			qc3_boosting;	/* QC3 burst owns the input limit */
	bool			detecting;	/* source detection drives D+/D- */
	int			capacity;
	int			temp_deci;
	bool			temp_valid;
	int			temp_bad;	/* out-of-range samples in a row */
	bool			charge_allowed;
	int			current_ua;
	bool			current_valid;

	/*
	 * D+/D- are shared between the USB gadget and this charger's own
	 * source detection and QC signalling, see chg_dpdm_float().
	 */
	struct regulator	*dpdm;
	bool			dpdm_held;

	/*
	 * The SMB1390 charge pump and the limits that go with it, see
	 * chg_cp_enable().
	 */
	struct regulator	*cp;
	bool			cp_active;	/* pump is armed by us */
	bool			cp_converting;	/* switcher carries the charge */
	unsigned int		cp_ilim_ua;	/* the pump's input limit */
	unsigned int		cp_fcc_ua;	/* ceiling with the pump */
	unsigned long		cp_retry_jiff;	/* earliest start retry */
	unsigned long		cp_arm_jiff;	/* when the pump was armed */
	bool			src_qc3;	/* QC3 source on the input */

	/*
	 * The measured input voltage, on boards that wire it up.  The QC
	 * class says what was asked of the adapter; this says what the
	 * wire carries, see chg_get_vbus_uv().
	 */
	struct iio_channel	*vbus;

	/* Requested IRQs, held for the unload sequence, see chg_remove(). */
	int			*irqs;

	struct delayed_work	status_work;
	struct delayed_work	detect_work;
	struct delayed_work	qc3_work;
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

/* ----------------------- D+/D- ownership ---------------------------- */

/*
 * D+/D- are shared between the USB gadget and this charger.  The gadget
 * talks to a host over them, but the charger has to see the same pair to
 * classify the source and to signal QC voltages, and a gadget sitting on
 * the lines corrupts both: BC1.2 then reports an "other charging port"
 * whatever is plugged in and the QC handshake never completes.  The USB HS
 * PHY's "dpdm" regulator floats the pair while the charger needs it.
 */
static void chg_dpdm_float(struct pm8150b_chg *chg, bool on)
{
	int rc;

	if (!chg->dpdm || chg->dpdm_held == on)
		return;

	rc = on ? regulator_enable(chg->dpdm) : regulator_disable(chg->dpdm);
	if (rc < 0) {
		dev_warn(chg->dev, "failed to %s D+/D-\n",
			 on ? "take over" : "give back");
		return;
	}

	chg->dpdm_held = on;
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

/* Battery current in microamperes, positive while charging. */
static int fg_read_current_ua(struct pm8150b_chg *chg, int *ua)
{
	unsigned int raw;
	int i, rc;

	/*
	 * Same arbitration story as the temperature: the DMA window is only
	 * granted in the gaps between the gauge's own conversions, so retry
	 * a few times before reporting nothing.  Without this the current
	 * reads back as zero far more often than it reads back correctly,
	 * which is worse than an error the caller can cache around.
	 *
	 * A granted window can still hand back an all-zero word before the
	 * gauge has written its first result, and that decodes to a current
	 * of exactly zero.  Treat it as "no sample yet" rather than as a
	 * reading of no current: the two are indistinguishable in the raw
	 * value, and reporting the plateau as a real 0 A is what made the
	 * battery look idle while it was not.
	 */
	for (i = 0; i < 3; i++) {
		rc = fg_sram_read_word(chg, BATT_CURRENT_WORD, &raw);
		if (rc < 0)
			continue;
		if (!raw)
			continue;

		/*
		 * The gauge counts discharge as positive, the reverse of
		 * what the power supply class expects, so flip the sign
		 * once here and keep it flipped everywhere above.
		 */
		*ua = -div_s64((s64)sign_extend32(raw, 15) *
			       BATT_CURRENT_NUMR, BATT_CURRENT_DENR);
		return 0;
	}

	return rc ?: -EAGAIN;
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
	bool				qc2;
	bool				qc3;
	bool				floating;	/* nothing on D+/D- */
};

/*
 * The pair is held by default: a charger has nowhere to enumerate to and
 * needs D+/D- for the BC1.2 verdict and for the QC handshake that follows.
 * A source that identifies as a host gets the pair back, since the gadget
 * is the only thing that can talk to it.
 *
 * A floating pair is neither, and is resolved in favour of the PHY, see
 * below.
 */
static void chg_dpdm_policy(struct pm8150b_chg *chg, const struct chg_source *src)
{
	bool host;

	if (src->usb_type == POWER_SUPPLY_USB_TYPE_UNKNOWN)
		return;

	/*
	 * A floating verdict means BC1.2 saw nothing identify the source:
	 * either a dumb charger, or a host that was never given the pair.
	 * Only the PHY can tell the two apart, because a host answers a
	 * pull-up and a dumb charger does not, so the pair is handed over
	 * and left there.  Re-running the detection here would be wrong:
	 * BC1.2 drives the same lines the gadget is now using, and tearing
	 * an enumeration down to classify it again helps nobody.  Nothing
	 * is lost for a dumb charger either, as it speaks neither BC1.2 nor
	 * QC and keeps charging at the float limit.
	 */
	if (src->floating) {
		chg_dpdm_float(chg, false);
		return;
	}

	host = src->usb_type == POWER_SUPPLY_USB_TYPE_SDP ||
	       src->usb_type == POWER_SUPPLY_USB_TYPE_CDP;
	chg_dpdm_float(chg, !host);
}

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
		 * itself, or a host whose enumeration the charger is
		 * blocking.  The conservative current serves both.
		 */
		src->type = POWER_SUPPLY_TYPE_USB_DCP;
		src->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		src->icl_ua = ICL_FLOAT_UA;
		src->floating = true;
	} else if (result & CDP_CHARGER) {
		src->type = POWER_SUPPLY_TYPE_USB_CDP;
		src->usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		src->icl_ua = ICL_CDP_UA;
	} else if (result & SDP_CHARGER) {
		src->usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		src->icl_ua = ICL_SDP_UA;
	}

	/*
	 * The QC handshake lands in stages and its intermediate states
	 * look like real verdicts: a QC3 adapter answers QC_2P0 before its
	 * authentication completes.  Firing voltage commands at a handshake
	 * still in formation takes the adapter down, so nothing is believed
	 * before the authenticated state; the vendor driver acts on the
	 * very same bit.
	 */
	if (!(apsd & QC_AUTH_DONE))
		return 0;

	/* A QC adapter says so both in APSD_STATUS and in the verdict. */
	if (!(apsd & QC_CHARGER) && !(result & (QC_2P0 | QC_3P0)))
		return 0;

	rc = chg_read(chg, QC_CHANGE_STATUS, &qc);
	if (rc < 0)
		return rc;

	src->type = POWER_SUPPLY_TYPE_USB_DCP;
	src->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	src->qc2 = !!(result & QC_2P0);
	src->qc3 = !!(result & QC_3P0);

	/*
	 * The two QC generations are trusted with different currents: a
	 * QC2 source is an 18 W class adapter, while a QC3 one reaches the
	 * 9 V/3 A class these phones ship with.
	 */
	src->icl_ua = src->qc3 ? ICL_QC3_UA : ICL_QC2_UA;

	/*
	 * A QC2 adapter reports its class in QC_CHANGE_STATUS.  A QC3 one
	 * has no classes at all: it steps 200 mV per D+ pulse and the
	 * hardware only keeps the pulse count, so what was sent is what
	 * tells the voltage.
	 */
	if (src->qc3)
		src->voltage_uv = 5000000 + HVDCP3_STEP_UV * chg->qc3_pulses;
	else if (qc & QC_12V)
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
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case CHARGER_INHIBIT:
	case CHARGER_PAUSE:
	case CHARGER_DISABLE:
	default:
		/*
		 * Inhibited is charging held off by command, which is what
		 * the temperature band and the warm JEITA hold do, so it is
		 * a pause rather than a full battery.
		 */
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
 * The input voltage the wire actually carries, in microvolts.  The QC
 * class says what was asked of the adapter; this is what the source
 * really delivers, and the two disagree whenever a source drops out of
 * its class.  Boards that do not wire the ADC simply have no second
 * opinion.
 */
static int chg_get_vbus_uv(struct pm8150b_chg *chg, int *vbus_uv)
{
	int val, rc;

	if (!chg->vbus)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->vbus, &val);
	if (rc < 0)
		return rc;

	*vbus_uv = val;
	return 0;
}

/*
 * Take the current the plugged-in source is good for, and ask a QC2 adapter
 * for the higher voltage class.  Detection settles some time after the plug
 * event, so this runs again on every poll.  True is returned when the
 * verdict differs from the one already reported.
 */
static bool chg_update_input(struct pm8150b_chg *chg)
{
	struct chg_source src;
	bool changed;
	int vbus_uv, rc;

	rc = chg_read_source(chg, &src);
	if (rc < 0)
		return false;

	/*
	 * A handshake that starts over starts over at 5 V: the adapter
	 * dropped out of QC and the pulse count no longer describes the
	 * output.  Drop it so the next burst climbs from the truth and the
	 * voltage reported alongside is never a stale figure.
	 */
	if (!src.qc3 && !chg->qc3_boosting)
		chg->qc3_pulses = 0;

	/*
	 * The class register is not the wire.  When the input sits near the
	 * default while the class says otherwise, the adapter has dropped
	 * out: a load step sends a marginal source back to 5 V, and the
	 * class state is stale with it.  Two samples in a row are a real
	 * gap -- the adapter is still slewing up right after the walk --
	 * and then the class starts over from what the wire says, which
	 * re-arms the walk below.
	 */
	if (!chg->qc3_boosting && !chg_get_vbus_uv(chg, &vbus_uv)) {
		if (src.voltage_uv - vbus_uv > QC_CLASS_TOL_UV) {
			if (++chg->class_bad >= QC_CLASS_BAD_SAMPLES) {
				chg->class_bad = 0;
				if (src.qc3) {
					chg->qc3_pulses = 0;
					src.voltage_uv = 5000000;
				} else {
					src.voltage_uv = vbus_uv;
				}
			}
		} else {
			chg->class_bad = 0;
		}
	}

	changed = src.type != chg->src_type ||
		  src.voltage_uv != chg->src_voltage_uv;

	chg->src_qc3 = src.qc3;

	/* The verdict is what decides who may drive D+/D-. */
	chg_dpdm_policy(chg, &src);

	/*
	 * A limit set from userspace stays until it is cleared again, and
	 * the QC3 burst owns the input limit while it runs.
	 */
	if (!chg->icl_manual && !chg->qc3_boosting &&
	    src.icl_ua != chg->icl_ua) {
		rc = chg_set_icl(chg, src.icl_ua);
		if (rc < 0)
			dev_warn(chg->dev, "failed to limit input to %u uA\n",
				 src.icl_ua);
	}

	/*
	 * A QC2 adapter waits at 5 V to be asked for more, so ask.  The
	 * command is a one-shot on the wire, so keep asking while the
	 * answer is still the default; the pulse counter caps the answer at
	 * the 9 V class, which is all the 4.4 V cell needs and keeps the
	 * switcher well inside its range.
	 */
	if (src.qc2 && src.voltage_uv < 6000000) {
		rc = chg_write(chg, CMD_HVDCP_2, HVDCP_FORCE_9V);
		if (rc < 0)
			dev_warn(chg->dev,
				 "failed to ask the adapter for 9 V\n");
	}

	/*
	 * A QC3 adapter answers no static request; it walks 200 mV per D+
	 * pulse instead, see chg_qc3_work().  The pulse count doubles as
	 * the guard against doing that twice, and detection owns D+/D-
	 * while it runs.
	 */
	if (src.qc3 && !chg->qc3_boosting && !chg->detecting &&
	    chg->qc3_pulses < QC3_PULSES_FOR_9V)
		schedule_delayed_work(&chg->qc3_work, 0);

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
 * left to the hardware's source detection.  The charge limits are the
 * effective ones, so the profile's tables apply from the very first
 * charge; see chg_update_limits().
 */
static int chg_configure(struct pm8150b_chg *chg)
{
	int rc;

	rc = chg_set_float_voltage(chg, chg->eff_fv_uv);
	if (rc < 0)
		return dev_err_probe(chg->dev, rc, "float voltage\n");

	rc = chg_set_fcc(chg, chg->eff_fcc_ua);
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

/*
 * Find the table entry a reading falls in.  Readings outside the table
 * are clamped to its ends, and a move to a neighbouring entry only takes
 * effect once the reading is clear of the boundary by the table's
 * hysteresis, so a reading sitting on a boundary settles where it first
 * landed instead of flapping between the two entries.
 */
static int chg_table_lookup(struct chg_table *tbl, int reading, int hyst,
			    int *val)
{
	int i, prev;

	if (!tbl->count)
		return -ENODATA;

	for (i = 0; i < tbl->count; i++) {
		if (reading >= tbl->entry[i].low &&
		    reading <= tbl->entry[i].high)
			break;
	}

	if (i == tbl->count)
		i = reading < tbl->entry[0].low ? 0 : tbl->count - 1;

	prev = tbl->index;
	if (prev >= 0) {
		if (i == prev + 1 && reading < tbl->entry[i].low + hyst)
			i = prev;
		else if (i == prev - 1 && reading > tbl->entry[i].high - hyst)
			i = prev;
	}

	tbl->index = i;
	*val = tbl->entry[i].val;

	return 0;
}

/* --------------------------- charge pump ----------------------------- */

/*
 * The pump only clears the cell from the 9 V class: at the 5 V class a
 * 2:1 divider cannot make the window above the battery.  The class
 * register says what was asked of the adapter; the wire has to agree
 * before the pump is started on it -- but within the class tolerance, as
 * a loaded 9 V line sits a little below its nominal figure.  A board
 * without the ADC has only the class to go by.
 */
static bool chg_cp_vbus_ok(struct pm8150b_chg *chg)
{
	int vbus_uv;

	if (chg_get_vbus_uv(chg, &vbus_uv))
		return true;

	return vbus_uv >= CP_INPUT_MIN_UV - QC_CLASS_TOL_UV;
}

/* Whether the pump is actually carrying the charge right now. */
static bool chg_cp_converting(struct pm8150b_chg *chg)
{
	return chg->cp_active && regulator_is_enabled(chg->cp) > 0;
}

/*
 * The pump's input current limit is the smaller of what the source may
 * deliver and half the charge current, the pump being a 2:1 divider.  A
 * zero answer means the input limit is not known, which is not a state
 * the pump may be started in.
 */
static unsigned int chg_cp_ilim_ua(struct pm8150b_chg *chg)
{
	unsigned int icl_ua = chg->icl_ua;

	if (!icl_ua && chg_get_icl(chg, &icl_ua))
		return 0;

	return min(icl_ua, chg->eff_fcc_ua / 2);
}

static void chg_cp_update_ilim(struct pm8150b_chg *chg)
{
	unsigned int ilim_ua = chg_cp_ilim_ua(chg);
	int rc;

	if (ilim_ua == chg->cp_ilim_ua)
		return;

	rc = regulator_set_current_limit(chg->cp, ilim_ua, ilim_ua);
	if (rc < 0) {
		dev_warn(chg->dev, "failed to limit the pump to %u uA\n",
			 ilim_ua);
		return;
	}

	chg->cp_ilim_ua = ilim_ua;
}

/*
 * Take the charge path over to the pump.  The pin that really starts it
 * belongs to the hardware: SMB_EN is routed to the pump and raised by the
 * PMIC while charging is active, so arming the switcher underneath it is
 * all software has to do.  On the way out the pin is parked low, so a
 * charger that has not armed the pump never feeds it.
 */
static int chg_cp_enable(struct pm8150b_chg *chg)
{
	unsigned int ilim_ua = chg_cp_ilim_ua(chg);
	int rc;

	/*
	 * The pump's limit is programmed first: a pump that cannot be
	 * limited is never started, and the pin is only routed over after
	 * that is settled.
	 */
	rc = regulator_set_current_limit(chg->cp, ilim_ua, ilim_ua);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, MISC_SMB_CFG, SMB_EN_SEL, SMB_EN_SEL);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, MISC_SMB_EN_CMD,
			      SMB_EN_OVERRIDE | SMB_EN_OVERRIDE_VALUE, 0);
	if (rc < 0)
		return rc;

	rc = chg_masked_write(chg, MISC_SMB_EN_CMD, EN_CP_CMD, EN_CP_CMD);
	if (rc < 0)
		goto rollback;

	rc = regulator_enable(chg->cp);
	if (rc < 0)
		goto rollback;

	chg->cp_active = true;
	chg->cp_ilim_ua = ilim_ua;
	chg->cp_arm_jiff = jiffies;
	dev_info(chg->dev, "charge pump active (ILIM %u uA)\n", ilim_ua);
	return 0;

rollback:
	chg_masked_write(chg, MISC_SMB_EN_CMD,
			 EN_CP_CMD | SMB_EN_OVERRIDE | SMB_EN_OVERRIDE_VALUE,
			 SMB_EN_OVERRIDE);
	return rc;
}

static void chg_cp_disable(struct pm8150b_chg *chg)
{
	int rc;

	if (!chg->cp_active)
		return;

	rc = regulator_disable(chg->cp);
	if (rc < 0)
		dev_warn(chg->dev, "failed to stop the charge pump\n");

	chg_masked_write(chg, MISC_SMB_EN_CMD, EN_CP_CMD, 0);
	chg_masked_write(chg, MISC_SMB_EN_CMD,
			 SMB_EN_OVERRIDE | SMB_EN_OVERRIDE_VALUE,
			 SMB_EN_OVERRIDE);
	chg_masked_write(chg, MISC_SMB_CFG, SMB_EN_SEL, 0);

	chg->cp_active = false;
	chg->cp_ilim_ua = 0;
	dev_info(chg->dev, "charge pump inactive\n");
}

/*
 * Combine the profile's JEITA and step-charging tables into the charge
 * limits in effect: the tables only ever lower the profile maxima, and
 * only JEITA derates the float voltage.  A derated float marks the warm
 * band, where charging is held off at the derated top instead of being
 * left to cycle around it; see JEITA_HOLD_HYST_UV.
 */
static bool chg_update_limits(struct pm8150b_chg *chg)
{
	unsigned int fcc_ua, fv_uv = chg->fv_uv;
	bool changed = false;
	int vbat_uv, val;

	/*
	 * While the pump carries the charge, the cell's charge pump
	 * ceiling replaces the buck ceiling; see the battery profile's
	 * qcom,charge-pump-fcc-ua.  The ceiling only applies while the
	 * pump is really converting: a buck charger left on its own at the
	 * 9 V class is what the source's power budget is measured for.  The
	 * tables derate either the same way.
	 */
	fcc_ua = chg_cp_converting(chg) && chg->cp_fcc_ua ? chg->cp_fcc_ua
							  : chg->fcc_ua;

	if (!chg_table_lookup(&chg->jeita_fcc, chg->temp_deci,
			      JEITA_HYST_DECI_C, &val))
		fcc_ua = min_t(unsigned int, fcc_ua, val);

	if (!chg_table_lookup(&chg->jeita_fv, chg->temp_deci,
			      JEITA_HYST_DECI_C, &val))
		fv_uv = min_t(unsigned int, fv_uv, val);

	if (!fg_get_voltage_uv(chg, &vbat_uv)) {
		if (!chg_table_lookup(&chg->step_fcc, vbat_uv,
				      STEP_HYST_UV, &val))
			fcc_ua = min_t(unsigned int, fcc_ua, val);

		if (fv_uv < chg->fv_uv) {
			if (vbat_uv > fv_uv)
				chg->warm_hold = true;
			else if (vbat_uv < fv_uv - JEITA_HOLD_HYST_UV)
				chg->warm_hold = false;
		} else {
			chg->warm_hold = false;
		}
	}

	if (fcc_ua != chg->eff_fcc_ua && !chg_set_fcc(chg, fcc_ua)) {
		chg->eff_fcc_ua = fcc_ua;
		changed = true;
	}

	if (fv_uv != chg->eff_fv_uv && !chg_set_float_voltage(chg, fv_uv)) {
		chg->eff_fv_uv = fv_uv;
		changed = true;
	}

	/* The pump's input limit follows the charge current above. */
	if (chg->cp_active)
		chg_cp_update_ilim(chg);

	return changed;
}

/*
 * Arming the pump is not the same as carrying the charge: the hardware
 * pin that starts it only rises while charging is active.  A pump that is
 * armed but not running while the charger is pushing current has been
 * stopped by its protections or never came up; drop it and let the pump
 * policy in the poll arm it again, and one still latched off refuses that
 * too, leaving the buck charger carrying the charge at its own limits.
 */
static bool chg_cp_check(struct pm8150b_chg *chg)
{
	bool converting = chg_cp_converting(chg);
	bool changed = false;
	int running;

	/*
	 * The charge ceiling is released for real conversion only, and the
	 * switcher crosses in and out of it on the hardware pin, without
	 * the driver being asked -- the pin drops on its own at charge
	 * termination, say.  Follow every crossing so that a ceiling meant
	 * for the pump is never left sitting on the buck.
	 */
	if (converting != chg->cp_converting) {
		chg->cp_converting = converting;
		if (chg_update_limits(chg))
			changed = true;
	}

	if (!chg->cp_active || chg->status != POWER_SUPPLY_STATUS_CHARGING ||
	    time_before(jiffies, chg->cp_retry_jiff) ||
	    time_before(jiffies, chg->cp_arm_jiff +
				  msecs_to_jiffies(CP_SS_GRACE_MS)))
		return changed;

	running = regulator_is_enabled(chg->cp);
	if (running == 0) {
		/*
		 * Armed and still not carrying the charge: the load did
		 * not take the switcher up.  Rest before the policy below
		 * arms it again, or a load too light for the pump turns
		 * into an arm and trip loop.
		 */
		chg_cp_disable(chg);
		chg->cp_retry_jiff = jiffies + msecs_to_jiffies(CP_RETRY_MS);
	}

	return changed;
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
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/*
		 * Sampled in the background alongside the temperature, for
		 * the same reason: the gauge's DMA window blocks for longer
		 * than a property getter is allowed to.
		 */
		if (!chg->current_valid)
			return -EAGAIN;
		val->intval = chg->current_ua;
		rc = 0;
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
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		/*
		 * The limit in effect, after the profile's charge tables
		 * have spoken; the maximum stays in the battery description.
		 */
		val->intval = chg->eff_fcc_ua;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = chg->eff_fv_uv;
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
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
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

/*
 * The Qualcomm handshake is a second pass over an already classified source,
 * and it is kept out of the first one: BC1.2 has to see the plain D+/D-
 * picture before the handshake starts moving the pair around.  It is turned
 * on shortly after a source appears -- the vendor driver waits 700 ms for
 * the input to settle -- and off again when the source goes away, so every
 * plug starts from the plain classification.
 */
static void chg_hvdcp_enable(struct pm8150b_chg *chg, bool enable)
{
	unsigned int mask = HVDCP_AUTH_ALG_EN | HVDCP_EN;
	int rc;

	rc = chg_masked_write(chg, USBIN_OPTIONS_1_CFG, mask,
			      enable ? mask : 0);
	if (rc < 0)
		dev_warn(chg->dev, "failed to %s the QC handshake\n",
			 enable ? "enable" : "disable");
}

/*
 * Run source detection again with the handshake turned on.  Anything that
 * was classified before this runs -- a charger already plugged in while the
 * driver loaded, say -- never gets as far as the handshake without it.
 *
 * Detection drives D+/D-, so the verdict is waited out before anything
 * else may touch the pair: a QC3 burst racing a detection gets the
 * handshake torn down under it.  The verdict also lands in stages, BC1.2
 * first and the QC handshake seconds later, and only the final state may
 * be believed; see chg_read_source().
 */
static void chg_detect_work(struct work_struct *work)
{
	struct pm8150b_chg *chg =
		container_of(work, struct pm8150b_chg, detect_work.work);
	unsigned int stat, result;
	bool settling = false;
	int i, present, rc;

	if (!chg_usb_present(chg, &present) && present) {
		chg_hvdcp_enable(chg, true);

		rc = chg_write(chg, CMD_APSD, APSD_RERUN);
		if (rc < 0)
			dev_warn(chg->dev,
				 "failed to re-run source detection\n");

		/*
		 * Only a state reached after the detection has visibly been
		 * in flight counts as final: the registers still hold the
		 * previous verdict until the re-run takes effect.  A source
		 * that speaks no QC settles as soon as BC1.2 is done and the
		 * QC check has timed out; a QC one only once authenticated.
		 */
		for (i = 0; i < 50; i++) {
			if (chg_read(chg, APSD_STATUS, &stat) ||
			    chg_read(chg, APSD_RESULT_STATUS, &result))
				break;

			if (!(stat & APSD_DTC_DONE)) {
				settling = true;
			} else if (settling &&
				   (!(result & (DCP_CHARGER | OCP_CHARGER)) ||
				    (stat & HVDCP_CHECK_TIMEOUT)) &&
				   (!(stat & QC_CHARGER) ||
				    (stat & QC_AUTH_DONE))) {
				break;
			}

			msleep(200);
		}
	}

	chg->detecting = delayed_work_pending(&chg->detect_work);
	mod_delayed_work(system_percpu_wq, &chg->status_work, 0);
}

/*
 * The switcher frequency follows the pulses up the ladder, the same way
 * the vendor driver walks it: the frequency is set for the step about to
 * be sent, so the switcher is already in the right gear when the new
 * input voltage arrives.
 */
static void chg_qc3_set_fsw(struct pm8150b_chg *chg, int pulses)
{
	unsigned int raw;

	if (pulses < QC3_PULSES_FOR_6V)
		raw = FSW_1200_KHZ;
	else if (pulses < QC3_PULSES_FOR_9V)
		raw = FSW_1600_KHZ;
	else
		raw = FSW_2100_KHZ;

	chg_write(chg, DCDC_FSW_SEL, raw);
}

/*
 * Walk a QC3 adapter up to the 9 V class, one 200 mV D+ pulse at a time
 * and with the interval the pulses are allowed to come at.  The input
 * current is held back while the adapter walks, see ICL_QC3_BOOST_UA.  A
 * source that disappears mid-burst gets the rest of the pulses cancelled
 * with it, and the unplug resets the count, so a new source always starts
 * from 5 V.
 */
static void chg_qc3_work(struct work_struct *work)
{
	struct pm8150b_chg *chg =
		container_of(work, struct pm8150b_chg, qc3_work.work);
	int present, rc;

	chg->qc3_boosting = true;
	rc = chg_set_icl(chg, ICL_QC3_BOOST_UA);
	if (rc < 0)
		dev_warn(chg->dev,
			 "failed to back the input off for the boost\n");

	while (chg->qc3_pulses < QC3_PULSES_FOR_9V) {
		if (chg_usb_present(chg, &present) || !present)
			break;

		chg_qc3_set_fsw(chg, chg->qc3_pulses + 1);

		rc = chg_masked_write(chg, CMD_HVDCP_2, HVDCP_SINGLE_INC,
				      HVDCP_SINGLE_INC);
		if (rc < 0) {
			dev_warn(chg->dev,
				 "failed to pulse the adapter (%d)\n", rc);
			break;
		}

		chg->qc3_pulses++;
		msleep(40);
	}

	chg->qc3_boosting = false;

	/* The new class wants reporting and a fresh input limit. */
	mod_delayed_work(system_percpu_wq, &chg->status_work, 0);
}

static void chg_status_work(struct work_struct *work)
{
	struct pm8150b_chg *chg =
		container_of(work, struct pm8150b_chg, status_work.work);
	bool changed = false;
	bool allow = true;
	bool want_cp;
	unsigned int delay_ms;
	int status, online, present, capacity, temp;
	int current_ua;

	if (!chg_usb_online(chg, &online) && online != chg->usb_online) {
		chg->usb_online = online;
		changed = true;
	}

	/*
	 * When a source appears, take D+/D- over from the gadget and run the
	 * QC handshake over the classification once the input has settled;
	 * see chg_detect_work().  Nothing needs poking on the way out beyond
	 * dropping the handshake again, and the pair can rest where it is
	 * until the next source shows up.
	 */
	if (!chg_usb_present(chg, &present) && present != chg->usb_present) {
		chg_dpdm_float(chg, true);
		if (present) {
			chg->detecting = true;
			schedule_delayed_work(&chg->detect_work,
					      msecs_to_jiffies(700));
		} else {
			chg_cp_disable(chg);
			chg_hvdcp_enable(chg, false);
			cancel_delayed_work_sync(&chg->qc3_work);
			chg->qc3_pulses = 0;
		}
		chg->usb_present = present;
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

	if (!fg_read_current_ua(chg, &current_ua)) {
		if (!chg->current_valid || current_ua != chg->current_ua) {
			chg->current_ua = current_ua;
			chg->current_valid = true;
			changed = true;
		}
	}

	/*
	 * The temperature band is the union of the JEITA table's ranges:
	 * inside it the tables derate the charge limits as temperature and
	 * voltage move, outside it charging is held off entirely.
	 */
	if (chg->temp_valid) {
		if (chg->temp_deci >= TEMP_MIN_DECI_C &&
		    chg->temp_deci <= TEMP_MAX_DECI_C) {
			chg->temp_bad = 0;
			if (chg_update_limits(chg))
				changed = true;
			if (chg->warm_hold)
				allow = false;
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

	/*
	 * The charge pump takes the charge path over once the source has
	 * been walked up to the 9 V class and comes off again below it, on
	 * a plug out, or while charging is held off: the pump is a 2:1
	 * divider and only clears the cell at the higher input class.  It
	 * is armed for charging only: an armed switcher left idle has its
	 * start pin pulsed at it whenever the charger twitches around
	 * termination, and a start into no load only trips the switcher's
	 * window protection.  Its input limit has to be in the pump's
	 * accurate range to be worth starting for, see chg_cp_ilim_ua().
	 */
	if (chg_cp_check(chg))
		changed = true;

	want_cp = chg->cp && chg->usb_present && chg->src_qc3 &&
		  chg->status == POWER_SUPPLY_STATUS_CHARGING &&
		  chg->src_voltage_uv >= CP_INPUT_MIN_UV &&
		  chg_cp_vbus_ok(chg) && chg->charge_allowed &&
		  chg_cp_ilim_ua(chg) >= CP_ILIM_MIN_UA;

	/* A pump that refused to start waits out its backoff, see above. */
	if (want_cp && !chg->cp_active &&
	    time_before(jiffies, chg->cp_retry_jiff))
		want_cp = false;

	if (want_cp != chg->cp_active) {
		bool ok;

		if (want_cp) {
			ok = !chg_cp_enable(chg);
			if (!ok) {
				chg->cp_retry_jiff = jiffies +
					msecs_to_jiffies(CP_RETRY_MS);
				dev_warn(chg->dev,
					 "failed to start the charge pump\n");
			}
		} else {
			chg_cp_disable(chg);
			ok = true;
		}

		if (ok) {
			/* The charge ceiling moves with the pump. */
			chg_update_limits(chg);
			changed = true;
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
	{ "usbin-src-change",		chg_plugin_irq },
	{ "chg-state-change",		chg_state_irq },
};

/* ------------------------------- probe -------------------------------- */

/*
 * Parse one of the battery profile's charge tables: triplets of low
 * bound, high bound and the value the cell asks for between them, in the
 * profile's own units.  A battery may describe any subset of the tables;
 * what is missing is simply not a derating source.  A table that is
 * present but malformed is refused, since guessing its intent could
 * charge the cell outside the profile.
 */
static int chg_read_table(struct device *dev, struct device_node *np,
			  const char *name, struct chg_table *tbl)
{
	u32 data[CHG_TABLE_MAX * 3];
	int count, i, rc;

	count = of_property_count_elems_of_size(np, name, sizeof(u32));
	if (count == -EINVAL)
		return 0;
	if (count < 0)
		return dev_err_probe(dev, count, "%s\n", name);

	if (!count || count % 3 || count / 3 > CHG_TABLE_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "%s: expected up to %d <low high value> triplets\n",
				     name, CHG_TABLE_MAX);

	rc = of_property_read_u32_array(np, name, data, count);
	if (rc < 0)
		return dev_err_probe(dev, rc, "%s\n", name);

	tbl->count = count / 3;
	tbl->index = -1;
	for (i = 0; i < tbl->count; i++) {
		tbl->entry[i].low = data[i * 3];
		tbl->entry[i].high = data[i * 3 + 1];
		tbl->entry[i].val = data[i * 3 + 2];
		dev_dbg(dev, "%s: %d..%d -> %d\n", name, tbl->entry[i].low,
			tbl->entry[i].high, tbl->entry[i].val);
	}

	dev_info(dev, "%s: %d entries\n", name, tbl->count);
	return 0;
}

/*
 * The charge limits come from the battery's own description, so they stay
 * with whichever cell is fitted rather than with the board.  The JEITA
 * and step-charging breakpoints belong to the same description: they are
 * what the cell vendor's profile asks for at each temperature and
 * voltage, and travel with the cell.
 */
static int chg_read_battery_limits(struct pm8150b_chg *chg)
{
	struct power_supply_battery_info *info;
	struct device_node *np;
	int rc;

	rc = power_supply_get_battery_info(chg->battery, &info);
	if (rc)
		return dev_err_probe(chg->dev, rc, "battery description\n");

	chg->fv_uv = info->constant_charge_voltage_max_uv;
	chg->fcc_ua = info->constant_charge_current_max_ua;

	power_supply_put_battery_info(chg->battery, info);

	np = of_parse_phandle(chg->dev->of_node, "monitored-battery", 0);
	if (np) {
		rc = chg_read_table(chg->dev, np, "qcom,jeita-fcc-ranges",
				    &chg->jeita_fcc);
		if (!rc)
			rc = chg_read_table(chg->dev, np,
					    "qcom,jeita-fv-ranges",
					    &chg->jeita_fv);
		if (!rc)
			rc = chg_read_table(chg->dev, np, "qcom,step-chg-ranges",
					    &chg->step_fcc);

		/*
		 * The cell takes more current while the charge pump
		 * carries the charge; the figure lives with the cell's
		 * other tables, and is optional.
		 */
		of_property_read_u32(np, "qcom,charge-pump-fcc-ua",
				     &chg->cp_fcc_ua);
		of_node_put(np);
		if (rc)
			return rc;
	}

	if (chg->cp_fcc_ua > 8000000)
		return dev_err_probe(chg->dev, -EINVAL,
				     "charge pump charge current %u uA out of range\n",
				     chg->cp_fcc_ua);

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

	/* The tables only ever lower these, see chg_update_limits(). */
	chg->eff_fv_uv = chg->fv_uv;
	chg->eff_fcc_ua = chg->fcc_ua;

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
 * chg_read_source() to pick up, which is what makes charging work before the
 * driver has even probed.  The QC handshake is not part of this pass; it is
 * layered on top of a settled classification later, see chg_detect_work().
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
	unsigned int hvdcp = HVDCP_AUTH_ALG_EN | HVDCP_EN;
	int rc;

	rc = chg_masked_write(chg, USBIN_AICL_OPTIONS_CFG,
			      SUSPEND_ON_COLLAPSE_USBIN, 0);
	if (rc < 0)
		return rc;

	/*
	 * Start the detection with the input rather than with the CC attach.
	 * On a legacy cable the two events do not coincide, and detection
	 * launched from CC runs before the input has settled.
	 */
	rc = chg_masked_write(chg, TYPE_C_CFG, BC1P2_START_ON_CC, 0);
	if (rc < 0)
		return rc;

	/* Input-collapse filtering and AICL step timing, as the vendor sets. */
	rc = chg_masked_write(chg, USBIN_LOAD_CFG,
			      USBIN_IN_COLLAPSE_GF_SEL |
			      USBIN_AICL_STEP_TIMING_SEL, 0);
	if (rc < 0)
		return rc;

	return chg_masked_write(chg, USBIN_OPTIONS_1_CFG,
				hvdcp | BC1P2_SRC_DETECT, BC1P2_SRC_DETECT);
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

	/*
	 * Re-run detection with the QC handshake on: a source plugged in
	 * before the driver loaded is otherwise never handshaked.  It is
	 * queued ahead of the input update so a stale verdict cannot start
	 * the QC3 boost before the fresh detection has had the pair.
	 */
	chg->detecting = true;
	schedule_delayed_work(&chg->detect_work, msecs_to_jiffies(700));

	/* Take whatever the attached source is good for. */
	chg_update_input(chg);

	/* Charging is enabled; only the battery's policy can hold it off. */
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

	/*
	 * Optional: without the supply there is nobody to hand D+/D- to,
	 * and the gadget keeps the pair whatever the charger thinks.
	 */
	chg->dpdm = devm_regulator_get_optional(chg->dev, "dpdm");
	if (IS_ERR(chg->dpdm)) {
		if (PTR_ERR(chg->dpdm) != -ENODEV)
			return dev_err_probe(chg->dev, PTR_ERR(chg->dpdm),
					     "dpdm supply\n");
		chg->dpdm = NULL;
	}

	/*
	 * Optional: boards without an SMB1390 charge pump simply charge
	 * through the buck converter.
	 */
	chg->cp = devm_regulator_get_optional(chg->dev, "charge-pump");
	if (IS_ERR(chg->cp)) {
		if (PTR_ERR(chg->cp) != -ENODEV)
			return dev_err_probe(chg->dev, PTR_ERR(chg->cp),
					     "charge-pump supply\n");
		chg->cp = NULL;
	}

	/*
	 * Optional: the measured input voltage is what keeps the charge
	 * pump honest about the class the adapter is really at, see
	 * chg_get_vbus_uv().
	 */
	chg->vbus = devm_iio_channel_get(chg->dev, "vbus");
	if (IS_ERR(chg->vbus)) {
		if (PTR_ERR(chg->vbus) != -ENODEV)
			return dev_err_probe(chg->dev, PTR_ERR(chg->vbus),
					     "vbus ADC\n");
		chg->vbus = NULL;
	}

	INIT_DEFERRABLE_WORK(&chg->status_work, chg_status_work);
	INIT_DEFERRABLE_WORK(&chg->detect_work, chg_detect_work);
	INIT_DEFERRABLE_WORK(&chg->qc3_work, chg_qc3_work);

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
	 * The IRQs feed the pollers below and the pollers re-arm each
	 * other, so the IRQs go first: disable_irq() waits out a handler
	 * that is in flight and, with them off, nothing can queue work once
	 * the cancels have drained what is pending.  Work running past this
	 * point would chase freed driver state, and might even hand D+/D-
	 * back after it was given up below.
	 */
	for (i = 0; i < ARRAY_SIZE(chg_irqs); i++)
		if (chg->irqs[i] > 0)
			disable_irq(chg->irqs[i]);

	cancel_delayed_work_sync(&chg->detect_work);
	cancel_delayed_work_sync(&chg->qc3_work);
	cancel_delayed_work_sync(&chg->status_work);

	/*
	 * Give the charge pump and D+/D- back before the devm-managed
	 * supplies are released: the regulator core complains about being
	 * put while still enabled.
	 */
	chg_cp_disable(chg);
	chg_dpdm_float(chg, false);
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
MODULE_IMPORT_NS("IIO_CONSUMER");
