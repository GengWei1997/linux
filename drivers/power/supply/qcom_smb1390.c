// SPDX-License-Identifier: GPL-2.0-only
/*
 * SMB1390 switched-capacitor charge pump driver
 *
 * The SMB1390 is a 2:1 switched-capacitor divider sitting between the
 * adapter and the charger's system rail: fed 9 V it tracks VPH at about
 * 4.5 V, which halves the current the buck charger has to convert and
 * keeps the charge path cool while the cell takes several amps.
 *
 * The chip is reached over I2C with 16 bit register addresses.  It runs
 * unattended once armed: its own window, over-voltage, reverse-current
 * and over-temperature protections latch the switcher off without
 * software help.  All software has to do is arm the switcher and set the
 * input current limit.  The protections report on one interrupt wire to
 * the PMIC charger; this driver reads which of the chip's fault sources
 * fired from the chip itself and leaves the switcher off afterwards, so
 * charging falls back to the buck charger.
 *
 * The charge path around the pump is driven from the charger side: the
 * SMB_EN hardware pin that actually starts the pump belongs to the PMIC
 * charger, and the consumer end of this driver's regulator is what arms
 * the switcher underneath it.
 *
 * This is a clean-room driver written from the register map as used by
 * the Qualcomm vendor drivers; none of that code is copied here.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/types.h>

/* Status registers */
#define SMB1390_CORE_STATUS1		0x1006
#define SMB1390_WIN_OV			BIT(0)
#define SMB1390_WIN_UV			BIT(1)
#define SMB1390_EN_PIN_OUT		BIT(2)
#define SMB1390_LCM_AUTO		BIT(3)
#define SMB1390_LCM_PIN			BIT(4)
#define SMB1390_ILIM			BIT(5)
#define SMB1390_TEMP_ALARM		BIT(6)
#define SMB1390_VPH_OV_SOFT		BIT(7)

#define SMB1390_CORE_STATUS2		0x1007
#define SMB1390_SWITCHER_HOLD_OFF	BIT(0)
#define SMB1390_VPH_OV_HARD		BIT(1)
#define SMB1390_TSD			BIT(2)
#define SMB1390_IREV			BIT(3)
#define SMB1390_IOC			BIT(4)
#define SMB1390_VIN_UV			BIT(5)
#define SMB1390_VIN_OV			BIT(6)
#define SMB1390_EN_PIN_OUT2		BIT(7)

#define SMB1390_CORE_STATUS3		0x1008
#define SMB1390_EN_SL			BIT(0)
#define SMB1390_IIN_REF_SS_DONE		BIT(1)
#define SMB1390_FLYCAP_SS_DONE		BIT(2)
#define SMB1390_SL_DETECTED		BIT(3)

/* Both soft-start steps done: the switcher is carrying the charge. */
#define SMB1390_SS_DONE_MASK		(SMB1390_IIN_REF_SS_DONE | \
					 SMB1390_FLYCAP_SS_DONE)

/* Interrupt block: one bit per fault source, latched until cleared */
#define SMB1390_CORE_INT_RT_STS		0x1010
#define SMB1390_SW_OFF_WINDOW_STS	BIT(0)
#define SMB1390_SW_OFF_FAULT_STS	BIT(1)
#define SMB1390_TSD_STS			BIT(2)
#define SMB1390_IREV_STS		BIT(3)
#define SMB1390_VPH_OV_HARD_STS		BIT(4)
#define SMB1390_VPH_OV_SOFT_STS		BIT(5)
#define SMB1390_ILIM_STS		BIT(6)
#define SMB1390_TEMP_ALARM_STS		BIT(7)

#define SMB1390_CORE_INT_SET_TYPE	0x1011
#define SMB1390_CORE_INT_POL_HIGH	0x1012
#define SMB1390_CORE_INT_POL_LOW	0x1013
#define SMB1390_CORE_INT_LATCHED_CLR	0x1014
#define SMB1390_CORE_INT_EN_SET		0x1015
#define SMB1390_CORE_INT_EN_CLR		0x1016
#define SMB1390_CORE_INT_LATCHED_STS	0x1018

#define SMB1390_INT_ALL			GENMASK(7, 0)

/*
 * The interrupt sources worth stopping for.  The input-current warning
 * is left masked out: it fires routinely while the pump is working at
 * its limit, and the input limit is the charger's business anyway.
 */
#define SMB1390_FAULT_MASK		(SMB1390_SW_OFF_WINDOW_STS | \
					 SMB1390_SW_OFF_FAULT_STS | \
					 SMB1390_TSD_STS | \
					 SMB1390_IREV_STS | \
					 SMB1390_VPH_OV_HARD_STS | \
					 SMB1390_VPH_OV_SOFT_STS | \
					 SMB1390_TEMP_ALARM_STS)

/* Control registers */
#define SMB1390_CORE_CONTROL1		0x1020
#define SMB1390_CMD_EN_SWITCHER		BIT(0)
#define SMB1390_CMD_EN_SL		BIT(1)

/*
 * Trims.  The input current limit runs from 500 mA in 100 mA steps, in
 * the low five bits of its trim register.
 */
#define SMB1390_CORE_FTRIM_ILIM		0x1030
#define SMB1390_ILIM_RAW_MASK		GENMASK(4, 0)
#define SMB1390_ILIM_MIN_UA		500000
#define SMB1390_ILIM_STEP_UA		100000

#define SMB1390_CORE_FTRIM_LVL		0x1033
#define SMB1390_CFG_WIN_HI_MASK		GENMASK(3, 2)
#define SMB1390_WIN_OV_LVL_1000MV	0x08

#define SMB1390_CORE_FTRIM_MISC		0x1034
#define SMB1390_TR_WIN_1P5X		BIT(0)
#define SMB1390_WIN_DELTA_X1P0		0

struct smb1390 {
	struct device		*dev;
	struct regmap		*regmap;
};

static const struct regmap_config smb1390_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

/* ------------------------- charge pump ------------------------------- */

static int smb1390_enable(struct regulator_dev *rdev)
{
	struct smb1390 *chip = rdev_get_drvdata(rdev);
	unsigned int status1, status2;
	int rc;

	/*
	 * Do not arm a switcher that still reports the condition that
	 * stopped it: a chip latched off by its protections answers every
	 * arm request the same way, and the charger retries once the
	 * condition has cleared.  The window warning counts too: it is what
	 * a 2:1 divider reports when the input is too low to clear the
	 * battery, and a start there only fails again.
	 */
	rc = regmap_read(chip->regmap, SMB1390_CORE_STATUS1, &status1);
	if (rc < 0)
		return rc;

	rc = regmap_read(chip->regmap, SMB1390_CORE_STATUS2, &status2);
	if (rc < 0)
		return rc;

	if ((status1 & (SMB1390_TEMP_ALARM | SMB1390_VPH_OV_SOFT |
			SMB1390_WIN_UV)) ||
	    (status2 & (SMB1390_VPH_OV_HARD | SMB1390_TSD | SMB1390_IREV)))
		return -EBUSY;

	return regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1,
				  SMB1390_CMD_EN_SWITCHER,
				  SMB1390_CMD_EN_SWITCHER);
}

static int smb1390_disable(struct regulator_dev *rdev)
{
	struct smb1390 *chip = rdev_get_drvdata(rdev);

	return regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1,
				  SMB1390_CMD_EN_SWITCHER, 0);
}

/*
 * Whether the switcher is actually carrying the charge.  Arming the pump
 * only makes it ready: the hardware pin that really starts it is raised
 * while charging is active, and the soft-start bits say the switcher took
 * over.  Consumers use this to tell a running pump from an idle one.
 */
static int smb1390_is_enabled(struct regulator_dev *rdev)
{
	struct smb1390 *chip = rdev_get_drvdata(rdev);
	unsigned int val;
	int rc;

	rc = regmap_read(chip->regmap, SMB1390_CORE_STATUS3, &val);
	if (rc < 0)
		return rc;

	return (val & SMB1390_SS_DONE_MASK) == SMB1390_SS_DONE_MASK;
}

/*
 * The input current limit, in microamps.  The encoding is a step count
 * from 500 mA, and the closest step to the request is what the register
 * takes.
 */
static int smb1390_set_current_limit(struct regulator_dev *rdev,
				     int min_uA, int max_uA)
{
	struct smb1390 *chip = rdev_get_drvdata(rdev);
	unsigned int raw;

	if (max_uA < SMB1390_ILIM_MIN_UA)
		return -EINVAL;

	raw = DIV_ROUND_CLOSEST(max_uA - SMB1390_ILIM_MIN_UA,
				SMB1390_ILIM_STEP_UA);
	if (raw > SMB1390_ILIM_RAW_MASK)
		raw = SMB1390_ILIM_RAW_MASK;

	return regmap_update_bits(chip->regmap, SMB1390_CORE_FTRIM_ILIM,
				  SMB1390_ILIM_RAW_MASK, raw);
}

static int smb1390_get_current_limit(struct regulator_dev *rdev)
{
	struct smb1390 *chip = rdev_get_drvdata(rdev);
	unsigned int raw;
	int rc;

	rc = regmap_read(chip->regmap, SMB1390_CORE_FTRIM_ILIM, &raw);
	if (rc < 0)
		return rc;

	return SMB1390_ILIM_MIN_UA +
	       (raw & SMB1390_ILIM_RAW_MASK) * SMB1390_ILIM_STEP_UA;
}

static const struct regulator_ops smb1390_regulator_ops = {
	.enable			= smb1390_enable,
	.disable		= smb1390_disable,
	.is_enabled		= smb1390_is_enabled,
	.set_current_limit	= smb1390_set_current_limit,
	.get_current_limit	= smb1390_get_current_limit,
};

static const struct regulator_desc smb1390_regulator = {
	.name		= "smb1390_cp",
	.type		= REGULATOR_CURRENT,
	.ops		= &smb1390_regulator_ops,
	.owner		= THIS_MODULE,
};

/* ------------------------- fault handling ---------------------------- */

static irqreturn_t smb1390_irq(int irq, void *data)
{
	struct smb1390 *chip = data;
	unsigned int latched, status;
	int rc;

	rc = regmap_read(chip->regmap, SMB1390_CORE_INT_LATCHED_STS, &latched);
	if (rc < 0)
		return IRQ_HANDLED;

	/*
	 * Clear what fired before anything else: the sources are latched,
	 * and the wire to the PMIC stays asserted for as long as any of
	 * them is pending.
	 */
	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_LATCHED_CLR,
			  SMB1390_INT_ALL);
	if (rc < 0)
		return IRQ_HANDLED;

	if (!(latched & SMB1390_FAULT_MASK))
		return IRQ_HANDLED;

	/*
	 * A protection has acted.  Leave the switcher off and let the
	 * charger notice, which drops the charge back to the buck charger;
	 * the pump is only re-armed once the chip reports clean again.
	 */
	regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1,
			   SMB1390_CMD_EN_SWITCHER, 0);

	if (regmap_read(chip->regmap, SMB1390_CORE_STATUS2, &status))
		dev_warn(chip->dev, "fault, latch %02x\n", latched);
	else
		dev_warn(chip->dev, "fault, latch %02x status2 %02x\n",
			 latched, status);

	return IRQ_HANDLED;
}

/*
 * The chip's fault sources share one wire to the PMIC, whose interrupt
 * is described in the device tree; all this side needs is the chip's own
 * interrupt block pointed at the faults worth stopping for.  The sources
 * are signalled as edges of either polarity, as the wire only reports
 * that something is pending and the latched register says what.
 */
static int smb1390_irq_init(struct smb1390 *chip, struct i2c_client *client)
{
	int rc;

	/* Start from a clean slate. */
	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_EN_CLR,
			  SMB1390_INT_ALL);
	if (rc < 0)
		return rc;

	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_LATCHED_CLR,
			  SMB1390_INT_ALL);
	if (rc < 0)
		return rc;

	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_SET_TYPE,
			  SMB1390_FAULT_MASK);
	if (rc < 0)
		return rc;

	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_POL_HIGH,
			  SMB1390_FAULT_MASK);
	if (rc < 0)
		return rc;

	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_POL_LOW,
			  SMB1390_FAULT_MASK);
	if (rc < 0)
		return rc;

	rc = regmap_write(chip->regmap, SMB1390_CORE_INT_EN_SET,
			  SMB1390_FAULT_MASK);
	if (rc < 0)
		return rc;

	return devm_request_threaded_irq(chip->dev, client->irq, NULL,
					 smb1390_irq, IRQF_ONESHOT,
					 "smb1390", chip);
}

/* ------------------------------- probe ------------------------------- */

/*
 * Configure the chip the way its input current limit's accuracy figures
 * are specified against, and leave the switcher disarmed: the charger
 * arms it when the charge path wants the pump.
 */
static int smb1390_init_hw(struct smb1390 *chip)
{
	int rc;

	/* Window (Vin - 2*Vout) over-voltage level to 1000 mV. */
	rc = regmap_update_bits(chip->regmap, SMB1390_CORE_FTRIM_LVL,
				SMB1390_CFG_WIN_HI_MASK,
				SMB1390_WIN_OV_LVL_1000MV);
	if (rc < 0)
		return rc;

	/* VOUT tracking delta to 1.0x. */
	rc = regmap_update_bits(chip->regmap, SMB1390_CORE_FTRIM_MISC,
				SMB1390_TR_WIN_1P5X, SMB1390_WIN_DELTA_X1P0);
	if (rc < 0)
		return rc;

	return regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1,
				  SMB1390_CMD_EN_SWITCHER, 0);
}

static int smb1390_probe(struct i2c_client *client)
{
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	struct smb1390 *chip;
	int rc;

	if (client->irq <= 0)
		return dev_err_probe(&client->dev, -EINVAL,
				     "no fault interrupt\n");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &smb1390_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(chip->regmap),
				     "regmap\n");

	rc = smb1390_init_hw(chip);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "hardware init\n");

	rc = smb1390_irq_init(chip, client);
	if (rc < 0)
		return dev_err_probe(chip->dev, rc, "interrupt\n");

	/*
	 * The regulator is the pump's contract with the charger: enabling
	 * it arms the switcher, and its current limit is the pump's input
	 * limit.  The operating range comes from the device tree, which
	 * this driver parses from its own node the way the fixed regulator
	 * driver does; the regulator core only walks child nodes.
	 */
	config.dev = chip->dev;
	config.of_node = chip->dev->of_node;
	config.driver_data = chip;
	config.init_data = of_get_regulator_init_data(chip->dev,
						      chip->dev->of_node,
						      &smb1390_regulator);
	if (!config.init_data)
		return dev_err_probe(chip->dev, -EINVAL,
				     "regulator constraints\n");

	rdev = devm_regulator_register(chip->dev, &smb1390_regulator, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(chip->dev, PTR_ERR(rdev), "regulator\n");

	return 0;
}

static void smb1390_remove(struct i2c_client *client)
{
	struct smb1390 *chip = i2c_get_clientdata(client);

	/* Leave the pump disarmed behind us. */
	regmap_update_bits(chip->regmap, SMB1390_CORE_CONTROL1,
			   SMB1390_CMD_EN_SWITCHER, 0);
}

static const struct of_device_id smb1390_of_match[] = {
	{ .compatible = "qcom,smb1390" },
	{ }
};
MODULE_DEVICE_TABLE(of, smb1390_of_match);

static struct i2c_driver smb1390_driver = {
	.driver = {
		.name = "smb1390",
		.of_match_table = smb1390_of_match,
	},
	.probe = smb1390_probe,
	.remove = smb1390_remove,
};
module_i2c_driver(smb1390_driver);

MODULE_DESCRIPTION("SMB1390 charge pump driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
