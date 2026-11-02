// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8150B haptics driver
 *
 * The PM8150B contains an H-bridge with a DAC that drives a vibration
 * actuator.  The DAC plays a waveform clocked by the play-rate generator;
 * this driver only uses the direct-play mode, where the waveform is a
 * constant level and the output amplitude selects the vibration strength.
 * That is what an ERM actuator needs and what the force-feedback rumble
 * interface provides.
 *
 * Copyright (c) 2026 GengWei1997 <gw19970326@gmail.com>
 */

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* Peripheral type register, used to check we probed the right block. */
#define HAP_TYPE			0x15
#define HAP_REG_TYPE			0x04

#define HAP_EN_CTL1			0x46
#define HAP_EN				BIT(7)

#define HAP_EN_CTL3			0x4a
#define HAP_HBRIDGE_EN			BIT(7)
#define HAP_PWM_SIGNAL_EN		BIT(6)
#define HAP_ILIM_EN			BIT(5)
#define HAP_ILIM_CC_EN			BIT(4)
#define HAP_AUTO_RES_RBIAS_EN		BIT(3)
#define HAP_DAC_EN			BIT(2)
#define HAP_ZX_HYST_EN			BIT(1)
#define HAP_PWM_CTL_EN			BIT(0)

#define HAP_AUTO_RES_CTRL		0x4b
#define HAP_AUTO_RES_EN			BIT(7)

#define HAP_CFG1			0x4c
#define HAP_ACT_ERM			1

#define HAP_SEL				0x4e
#define HAP_WF_SOURCE_MASK		GENMASK(5, 4)
#define HAP_WF_SOURCE_SHIFT		4
#define HAP_WF_TRIGGER			BIT(0)
#define HAP_WF_SOURCE_VMAX		0

#define HAP_VMAX_CFG			0x51
#define HAP_VMAX_MV_MASK		GENMASK(5, 1)
#define HAP_VMAX_MV_SHIFT		1
#define HAP_VMAX_MV_LSB			116
#define HAP_VMAX_MV_MAX			(31 * HAP_VMAX_MV_LSB)
#define HAP_VMAX_MV_DEFAULT		3000

#define HAP_ILIM_CFG			0x52
#define HAP_ILIM_1000MA			BIT(1)

#define HAP_RATE_CFG1			0x54
#define HAP_RATE_CFG2			0x55
#define HAP_RATE_US_LSB			5
#define HAP_RATE_US_DEFAULT		5715
#define HAP_RATE_US_MAX			20475

#define HAP_PLAY			0x70
#define HAP_PLAY_BIT			BIT(7)

/* force-feedback reports the magnitude as a 16-bit fraction */
#define HAP_MAGNITUDE_MAX		0xffff

struct qcom_hap {
	struct device *dev;
	struct regmap *regmap;
	struct work_struct work;

	unsigned int base;
	unsigned int vmax_mv;
	unsigned int play_rate_us;

	u16 magnitude;
	bool playing;
	bool suspended;
};

static int qcom_hap_set_level(struct qcom_hap *hap, u16 magnitude)
{
	unsigned int mv = hap->vmax_mv * magnitude / HAP_MAGNITUDE_MAX;

	return regmap_update_bits(hap->regmap,
				  hap->base + HAP_VMAX_CFG,
				  HAP_VMAX_MV_MASK,
				  (mv / HAP_VMAX_MV_LSB) << HAP_VMAX_MV_SHIFT);
}

static int qcom_hap_play(struct qcom_hap *hap, bool play)
{
	return regmap_update_bits(hap->regmap, hap->base + HAP_PLAY,
				  HAP_PLAY_BIT, play ? HAP_PLAY_BIT : 0);
}

static int qcom_hap_module_enable(struct qcom_hap *hap, bool enable)
{
	return regmap_update_bits(hap->regmap, hap->base + HAP_EN_CTL1,
				  HAP_EN, enable ? HAP_EN : 0);
}

static void qcom_hap_stop(struct qcom_hap *hap)
{
	qcom_hap_play(hap, false);

	/* Let the sample being generated finish so that stopping is silent. */
	usleep_range(hap->play_rate_us, hap->play_rate_us + 1000);
	qcom_hap_module_enable(hap, false);
	hap->playing = false;
}

static void qcom_hap_work(struct work_struct *work)
{
	struct qcom_hap *hap = container_of(work, struct qcom_hap, work);
	int ret;

	if (hap->suspended || !hap->magnitude) {
		if (hap->playing)
			qcom_hap_stop(hap);

		return;
	}

	ret = qcom_hap_set_level(hap, hap->magnitude);
	if (ret)
		goto err;

	if (!hap->playing) {
		ret = qcom_hap_module_enable(hap, true);
		if (ret)
			goto err;

		/* Play the constant level programmed above. */
		ret = regmap_update_bits(hap->regmap, hap->base + HAP_SEL,
					 HAP_WF_SOURCE_MASK | HAP_WF_TRIGGER,
					 HAP_WF_SOURCE_VMAX << HAP_WF_SOURCE_SHIFT);
		if (ret)
			goto err_disable;

		ret = qcom_hap_play(hap, true);
		if (ret)
			goto err_disable;

		hap->playing = true;
	}

	return;

err_disable:
	qcom_hap_module_enable(hap, false);
err:
	dev_err(hap->dev, "failed to start vibration: %d\n", ret);
	hap->magnitude = 0;
}

static int qcom_hap_play_effect(struct input_dev *input, void *data,
				struct ff_effect *effect)
{
	struct qcom_hap *hap = input_get_drvdata(input);

	hap->magnitude = effect->u.rumble.strong_magnitude;
	if (!hap->magnitude)
		hap->magnitude = effect->u.rumble.weak_magnitude;

	schedule_work(&hap->work);

	return 0;
}

static void qcom_hap_close(struct input_dev *input)
{
	struct qcom_hap *hap = input_get_drvdata(input);

	hap->magnitude = 0;
	cancel_work_sync(&hap->work);

	if (hap->playing)
		qcom_hap_stop(hap);
}

static int qcom_hap_hw_init(struct qcom_hap *hap)
{
	unsigned int rate = hap->play_rate_us / HAP_RATE_US_LSB;
	int ret;

	/* ERM actuator, 1 A current limit and the analogue blocks powered. */
	ret = regmap_write(hap->regmap, hap->base + HAP_CFG1, HAP_ACT_ERM);
	if (ret)
		return ret;

	ret = regmap_write(hap->regmap, hap->base + HAP_ILIM_CFG,
			   HAP_ILIM_1000MA);
	if (ret)
		return ret;

	ret = regmap_write(hap->regmap, hap->base + HAP_EN_CTL3,
			   HAP_HBRIDGE_EN | HAP_PWM_SIGNAL_EN | HAP_ILIM_EN |
			   HAP_ILIM_CC_EN | HAP_AUTO_RES_RBIAS_EN |
			   HAP_DAC_EN | HAP_ZX_HYST_EN | HAP_PWM_CTL_EN);
	if (ret)
		return ret;

	/* An ERM has no resonance to track. */
	ret = regmap_update_bits(hap->regmap, hap->base + HAP_AUTO_RES_CTRL,
				 HAP_AUTO_RES_EN, 0);
	if (ret)
		return ret;

	ret = regmap_write(hap->regmap, hap->base + HAP_RATE_CFG1,
			   rate & 0xff);
	if (ret)
		return ret;

	return regmap_write(hap->regmap, hap->base + HAP_RATE_CFG2,
			    (rate >> 8) & 0xf);
}

static int qcom_hap_parse_dt(struct qcom_hap *hap)
{
	struct device_node *node = hap->dev->of_node;
	u32 val;
	int ret;

	if (!node)
		return dev_err_probe(hap->dev, -EINVAL, "no device tree node\n");

	ret = of_property_read_u32(node, "reg", &hap->base);
	if (ret)
		return dev_err_probe(hap->dev, ret, "missing register base\n");

	hap->vmax_mv = HAP_VMAX_MV_DEFAULT;
	if (!of_property_read_u32(node, "qcom,vmax-microvolt", &val))
		hap->vmax_mv = min(val / 1000, (u32)HAP_VMAX_MV_MAX);

	hap->play_rate_us = HAP_RATE_US_DEFAULT;
	if (!of_property_read_u32(node, "qcom,play-rate-us", &val))
		hap->play_rate_us = clamp_val(val, HAP_RATE_US_LSB,
					      HAP_RATE_US_MAX);

	return 0;
}

static int qcom_hap_probe(struct platform_device *pdev)
{
	struct qcom_hap *hap;
	struct input_dev *input;
	unsigned int type;
	int ret;

	hap = devm_kzalloc(&pdev->dev, sizeof(*hap), GFP_KERNEL);
	if (!hap)
		return -ENOMEM;

	hap->dev = &pdev->dev;
	hap->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!hap->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "no parent regmap\n");

	ret = qcom_hap_parse_dt(hap);
	if (ret)
		return ret;

	/* The module type register tells us the haptics block is really there. */
	ret = regmap_read(hap->regmap, hap->base + HAP_REG_TYPE, &type);
	if (ret)
		return ret;

	if (type != HAP_TYPE)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "unexpected peripheral type 0x%02x\n", type);

	ret = qcom_hap_hw_init(hap);
	if (ret)
		return ret;

	input = devm_input_allocate_device(&pdev->dev);
	if (!input)
		return -ENOMEM;

	INIT_WORK(&hap->work, qcom_hap_work);

	input->name = "qcom-pm8150b-haptics";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;
	input->close = qcom_hap_close;

	input_set_drvdata(input, hap);
	input_set_capability(input, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input, NULL, qcom_hap_play_effect);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create force feedback\n");

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register input device\n");

	platform_set_drvdata(pdev, hap);

	return 0;
}

static int qcom_hap_suspend(struct device *dev)
{
	struct qcom_hap *hap = dev_get_drvdata(dev);

	hap->suspended = true;
	cancel_work_sync(&hap->work);
	if (hap->playing)
		qcom_hap_stop(hap);

	return 0;
}

static int qcom_hap_resume(struct device *dev)
{
	struct qcom_hap *hap = dev_get_drvdata(dev);

	hap->suspended = false;
	if (hap->magnitude)
		schedule_work(&hap->work);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_hap_pm_ops, qcom_hap_suspend,
				qcom_hap_resume);

static const struct of_device_id qcom_hap_of_match[] = {
	{ .compatible = "qcom,pm8150b-haptics" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_hap_of_match);

static struct platform_driver qcom_hap_driver = {
	.probe = qcom_hap_probe,
	.driver = {
		.name = "qcom-pm8150b-haptics",
		.pm = pm_sleep_ptr(&qcom_hap_pm_ops),
		.of_match_table = qcom_hap_of_match,
	},
};
module_platform_driver(qcom_hap_driver);

MODULE_DESCRIPTION("Qualcomm PM8150B haptics driver");
MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
MODULE_LICENSE("GPL");
