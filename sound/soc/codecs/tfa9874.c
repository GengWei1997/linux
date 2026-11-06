// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 GengWei1997 <gw19970326@gmail.com>
 *
 * NXP/Goodix TFA9874 (TFA2 family) speaker amplifier driver
 *
 * The chip exposes a large "CoolFlux DSP" which this driver deliberately
 * leaves unused.  The vendor code path for this board is the "probus"
 * (no-DSP) one: the amplifier is driven straight from I2S with a fixed
 * register setup.  All sequences below are taken from the vendor's
 * tfa98xx driver, which in turn generated them from the NXP container
 * file shipped in the firmware.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define TFA9874_REG_STATUS		0x00
#define TFA9874_STATUS_PWDN		BIT(0)
#define TFA9874_STATUS_I2CR		BIT(4)
#define TFA9874_STATUS_AMPE		BIT(3)
#define TFA9874_REG_MANSCONF		0x01
#define TFA9874_MANSCONF_MANSCONF	BIT(2)
#define TFA9874_REG_AUDIO_CTR		0x02
#define TFA9874_AUDIO_CTR_AUDFS_MSK	GENMASK(3, 0)
#define TFA9874_REG_REV			0x03
#define TFA9874_REV_MASK		GENMASK(15, 0)
#define TFA9874_DEVICE_ID		0x74
#define TFA9874_REG_KEY			0x0f
#define TFA9874_REG_MTPKEY1		0xa0
#define TFA9874_REG_MTPKEY2		0xa1
#define TFA9874_REG_KEY_VALUE		0x5a6b
#define TFA9874_MTPKEY2_VALUE		0x005a

/* Optimal settings, written as whole registers before the container tables. */
#define TFA9874_REV_0A74		0x0a74
#define TFA9874_REV_0B74		0x0b74
#define TFA9874_REV_0C74		0x0c74

/* Vendor reset pulse: assert (high) for 10 ms, release (low) for 10 ms. */
#define TFA9874_RESET_PULSE_MS		10

struct tfa9874_reg {
	u8 reg;
	u16 mask;
	u16 val;
};

struct tfa9874 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
};

/*
 * The optimal settings tables are written with regmap_write(), i.e. the whole
 * register is replaced.  The vendor comments them as "POR=0x...." which is the
 * power-on value, confirming these are complete register contents rather than
 * field updates.
 */
static const struct reg_sequence tfa9874_init_0a74[] = {
	{ 0x02, 0x22a8 }, { 0x51, 0x0020 }, { 0x52, 0x57dc }, { 0x58, 0x16a4 },
	{ 0x61, 0x0110 }, { 0x66, 0x0701 }, { 0x6f, 0x00a3 }, { 0x70, 0x07f8 },
	{ 0x73, 0x0007 }, { 0x74, 0x5068 }, { 0x75, 0x0d28 }, { 0x83, 0x0594 },
	{ 0x84, 0x0001 }, { 0x85, 0x0001 }, { 0x88, 0x0000 }, { 0xc4, 0x2001 },
};

static const struct reg_sequence tfa9874_init_0b74[] = {
	{ 0x02, 0x22a8 }, { 0x51, 0x0020 }, { 0x52, 0x57dc }, { 0x58, 0x16a4 },
	{ 0x61, 0x0110 }, { 0x66, 0x0701 }, { 0x6f, 0x00a3 }, { 0x70, 0x07f8 },
	{ 0x73, 0x0047 }, { 0x74, 0x5068 }, { 0x75, 0x0d28 }, { 0x83, 0x0595 },
	{ 0x84, 0x0001 }, { 0x85, 0x0001 }, { 0x88, 0x0000 }, { 0xc4, 0x2001 },
};

static const struct reg_sequence tfa9874_init_0c74[] = {
	{ 0x02, 0x22c8 }, { 0x52, 0x57dc }, { 0x53, 0x003e }, { 0x56, 0x0400 },
	{ 0x61, 0x0110 }, { 0x6f, 0x00a5 }, { 0x70, 0x07f8 }, { 0x73, 0x0047 },
	{ 0x74, 0x5098 }, { 0x75, 0x8d28 }, { 0x80, 0x0000 }, { 0x83, 0x0799 },
	{ 0x84, 0x0081 },
};

/* Device table from the vendor container: read-modify-write. */
static const struct tfa9874_reg tfa9874_device_regs[] = {
	/*
	 * Two 16-bit slots per frame: 32 BCLKs, the frame the MI2S link
	 * actually carries for S16_LE stereo.  reg20's TDMNBCK field states
	 * the bit clock to frame ratio and has to describe the same frame,
	 * otherwise the chip reports TDMLUTER and never unmutes.
	 */
	{ 0x20, 0xf000, 0x0000 },
	{ 0x21, 0x01ff, 0x00f1 },
	{ 0x23, 0x0008, 0x0008 },
	{ 0x23, 0x0010, 0x0010 },
	{ 0x26, 0x000f, 0x0000 },
	{ 0x26, 0xf000, 0x1000 },
	{ 0x27, 0x000f, 0x0000 },
	{ 0x68, 0x0007, 0x0002 },
	{ 0x68, 0x0018, 0x0008 },
	{ 0x68, 0x0060, 0x0040 },
	{ 0x02, 0x07e0, 0x01e0 },
	{ 0x64, 0xc000, 0x4000 },
	{ 0x74, 0x0008, 0x0008 },
	{ 0x88, 0x0003, 0x0002 },
	{ 0x74, 0x3e00, 0x1000 },
	{ 0x8f, 0x0001, 0x0000 },
	{ 0x74, 0x8000, 0x0000 },
	{ 0x70, 0x0180, 0x0180 },
	{ 0xc4, 0x2000, 0x0000 },
	{ 0x66, 0x0001, 0x0000 },
	{ 0x58, 0x01f0, 0x0010 },
	{ 0x6f, 0x0380, 0x0080 },
	{ 0x85, 0x0002, 0x0002 },
	{ 0x84, 0x01e0, 0x0080 },
	{ 0x6f, 0x0020, 0x0020 },
	{ 0x83, 0x003f, 0x0015 },
	{ 0x83, 0x07c0, 0x0700 },
	{ 0xf8, 0xe000, 0x6000 },
	{ 0x61, 0x03c0, 0x0100 },
	{ 0x61, 0x003c, 0x0010 },
	{ 0x74, 0x01f0, 0x0090 },
	{ 0x75, 0x1f00, 0x0d00 },
	{ 0x76, 0x7e00, 0x7e00 },
};

/* Profile table from the vendor container: read-modify-write. */
static const struct tfa9874_reg tfa9874_profile_regs[] = {
	{ 0x00, 0x0010, 0x0010 },	/* DCA: enable the DC-DC converter */
	{ 0x64, 0xc000, 0x4000 },	/* LPM1MODE */
	{ 0x62, 0xc000, 0x0000 },	/* LNMODE */
	{ 0x02, 0x000f, 0x0008 },	/* AUDFS: 48 kHz */
};

static int tfa9874_apply_table(struct tfa9874 *tfa,
			       const struct tfa9874_reg *regs, size_t len)
{
	int i, ret;

	for (i = 0; i < len; i++) {
		ret = regmap_update_bits(tfa->regmap, regs[i].reg,
					 regs[i].mask, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * The vendor unlocks the MTP key registers using a value derived from the
 * device itself, then hides them again.  Writing a fixed constant (as the
 * exported container does) would be wrong.
 */
static int tfa9874_unlock(struct tfa9874 *tfa)
{
	unsigned int key;
	int ret;

	ret = regmap_write(tfa->regmap, TFA9874_REG_KEY, TFA9874_REG_KEY_VALUE);
	if (ret)
		return ret;

	ret = regmap_read(tfa->regmap, 0xfb, &key);
	if (ret)
		return ret;

	ret = regmap_write(tfa->regmap, TFA9874_REG_MTPKEY1,
			   (key & 0xffff) ^ TFA9874_MTPKEY2_VALUE);
	if (ret)
		return ret;

	ret = regmap_write(tfa->regmap, TFA9874_REG_MTPKEY2,
			   TFA9874_MTPKEY2_VALUE);
	if (ret)
		return ret;

	return regmap_write(tfa->regmap, TFA9874_REG_KEY, 0);
}

static int tfa9874_optimal_settings(struct tfa9874 *tfa, unsigned int rev)
{
	const struct reg_sequence *seq;
	size_t len;

	switch (rev & TFA9874_REV_MASK) {
	case TFA9874_REV_0A74:
		seq = tfa9874_init_0a74;
		len = ARRAY_SIZE(tfa9874_init_0a74);
		break;
	case TFA9874_REV_0B74:
		seq = tfa9874_init_0b74;
		len = ARRAY_SIZE(tfa9874_init_0b74);
		break;
	case TFA9874_REV_0C74:
		seq = tfa9874_init_0c74;
		len = ARRAY_SIZE(tfa9874_init_0c74);
		break;
	default:
		dev_warn(tfa->dev, "no optimal settings for revision 0x%04x\n",
			 rev);
		return 0;
	}

	return regmap_multi_reg_write(tfa->regmap, seq, len);
}

static int tfa9874_init_sequence(struct tfa9874 *tfa, unsigned int rev)
{
	int ret;

	/* Software reset: I2CR is self-clearing. */
	ret = regmap_write(tfa->regmap, TFA9874_REG_STATUS,
			   TFA9874_STATUS_I2CR);
	if (ret)
		return ret;

	ret = tfa9874_unlock(tfa);
	if (ret)
		return ret;

	ret = tfa9874_optimal_settings(tfa, rev);
	if (ret)
		return ret;

	ret = tfa9874_apply_table(tfa, tfa9874_device_regs,
				  ARRAY_SIZE(tfa9874_device_regs));
	if (ret)
		return ret;

	return tfa9874_apply_table(tfa, tfa9874_profile_regs,
				   ARRAY_SIZE(tfa9874_profile_regs));
}

static int tfa9874_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tfa9874 *tfa = snd_soc_component_get_drvdata(component);
	unsigned int audfs;
	int ret;

	switch (params_rate(params)) {
	case 8000:
		audfs = 0;
		break;
	case 11025:
		audfs = 1;
		break;
	case 12000:
		audfs = 2;
		break;
	case 16000:
		audfs = 3;
		break;
	case 22050:
		audfs = 4;
		break;
	case 24000:
		audfs = 5;
		break;
	case 32000:
		audfs = 6;
		break;
	case 44100:
		audfs = 7;
		break;
	case 48000:
		audfs = 8;
		break;
	default:
		dev_err(component->dev, "unsupported sample rate %u\n",
			params_rate(params));
		return -EINVAL;
	}

	ret = regmap_update_bits(tfa->regmap, TFA9874_REG_AUDIO_CTR,
				 TFA9874_AUDIO_CTR_AUDFS_MSK, audfs);
	if (ret)
		return ret;

	dev_dbg(component->dev, "AUDFS=%u for %u Hz\n", audfs,
		params_rate(params));

	return 0;
}

static int tfa9874_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops tfa9874_dai_ops = {
	.hw_params	= tfa9874_hw_params,
	.set_fmt	= tfa9874_set_fmt,
};

static struct snd_soc_dai_driver tfa9874_dai = {
	.name = "tfa9874-aif",
	.playback = {
		.stream_name	= "HiFi Playback",
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.rate_min	= 8000,
		.rate_max	= 48000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &tfa9874_dai_ops,
};

static int tfa9874_spk_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct tfa9874 *tfa = snd_soc_component_get_drvdata(component);
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/*
		 * The vendor state machine is explicit that the order of these
		 * bits matters: release powerdown, mark the I2C configuration
		 * as valid, and only then enable the amplifier.
		 */
		ret = regmap_update_bits(tfa->regmap, TFA9874_REG_STATUS,
					 TFA9874_STATUS_PWDN, 0);
		if (ret)
			return ret;

		ret = regmap_update_bits(tfa->regmap, TFA9874_REG_MANSCONF,
					 TFA9874_MANSCONF_MANSCONF,
					 TFA9874_MANSCONF_MANSCONF);
		if (ret)
			return ret;

		return regmap_update_bits(tfa->regmap, TFA9874_REG_STATUS,
					  TFA9874_STATUS_AMPE,
					  TFA9874_STATUS_AMPE);
	case SND_SOC_DAPM_PRE_PMD:
		ret = regmap_update_bits(tfa->regmap, TFA9874_REG_STATUS,
					 TFA9874_STATUS_AMPE, 0);
		if (ret)
			return ret;

		return regmap_update_bits(tfa->regmap, TFA9874_REG_STATUS,
					  TFA9874_STATUS_PWDN,
					  TFA9874_STATUS_PWDN);
	default:
		return 0;
	}
}

static const struct snd_soc_dapm_widget tfa9874_dapm_widgets[] = {
	/*
	 * The core names the DAI widget after playback.stream_name, so this
	 * explicit widget is what connects the DAI to the amplifier output.
	 */
	SND_SOC_DAPM_AIF_IN("AIF IN", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("SPK DRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       tfa9874_spk_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route tfa9874_dapm_routes[] = {
	{ "SPK DRV", NULL, "AIF IN" },
	{ "SPK", NULL, "SPK DRV" },
};

static const struct snd_soc_component_driver tfa9874_component_driver = {
	.dapm_widgets		= tfa9874_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa9874_dapm_widgets),
	.dapm_routes		= tfa9874_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa9874_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config tfa9874_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.max_register	= 0xff,
	.cache_type	= REGCACHE_NONE,
};

static int tfa9874_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct tfa9874 *tfa;
	unsigned int rev;
	int ret;

	tfa = devm_kzalloc(dev, sizeof(*tfa), GFP_KERNEL);
	if (!tfa)
		return -ENOMEM;

	tfa->dev = dev;
	tfa->regmap = devm_regmap_init_i2c(i2c, &tfa9874_regmap_config);
	if (IS_ERR(tfa->regmap))
		return dev_err_probe(dev, PTR_ERR(tfa->regmap),
				     "failed to allocate register map\n");

	tfa->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tfa->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(tfa->reset_gpio),
				     "failed to get reset GPIO\n");

	if (tfa->reset_gpio) {
		msleep(TFA9874_RESET_PULSE_MS);
		gpiod_set_value_cansleep(tfa->reset_gpio, 0);
		msleep(TFA9874_RESET_PULSE_MS);
	}

	i2c_set_clientdata(i2c, tfa);

	ret = regmap_read(tfa->regmap, TFA9874_REG_REV, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read revision\n");

	if ((rev & 0xff) != TFA9874_DEVICE_ID) {
		dev_err(dev, "not a TFA9874 (revision 0x%04x)\n", rev);
		return -ENODEV;
	}

	dev_info(dev, "TFA9874 detected, revision 0x%04x\n", rev);

	ret = tfa9874_init_sequence(tfa, rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise device\n");

	return devm_snd_soc_register_component(dev, &tfa9874_component_driver,
					       &tfa9874_dai, 1);
}

static const struct of_device_id tfa9874_of_match[] = {
	{ .compatible = "nxp,tfa9874" },
	{ }
};
MODULE_DEVICE_TABLE(of, tfa9874_of_match);

static const struct i2c_device_id tfa9874_i2c_id[] = {
	{ "tfa9874" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9874_i2c_id);

static struct i2c_driver tfa9874_i2c_driver = {
	.driver = {
		.name		= "tfa9874",
		.of_match_table	= tfa9874_of_match,
	},
	.probe		= tfa9874_i2c_probe,
	.id_table	= tfa9874_i2c_id,
};
module_i2c_driver(tfa9874_i2c_driver);

MODULE_DESCRIPTION("NXP/Goodix TFA9874 speaker amplifier");
MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
MODULE_LICENSE("GPL");
