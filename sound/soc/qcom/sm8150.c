// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 GengWei1997 <gw19970326@gmail.com>
 *
 * SM8150 ASoC machine driver.
 *
 * The board wires a WCD9340 codec over SLIMbus for the earpiece, the
 * headphones and both microphones, and a TFA9874 class-D amplifier on
 * the quaternary MI2S for the speaker.  Everything board specific lives
 * in the device tree; this driver only configures the CPU DAI clocks,
 * the codec channel map and the headset jack.
 */

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/input-event-codes.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "common.h"
#include "qdsp6/q6afe.h"

#define DRIVER_NAME		"sm8150"
#define DEFAULT_SAMPLE_RATE	48000
#define MI2S_BCLK_RATE		1536000
#define DEFAULT_MCLK_RATE	19200000
#define WCD934X_MCLK_RATE	9600000
#define SLIM_MAX_TX_PORTS	16
#define SLIM_MAX_RX_PORTS	13

struct sm8150_snd_data {
	struct snd_soc_jack jack;
	bool jack_setup;
	bool slim_setup;
};

static struct snd_soc_jack_pin sm8150_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int sm8150_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = DEFAULT_SAMPLE_RATE;
	rate->max = DEFAULT_SAMPLE_RATE;
	channels->min = 2;
	channels->max = 2;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int sm8150_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	unsigned int fmt;
	int i, ret;

	/*
	 * Front-end DAIs use MSM_FRONTEND_DAI_MULTIMEDIA1..16 (ids 0..15),
	 * which collide with the q6afe port ids.  Only the back ends get
	 * clock configuration here.
	 */
	if (!rtd->dai_link->no_pcm)
		return 0;

	switch (cpu_dai->id) {
	case QUATERNARY_MI2S_RX:
		/*
		 * The MI2S bit clock is derived from the LPASS audio MCLK, so
		 * that root clock has to be running before the port is asked
		 * for a bit clock.  Without it set_sysclk() succeeds but no
		 * clock ever reaches the amplifier.
		 */
		ret = snd_soc_dai_set_sysclk(cpu_dai,
					     Q6AFE_LPASS_CLK_ID_MCLK_1,
					     DEFAULT_MCLK_RATE,
					     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_sysclk(cpu_dai,
					     Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
					     MI2S_BCLK_RATE,
					     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP |
						 SND_SOC_DAIFMT_NB_NF |
						 SND_SOC_DAIFMT_I2S);
		if (ret)
			return ret;

		fmt = SND_SOC_DAIFMT_CBC_CFC | SND_SOC_DAIFMT_NB_NF |
		      SND_SOC_DAIFMT_I2S;
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, fmt);
			if (ret)
				return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int sm8150_snd_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	unsigned int rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	unsigned int rx_ch_cnt = 0, tx_ch_cnt = 0;
	int i, ret;

	/* Only SLIM back ends need a channel map; MI2S does not. */
	if (!rtd->dai_link->no_pcm)
		return 0;
	if (cpu_dai->id < SLIMBUS_0_RX || cpu_dai->id > SLIMBUS_6_TX)
		return 0;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_get_channel_map(codec_dai, &tx_ch_cnt, tx_ch,
						  &rx_ch_cnt, rx_ch);
		if (ret == -ENOTSUPP)
			continue;
		if (ret)
			return ret;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
							  rx_ch_cnt, rx_ch);
		else
			ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt,
							  tx_ch, 0, NULL);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static const struct snd_soc_ops sm8150_be_ops = {
	.startup	= sm8150_snd_startup,
	.hw_params	= sm8150_snd_hw_params,
};

static int sm8150_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai_link *link = rtd->dai_link;
	struct sm8150_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai;
	struct snd_jack *jack;
	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {
		144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156 };
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {
		128, 129, 130, 131, 132, 133, 134, 135,
		136, 137, 138, 139, 140, 141, 142, 143 };
	int i, ret;

	if (!data->jack_setup) {
		ret = snd_soc_card_jack_new_pins(card, "Headset Jack",
						 SND_JACK_HEADSET |
						 SND_JACK_HEADPHONE |
						 SND_JACK_BTN_0 |
						 SND_JACK_BTN_1 |
						 SND_JACK_BTN_2 |
						 SND_JACK_BTN_3,
						 &data->jack,
						 sm8150_jack_pins,
						 ARRAY_SIZE(sm8150_jack_pins));
		if (ret) {
			dev_err(card->dev, "unable to add the headset jack\n");
			return ret;
		}

		jack = data->jack.jack;
		snd_jack_set_key(jack, SND_JACK_BTN_0, KEY_MEDIA);
		snd_jack_set_key(jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
		snd_jack_set_key(jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
		snd_jack_set_key(jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
		data->jack_setup = true;
	}

	/* Front-end ids collide with the SLIM port ids, see sm8150_snd_startup(). */
	if (!link->no_pcm || data->slim_setup)
		return 0;
	if (snd_soc_rtd_to_cpu(rtd, 0)->id < SLIMBUS_0_RX ||
	    snd_soc_rtd_to_cpu(rtd, 0)->id > SLIMBUS_6_TX)
		return 0;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_channel_map(codec_dai,
						  ARRAY_SIZE(tx_ch), tx_ch,
						  ARRAY_SIZE(rx_ch), rx_ch);
		if (ret && ret != -ENOTSUPP)
			return ret;

		snd_soc_dai_set_sysclk(codec_dai, 0, WCD934X_MCLK_RATE,
				       SNDRV_PCM_STREAM_PLAYBACK);

		ret = snd_soc_component_set_jack(codec_dai->component,
						 &data->jack, NULL);
		if (ret && ret != -ENOTSUPP) {
			dev_warn(card->dev, "failed to set jack: %d\n", ret);
			return ret;
		}
	}

	data->slim_setup = true;
	return 0;
}

static const struct snd_soc_dapm_widget sm8150_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static void sm8150_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->ops = &sm8150_be_ops;
			link->be_hw_params_fixup = sm8150_be_hw_params_fixup;
		}
		link->init = sm8150_dai_init;
	}
}

static int sm8150_snd_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct sm8150_snd_data *data;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->driver_name = DRIVER_NAME;
	card->dapm_widgets = sm8150_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(sm8150_dapm_widgets);
	card->dev = dev;
	card->owner = THIS_MODULE;
	dev_set_drvdata(dev, card);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	snd_soc_card_set_drvdata(card, data);
	sm8150_add_be_ops(card);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id sm8150_snd_device_id[] = {
	{ .compatible = "qcom,sm8150-sndcard" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm8150_snd_device_id);

static struct platform_driver sm8150_snd_driver = {
	.probe = sm8150_snd_platform_probe,
	.driver = {
		.name = "msm-snd-sm8150",
		.of_match_table = sm8150_snd_device_id,
	},
};
module_platform_driver(sm8150_snd_driver);

MODULE_DESCRIPTION("SM8150 ASoC Machine Driver");
MODULE_AUTHOR("GengWei1997 <gw19970326@gmail.com>");
MODULE_LICENSE("GPL");
