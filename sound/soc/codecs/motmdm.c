// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola Mapphone MDM6600 voice call audio support
 * Copyright 2018 Tony Lindgren <tony@atomide.com>
 */

#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/serdev.h>
#include <linux/serdev-gsm.h>

#include <sound/soc.h>
#include <sound/tlv.h>

#include <linux/mfd/motorola-mdm.h>

struct motmdm_driver_data {
	struct snd_soc_component *component;
	struct snd_soc_dai *master_dai;
	struct device *modem;
	struct motmdm_dlci mot_dlci;
	struct regmap *regmap;
	unsigned char *buf;
	size_t len;
	unsigned int enabled:1;
	spinlock_t lock;	/* enable/disabled lock */
};

enum motmdm_cmd {
	CMD_AT_EACC,
	CMD_AT_CLVL,
	CMD_AT_NREC,
};

const char * const motmd_read_fmt[] = {
	[CMD_AT_EACC] = "AT+EACC?",
	[CMD_AT_CLVL] = "AT+CLVL?",
	[CMD_AT_NREC] = "AT+NREC?",
};

const char * const motmd_write_fmt[] = {
	[CMD_AT_EACC] = "AT+EACC=%u,0",
	[CMD_AT_CLVL] = "AT+CLVL=%u",
	[CMD_AT_NREC] = "AT+NREC=%u",
};

/*
 * Currently unconfigured additional inactive (error producing) options
 * seem to be:
 * "TTY Headset", "HCQ Headset", "VCQ Headset", "No-Mic Headset",
 * "Handset Fluence Med", "Handset Fluence Low", "Car Dock", "Lapdock"
 */
static const char * const motmdm_out_mux_texts[] = {
	"Handset", "Headset", "Speakerphone", "Bluetooth",
};

static SOC_ENUM_SINGLE_EXT_DECL(motmdm_out_enum, motmdm_out_mux_texts);

static const DECLARE_TLV_DB_SCALE(motmdm_gain_tlv, 0, 100, 0);


static int motmdm_read_reg(void *context, unsigned int reg,
			   unsigned int *value)
{
	struct snd_soc_component *component = context;
	struct motmdm_driver_data *ddata = snd_soc_component_get_drvdata(component);
	const unsigned char *cmd;
	unsigned int val;
	int error;

	cmd = motmd_read_fmt[reg];
	error = motmdm_send_command(ddata->modem, &ddata->mot_dlci, 1000,
					 cmd, strlen(cmd),
					 ddata->buf, ddata->len);
	if (error < 0) {
		dev_err(component->dev, "%s: %s failed with %i\n",
			__func__, cmd, error);

		return error;
	}

	error = kstrtouint(ddata->buf, 0, &val);
	if (error)
		return -ENODEV;

	*value = val;

	return error;
}

static int motmdm_write_reg(void *context, unsigned int reg,
			    unsigned int value)
{
	struct snd_soc_component *component = context;
	struct motmdm_driver_data *ddata = snd_soc_component_get_drvdata(component);
	const unsigned char *fmt, *cmd;
	int error;

	fmt = motmd_write_fmt[reg];
	cmd = kasprintf(GFP_KERNEL, fmt, value);
	if (!cmd) {
		error = -ENOMEM;
		goto free;
	}

	error = motmdm_send_command(ddata->modem, &ddata->mot_dlci, 1000,
					 cmd, strlen(cmd),
					 ddata->buf, ddata->len);
	if (error < 0)
		dev_err(component->dev, "%s: %s failed with %i\n",
			__func__, cmd, error);

free:
	kfree(cmd);

	return error;
}

static const struct reg_default motmdm_reg_defaults[] = {
	{ CMD_AT_EACC, 0x0 },
	{ CMD_AT_CLVL, 0x0 },
};

static const struct regmap_config motmdm_regmap = {
	.reg_bits = 32,
	.reg_stride = 1,
	.val_bits = 32,
	.max_register = CMD_AT_NREC,
	.reg_defaults = motmdm_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(motmdm_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.reg_read = motmdm_read_reg,
	.reg_write = motmdm_write_reg,
};

static int motmdm_value_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol,
			    enum motmdm_cmd reg,
			    int cmd_base)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int error;

	error = regmap_read(ddata->regmap, reg, &val);
	if (error)
		return error;

	if (val >= cmd_base)
		val -= cmd_base;

	ucontrol->value.enumerated.item[0] = val;

	return 0;
}

static int motmdm_value_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol,
			    enum motmdm_cmd reg,
			    int cmd_base)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata = snd_soc_component_get_drvdata(component);
	int error;

	error = regmap_write(ddata->regmap, reg,
			     ucontrol->value.enumerated.item[0] + cmd_base);
	if (error)
		return error;

	regcache_mark_dirty(ddata->regmap);

	return error;
}

static int motmdm_audio_out_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_get(kcontrol, ucontrol, CMD_AT_EACC, 1);
}

static int motmdm_audio_out_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_put(kcontrol, ucontrol, CMD_AT_EACC, 1);
}

static int motmdm_gain_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_get(kcontrol, ucontrol, CMD_AT_CLVL, 0);
}

static int motmdm_gain_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_put(kcontrol, ucontrol, CMD_AT_CLVL, 0);
}

static int motmdm_noise_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_get(kcontrol, ucontrol, CMD_AT_NREC, 0);
}

static int motmdm_noise_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	return motmdm_value_put(kcontrol, ucontrol, CMD_AT_NREC, 0);
}

static int
motmdm_enable_primary_dai(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);
	int error;

	if (!ddata->master_dai)
		return -ENODEV;

	error = snd_soc_dai_set_sysclk(ddata->master_dai, 1, 19200000,
				       SND_SOC_CLOCK_OUT);
	if (error)
		return error;

	error = snd_soc_dai_set_fmt(ddata->master_dai,
				    SND_SOC_DAIFMT_I2S |
				    SND_SOC_DAIFMT_NB_NF |
				    SND_SOC_DAIFMT_CBM_CFM);
	if (error)
		return error;

	error = snd_soc_dai_set_tdm_slot(ddata->master_dai, 0, 1, 1, 8);
	if (error)
		return error;

	return error;
}

static int
motmdm_disable_primary_dai(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);
	int error;

	if (!ddata->master_dai)
		return -ENODEV;

	error = snd_soc_dai_set_sysclk(ddata->master_dai, 0, 26000000,
				       SND_SOC_CLOCK_OUT);
	if (error)
		return error;

	error = snd_soc_dai_set_fmt(ddata->master_dai,
				    SND_SOC_DAIFMT_CBM_CFM);
	if (error)
		return error;

	error = snd_soc_dai_set_tdm_slot(ddata->master_dai, 0, 0, 0, 48);
	if (error)
		return error;

	return error;
}

static int motmdm_find_primary_dai(struct snd_soc_component *component,
	const char *name)
{
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);
	struct device_node *bitclkmaster = NULL, *framemaster = NULL;
	struct device_node *ep, *master_ep, *master = NULL;
	struct snd_soc_dai_link_component dlc = { 0 };
	unsigned int daifmt;

	ep = of_graph_get_next_endpoint(component->dev->of_node, NULL);
	if (!ep)
		return -ENODEV;

	master_ep = of_graph_get_remote_endpoint(ep);
	of_node_put(ep);
	if (!master_ep)
		return -ENODEV;

	daifmt = snd_soc_of_parse_daifmt(master_ep, NULL,
					 &bitclkmaster, &framemaster);
	of_node_put(master_ep);
	if (bitclkmaster && framemaster)
		master = of_graph_get_port_parent(bitclkmaster);
	of_node_put(bitclkmaster);
	of_node_put(framemaster);
	if (!master)
		return -ENODEV;

	dlc.of_node = master;
	dlc.dai_name = name;
	ddata->master_dai = snd_soc_find_dai(&dlc);
	of_node_put(master);
	if (!ddata->master_dai)
		return -EPROBE_DEFER;

	dev_info(component->dev, "Master DAI is %s\n",
		 dev_name(ddata->master_dai->dev));

	return 0;
}

static int motmdm_parse_tdm(struct snd_soc_component *component)
{
	return motmdm_find_primary_dai(component, "cpcap-voice");
}

static const struct snd_kcontrol_new motmdm_snd_controls[] = {
        SOC_ENUM_EXT("Call Output", motmdm_out_enum,
                     motmdm_audio_out_get,
                     motmdm_audio_out_put),
        SOC_SINGLE_EXT_TLV("Call Volume",
			   0, 0, 7, 0,
			   motmdm_gain_get,
			   motmdm_gain_put,
			   motmdm_gain_tlv),
	SOC_SINGLE_BOOL_EXT("Call Noise Cancellation", 0,
			    motmdm_noise_get,
			    motmdm_noise_put),
};

static struct snd_soc_dai_driver motmdm_dai[] = {
	{
		.name = "mdm-call",
		.playback = {
			.stream_name = "Voice Call Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Voice Call Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static int motmdm_soc_notify(struct motmdm_dlci *mot_dlci,
			     enum motmdm_state state)
{
	struct motmdm_driver_data *ddata = mot_dlci->drvdata;
	unsigned long flags;
	bool enable, notify = false;

	switch (state) {
	case MOTMDM_STATE_DIAL:
	case MOTMDM_STATE_ANSWERING:
	case MOTMDM_STATE_CONNECTING:
	case MOTMDM_STATE_CONNECTED:
		enable = true;
		break;
	case MOTMDM_STATE_HANGING_UP:
	case MOTMDM_STATE_DISCONNECTED:
		enable = false;
		break;
	default:
		return 0;
	}

	spin_lock_irqsave(&ddata->lock, flags);
	if (ddata->enabled != enable) {
		ddata->enabled = enable;
		notify = true;
	}
	spin_unlock_irqrestore(&ddata->lock, flags);

	if (!notify)
		return 0;

	if (enable)
		motmdm_enable_primary_dai(ddata->component);
	else
		motmdm_disable_primary_dai(ddata->component);

	return 0;
}

static int motmdm_soc_probe(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata;
	struct motmdm_dlci *mot_dlci;
	const unsigned char *cmd = "AT+CMUT=0";
	int error;

	ddata = devm_kzalloc(component->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->component = component;
	ddata->modem = component->dev->parent;
	mot_dlci = &ddata->mot_dlci;
	ddata->len = PAGE_SIZE;
	spin_lock_init(&ddata->lock);
	snd_soc_component_set_drvdata(component, ddata);

	ddata->buf = devm_kzalloc(component->dev, ddata->len, GFP_KERNEL);
	if (!ddata->buf)
		return -ENOMEM;

	ddata->regmap = devm_regmap_init(component->dev, NULL, component,
					 &motmdm_regmap);
	if (IS_ERR(ddata->regmap)) {
		error = PTR_ERR(ddata->regmap);
		dev_err(component->dev, "%s: Failed to allocate regmap: %d\n",
			__func__, error);

		return error;
	}

	mot_dlci->drvdata = ddata;
	mot_dlci->line = MOTMDM_DLCI2;
	mot_dlci->notify = motmdm_soc_notify;

	error = motmdm_register_dlci(ddata->modem, mot_dlci);
	if (error)
		return error;

	error = motmdm_parse_tdm(component);
	if (error)
		goto unregister_dlci;

	regcache_sync(ddata->regmap);

	error = motmdm_send_command(ddata->modem, mot_dlci, 1000,
					 cmd, strlen(cmd),
					 ddata->buf, ddata->len);
	if (error < 0)
		goto unregister_dlci;

	return motmdm_disable_primary_dai(ddata->component);

unregister_dlci:
	motmdm_unregister_dlci(ddata->modem, mot_dlci);

	return error;
}

static void motmdm_soc_remove(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata;
	struct motmdm_dlci *mot_dlci;

	ddata = snd_soc_component_get_drvdata(component);
	mot_dlci = &ddata->mot_dlci;

	motmdm_unregister_dlci(ddata->modem, mot_dlci);
}

static struct snd_soc_component_driver soc_codec_dev_motmdm = {
	.probe = motmdm_soc_probe,
	.remove = motmdm_soc_remove,
	.controls = motmdm_snd_controls,
	.num_controls = ARRAY_SIZE(motmdm_snd_controls),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
	.non_legacy_dai_naming = 1,
};

static int motmdm_codec_probe(struct platform_device *pdev)
{
	struct device_node *codec_node =
		of_get_child_by_name(pdev->dev.parent->of_node, "audio-codec");

	pdev->dev.of_node = codec_node;

	return devm_snd_soc_register_component(&pdev->dev,
					       &soc_codec_dev_motmdm,
					       motmdm_dai,
					       ARRAY_SIZE(motmdm_dai));
}

static struct platform_driver motmdm_driver = {
	.probe = motmdm_codec_probe,
	.driver = {
		.name = "mot-mdm6600-codec",
	},
};
module_platform_driver(motmdm_driver);

MODULE_ALIAS("platform:motmdm-codec");
MODULE_DESCRIPTION("ASoC Motorola Mapphone MDM6600 codec driver");
MODULE_AUTHOR("Tony Lindgren <tony@atomide.com>");
MODULE_LICENSE("GPL v2");
