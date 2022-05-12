// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola Mapphone MDM6600 voice call audio support
 * Copyright 2018 - 2020 Tony Lindgren <tony@atomide.com>
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

#define MOTMDM_HEADER_LEN	5			/* U1234 */

#define MOTMDM_AUDIO_RESP_LEN	6			/* U1234+XXXX= */
#define MOTMDM_AUDIO_MAX_LEN	128

#define MOTMDM_VOICE_DLCI	1
#define MOTMDM_VOICE_RESP_LEN	7			/* U1234~+CIEV= */

struct motmdm_driver_data {
	struct snd_soc_component *component;
	struct snd_soc_dai *master_dai;
	struct device *modem;
	struct gsm_serdev_dlci dlci;
	struct regmap *regmap;
	unsigned char *buf;
	size_t len;
	unsigned int parsed:1;
	unsigned int enabled:1;
	spinlock_t lock;	/* enable/disabled lock */
	struct mutex mutex;	/* for sending commands */
	wait_queue_head_t read_queue;

	int (*receive_buf_orig)(struct gsm_serdev_dlci *ops,
				const unsigned char *buf,
				size_t len);
	unsigned int dtmf_val;
	unsigned int dtmf_en;
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

static int motmdm_send_command(struct motmdm_driver_data *ddata,
			       const u8 *buf, int len)
{
	struct device *dev = ddata->component->dev;
	const int timeout_ms = 1000;
	unsigned char cmd[MOTMDM_AUDIO_MAX_LEN];
	int ret, cmdlen;

	cmdlen = len + 5 + 1;
	if (cmdlen > MOTMDM_AUDIO_MAX_LEN)
		return -EINVAL;

	mutex_lock(&ddata->mutex);
	memset(ddata->buf, 0, ddata->len);
	ddata->parsed = false;
	snprintf(cmd, cmdlen, "U%04li%s", jiffies % 10000, buf);
	dev_dbg(dev, "%s: sending %s\n", __func__, cmd);
	ret = serdev_ngsm_write(ddata->modem, &ddata->dlci, cmd, cmdlen);
	if (ret < 0)
		goto out_unlock;

	ret = wait_event_timeout(ddata->read_queue, ddata->parsed,
				 msecs_to_jiffies(timeout_ms));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto out_unlock;
	} else if (ret < 0) {
		goto out_unlock;
	}

	if (strstr(ddata->buf, "ERROR")) {
		dev_err(dev, "command %s error %s\n", cmd, ddata->buf);
		ret = -EPIPE;
	}

	ret = len;

out_unlock:
	mutex_unlock(&ddata->mutex);

	return ret;
}

/* Handle U1234+XXXX= style command response */
static int motmdm_receive_data(struct gsm_serdev_dlci *dlci,
			       const unsigned char *buf,
			       size_t len)
{
	struct motmdm_driver_data *ddata = dlci->drvdata;
	struct device *dev = ddata->component->dev;

	if (len > MOTMDM_AUDIO_MAX_LEN)
		len = MOTMDM_AUDIO_MAX_LEN;

	if (len <= MOTMDM_HEADER_LEN)
		return 0;

	if (buf[MOTMDM_HEADER_LEN] == '~') {
		dev_warn(dev, "unhandled message: %s\n", ddata->buf);

		return 0;
	}

	snprintf(ddata->buf, len - MOTMDM_HEADER_LEN, buf + MOTMDM_HEADER_LEN);
	dev_dbg(dev, "%s: received: %s\n", __func__, ddata->buf);
	ddata->parsed = true;
	wake_up(&ddata->read_queue);

	return len;
}

static int motmdm_read_reg(void *context, unsigned int reg,
			   unsigned int *value)
{
	struct snd_soc_component *component = context;
	struct motmdm_driver_data *ddata = snd_soc_component_get_drvdata(component);
	const unsigned char *cmd;
	unsigned int val;
	int error;

	cmd = motmd_read_fmt[reg];
	error = motmdm_send_command(ddata, cmd, strlen(cmd));
	if (error < 0) {
		dev_err(component->dev, "%s: %s failed with %i\n",
			__func__, cmd, error);

		return error;
	}

	error = kstrtouint(ddata->buf + MOTMDM_AUDIO_RESP_LEN, 0, &val);
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

	error = motmdm_send_command(ddata, cmd, strlen(cmd));
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

static const char * const motmdm_tonegen_dtmf_key_txt[] = {
	"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D",
	"*", "#"
};

static SOC_ENUM_SINGLE_EXT_DECL(motmd_tonegen_dtmf_enum,
				motmdm_tonegen_dtmf_key_txt);

static int motmdm_dtmf_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ddata->dtmf_val;

	return 0;
}

static int motmdm_dtmf_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);

	ddata->dtmf_val = ucontrol->value.enumerated.item[0];

	return 0;
}

static int motmdm_tonegen_dtmf_send_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ddata->dtmf_en;

	return 0;
}

static int motmdm_tonegen_dtmf_send_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct motmdm_driver_data *ddata =
		snd_soc_component_get_drvdata(component);
	const unsigned char *cmd, *fmt = "AT+DTSE=%s,%i";
	const char *tone = "";
	int error;

	if (!ddata->enabled)
		return 0;

	ddata->dtmf_en = ucontrol->value.enumerated.item[0];
	if (ddata->dtmf_en)
		tone = motmdm_tonegen_dtmf_key_txt[ddata->dtmf_val];

	/* Value 0 enables tone generator, 1 disables it */
	cmd = kasprintf(GFP_KERNEL, fmt, tone, !ddata->dtmf_en);

	error = motmdm_send_command(ddata, cmd, strlen(cmd));
	if (error < 0) {
		dev_err(component->dev, "%s: %s failed with %i\n",
			__func__, cmd, error);
		goto free;
	}

free:
	kfree(cmd);

	return error;
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

	daifmt = snd_soc_daifmt_parse_format(master_ep, NULL);
	snd_soc_daifmt_parse_clock_provider_as_phandle(master_ep, NULL,
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
	SOC_ENUM_EXT("Call DTMF", motmd_tonegen_dtmf_enum,
		     motmdm_dtmf_get,
		     motmdm_dtmf_put),
	SOC_SINGLE_BOOL_EXT("Call DTMF Send", 0,
			    motmdm_tonegen_dtmf_send_get,
			    motmdm_tonegen_dtmf_send_put),
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

/* Parses the voice call state from unsolicited notifications on dlci1 */
static void motmdm_voice_get_state(struct motmdm_driver_data *ddata,
				   const unsigned char *buf,
				   size_t len)
{
	struct device *dev = ddata->component->dev;
	bool enable, notify = false;
	unsigned char state[5 + 1];
	unsigned long flags;

	if (len < MOTMDM_HEADER_LEN + MOTMDM_VOICE_RESP_LEN + 5)
		return;

	/* We only care about the unsolicted messages */
	if (buf[MOTMDM_HEADER_LEN] != '~')
		return;

	if (strncmp(buf + MOTMDM_HEADER_LEN + 1, "+CIEV=", 6))
		return;

	snprintf(state, 5 + 1, buf + MOTMDM_HEADER_LEN +
		 MOTMDM_VOICE_RESP_LEN);
	dev_info(dev, "%s: ciev=%s\n", __func__, state);

	if (!strncmp(state, "1,1,0", 5) ||	/* connecting */
	    !strncmp(state, "1,4,0", 5) ||	/* incoming call */
	    !strncmp(state, "1,2,0", 5))	/* connected */
		enable = true;
	else if (!strncmp(state, "1,0,0", 5) ||	/* disconnected */
		!strncmp(state, "1,0,2", 5) ||	/* call failed */
		!strncmp(state, "1,0,7", 5) ||	/* line busy */
		!strncmp(state, "1,0,9", 5))	/* emergency calls only */
		enable = false;
	else
		return;

	spin_lock_irqsave(&ddata->lock, flags);
	if (ddata->enabled != enable) {
		ddata->enabled = enable;
		notify = true;
	}
	spin_unlock_irqrestore(&ddata->lock, flags);

	if (!notify)
		return;

	if (enable)
		motmdm_enable_primary_dai(ddata->component);
	else
		motmdm_disable_primary_dai(ddata->component);
}

static int receive_buf_voice(struct gsm_serdev_dlci *ops,
			     const unsigned char *buf,
			     size_t len)
{
	struct motmdm_driver_data *ddata = ops->drvdata;

	motmdm_voice_get_state(ddata, buf, len);
	if (ddata->receive_buf_orig)
		return ddata->receive_buf_orig(ops, buf, len);

	return len;
}

/* Read the voice status from dlci1 and let user space handle rest */
static int motmdm_init_voice_dlci(struct motmdm_driver_data *ddata)
{
	struct gsm_serdev_dlci *dlci;

	dlci = serdev_ngsm_get_dlci(ddata->modem, MOTMDM_VOICE_DLCI);
	if (!dlci)
		return -ENODEV;

	dlci->drvdata = ddata;
	ddata->receive_buf_orig = dlci->receive_buf;
	dlci->receive_buf = receive_buf_voice;

	return 0;
}

static void motmdm_free_voice_dlci(struct motmdm_driver_data *ddata)
{
	struct gsm_serdev_dlci *dlci;

	dlci = serdev_ngsm_get_dlci(ddata->modem, MOTMDM_VOICE_DLCI);
	if (!dlci)
		return;

	dlci->receive_buf = ddata->receive_buf_orig;
	dlci->drvdata = NULL;
}

static int motmdm_soc_probe(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata;
	struct gsm_serdev_dlci *dlci;
	const unsigned char *cmd = "AT+CMUT=0";
	int error;
	u32 line;

	ddata = kzalloc(sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	error = of_property_read_u32(component->dev->of_node, "reg", &line);
	if (error)
		goto free_ddata;

	ddata->component = component;
	ddata->modem = component->dev->parent;
	mutex_init(&ddata->mutex);
	init_waitqueue_head(&ddata->read_queue);
	dlci = &ddata->dlci;
	ddata->len = PAGE_SIZE;
	spin_lock_init(&ddata->lock);
	snd_soc_component_set_drvdata(component, ddata);
	ddata->len = MOTMDM_AUDIO_MAX_LEN;

	ddata->buf = kzalloc(ddata->len, GFP_KERNEL);
	if (!ddata->buf)
		goto free_ddata;

	ddata->regmap = regmap_init(component->dev, NULL, component,
				    &motmdm_regmap);
	if (IS_ERR(ddata->regmap)) {
		error = PTR_ERR(ddata->regmap);
		dev_err(component->dev, "%s: Failed to allocate regmap: %d\n",
			__func__, error);
		goto free_buf;
	}

	dlci->line = line;
	dlci->drvdata = ddata;
	dlci->receive_buf = motmdm_receive_data;

	error = serdev_ngsm_register_dlci(ddata->modem, dlci);
	if (error)
		goto unregister_regmap;

	error = motmdm_parse_tdm(component);
	if (error)
		goto unregister_dlci;

	regcache_sync(ddata->regmap);

	error = motmdm_send_command(ddata, cmd, strlen(cmd));
	if (error < 0)
		goto unregister_dlci;

	error = motmdm_init_voice_dlci(ddata);
	if (error)
		goto unregister_dlci;

	error = motmdm_disable_primary_dai(ddata->component);
	if (error)
		goto unregister_voice;

	return 0;

unregister_voice:
	motmdm_free_voice_dlci(ddata);

unregister_dlci:
	serdev_ngsm_unregister_dlci(ddata->modem, dlci);

unregister_regmap:
	regmap_exit(ddata->regmap);

free_buf:
	kfree(ddata->buf);

free_ddata:
	kfree(ddata);

	return error;
}

static void motmdm_soc_remove(struct snd_soc_component *component)
{
	struct motmdm_driver_data *ddata;
	struct gsm_serdev_dlci *dlci;

	ddata = snd_soc_component_get_drvdata(component);
	dlci = &ddata->dlci;
	motmdm_free_voice_dlci(ddata);
	serdev_ngsm_unregister_dlci(ddata->modem, dlci);
	regmap_exit(ddata->regmap);
	kfree(ddata->buf);
	kfree(ddata);
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

#ifdef CONFIG_OF
static const struct of_device_id motmdm_of_match[] = {
	{ .compatible = "motorola,mapphone-mdm6600-codec" },
	{},
};
MODULE_DEVICE_TABLE(of, motmdm_of_match);
#endif

static struct platform_driver motmdm_driver = {
	.probe = motmdm_codec_probe,
	.driver = {
		.name = "mot-mdm6600-codec",
		.of_match_table = of_match_ptr(motmdm_of_match),
	},
};
module_platform_driver(motmdm_driver);

MODULE_ALIAS("platform:motmdm-codec");
MODULE_DESCRIPTION("ASoC Motorola Mapphone MDM6600 codec driver");
MODULE_AUTHOR("Tony Lindgren <tony@atomide.com>");
MODULE_LICENSE("GPL v2");
