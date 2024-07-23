// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2024 Intel Corporation. All rights reserved.
//
// Author: Ievgen Ganakov <ievgen.ganakov@intel.com>

#include <sof/trace/trace.h>
#include <ipc4/base-config.h>
#include <sof/audio/component_ext.h>
#include <module/module/base.h>
#include "copier.h"
#include "copier_gain.h"

LOG_MODULE_DECLARE(copier, CONFIG_SOF_LOG_LEVEL);

int copier_gain_set_params(struct comp_dev *dev, struct dai_data *dd)
{
	struct processing_module *mod = comp_mod(dev);
	struct copier_data *cd = module_get_private_data(mod);
	struct ipc4_base_module_cfg *ipc4_cfg = &cd->config.base;
	uint32_t sampling_freq = ipc4_cfg->audio_fmt.sampling_frequency;
	uint32_t frames = sampling_freq / 1000;
	struct copier_gain_params *gain_params = dd->gain_data;
	uint32_t fade_period;
	int ret;

	/* Set basic gain parameters */
	copier_gain_set_basic_params(dev, dd);

	switch (dd->dai->type) {
	default:
		fade_period = 0;
		break;
	}

	/* Set fade parameters */
	ret = copier_gain_set_fade_params(dev, dd, fade_period, frames);
	if (ret)
		comp_err(dev, "Failed to set fade params");

	return ret;
}

int copier_gain_input(struct comp_dev *dev, struct comp_buffer *buff,
		      struct copier_gain_params *gain_params,
		      enum copier_gain_direction dir, uint32_t stream_bytes)
{
	enum sof_ipc_frame frame_fmt = audio_stream_get_frm_fmt(&buff->stream);
	uint32_t samples = stream_bytes / audio_stream_sample_bytes(&buff->stream);
	enum copier_gain_state state;

	if (!gain_params)
		return -EINVAL;

	state = copier_gain_eval_state(gain_params);

	comp_info(dev, "copier selected gain state %d", state);

	switch (frame_fmt) {
	case SOF_IPC_FRAME_S16_LE:
		return copier_gain_input16(buff, state, dir, gain_params, samples);
	case SOF_IPC_FRAME_S32_LE:
		return copier_gain_input32(buff, state, dir, gain_params, samples);
	default:
		comp_err(dev, "unsupported frame format %d for copier gain", frame_fmt);
		return -EINVAL;
	}
}

enum copier_gain_state copier_gain_eval_state(struct copier_gain_params *gain_params)
{
	enum copier_gain_state state = STATIC_GAIN;

	if (gain_params->silence_sg_count < gain_params->silence_sg_length)
		state = TRANS_MUTE;
	else if ((gain_params->fade_in_sg_count < gain_params->fade_sg_length) &&
		 (gain_params->fade_sg_length != 0))
		state = TRANS_GAIN;

	return state;
}

int copier_gain_dma_control(uint32_t node_id, const uint32_t *config_data,
			    size_t config_size, enum sof_ipc_dai_type dai_type)
{
	union ipc4_connector_node_id node = (union ipc4_connector_node_id)node_id;
	struct gain_dma_control_data *gain_data = NULL;
	struct ipc_comp_dev *icd = NULL;
	struct processing_module *mod = NULL;
	struct copier_data *cd = NULL;
	struct ipc *ipc = ipc_get();
	struct list_item *clist;
	struct comp_dev *dev;
	int ret;

	list_for_item(clist, &ipc->comp_list) {
		icd = container_of(clist, struct ipc_comp_dev, list);

		if (icd->type != COMP_TYPE_COMPONENT)
			continue;

		dev = icd->cd;

		if (dev->ipc_config.type != SOF_COMP_DAI)
			continue;

		mod = comp_mod(dev);
		cd = module_get_private_data(mod);

		switch (dai_type) {
		default:
			comp_warn(dev, "Gain DMA control: no dai type=%d found", dai_type);
			break;
		}

		ret = copier_set_gain(dev, cd->dd[0], gain_data);
		if (ret)
			comp_err(dev, "Gain DMA control: failed to set gain");

		return ret;
	}

	comp_err(dev, "Gain DMA control: no dai component found dai type=%d", dai_type);
	return -ENODEV;
}

int copier_set_gain(struct comp_dev *dev, struct dai_data *dd,
		    struct gain_dma_control_data *gain_data)
{
	struct copier_gain_params *gain_params = dd->gain_data;
	struct ipc4_copier_module_cfg *copier_cfg = dd->dai_spec_config;
	uint16_t static_gain[MAX_GAIN_COEFFS_CNT];
	int ret;

	if (!gain_data) {
		comp_err(dev, "Gain data is NULL");
		return -EINVAL;
	}

	/* Set gain coefficients */
	comp_info(dev, "Update gain coefficients from DMA_CONTROL ipc");

	ret = memcpy_s(static_gain, sizeof(static_gain), gain_data->gain_coeffs,
		       sizeof(gain_data->gain_coeffs));
	assert(!ret);

	for (size_t i = 0; i < MAX_GAIN_COEFFS_CNT; i++)
		static_gain[i] = static_gain[i % copier_cfg->base.audio_fmt.channels_count];

	ret = memcpy_s(gain_params->gain_coeffs, sizeof(gain_params->gain_coeffs),
		       static_gain, sizeof(static_gain));
	assert(!ret);

	gain_params->unity_gain = copier_is_unity_gain(gain_params);

	return 0;
}
