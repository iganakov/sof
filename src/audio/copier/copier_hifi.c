// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2022 Intel Corporation. All rights reserved.
//
// Author: Andrula Song <xiaoyuan.song@intel.com>
#include "copier.h"
#include <sof/common.h>

#if SOF_USE_HIFI(3, COPIER) || SOF_USE_HIFI(4, COPIER) || SOF_USE_HIFI(5, COPIER)

#include <sof/audio/buffer.h>
#include <sof/audio/component_ext.h>
#include <sof/audio/format.h>
#include <sof/audio/pipeline.h>
#include <sof/audio/component.h>
#include <module/module/base.h>
#include <stddef.h>
#include <errno.h>
#include <stdint.h>
#include <xtensa/tie/xt_hifi3.h>
#include "copier_gain.h"

LOG_MODULE_REGISTER(copier_hifi, CONFIG_SOF_LOG_LEVEL);

int apply_attenuation(struct comp_dev *dev, struct copier_data *cd,
		      struct comp_buffer *sink, int frame)
{
	int i;
	int n;
	int nmax;
	int m;
	ae_int32x2 sample;
	ae_valign uu = AE_ZALIGN64();
	ae_valign su = AE_ZALIGN64();
	int remaining_samples = frame * audio_stream_get_channels(&sink->stream);
	uint32_t bytes = frame * audio_stream_frame_bytes(&sink->stream);
	uint32_t *dst = audio_stream_rewind_wptr_by_bytes(&sink->stream, bytes);
	ae_int32x2 *in = (ae_int32x2 *)dst;
	ae_int32x2 *out = (ae_int32x2 *)dst;

	/* only support attenuation in format of 32bit */
	switch (audio_stream_get_frm_fmt(&sink->stream)) {
	case SOF_IPC_FRAME_S16_LE:
		comp_err(dev, "16bit sample isn't supported by attenuation");
		return -EINVAL;
	case SOF_IPC_FRAME_S24_4LE:
	case SOF_IPC_FRAME_S32_LE:
		while (remaining_samples) {
			nmax = audio_stream_samples_without_wrap_s32(&sink->stream, dst);
			in = (ae_int32x2 *)dst;
			out = (ae_int32x2 *)dst;
			uu = AE_LA64_PP(in);
			n = MIN(remaining_samples, nmax);
			m = n >> 1;
			for (i = 0; i < m; i++) {
				AE_LA32X2_IP(sample, uu, in);
				sample = AE_SRAA32(sample, cd->attenuation);
				AE_SA32X2_IP(sample, su, out);
			}
			AE_SA64POS_FP(su, out);
			if (n & 0x01) {
				AE_L32_IP(sample, (ae_int32 *)in, sizeof(ae_int32));
				sample = AE_SRAA32(sample, cd->attenuation);
				AE_S32_L_IP(sample, (ae_int32 *)out, sizeof(ae_int32));
			}
			remaining_samples -= n;
			dst = audio_stream_wrap(&sink->stream, dst + n);
		}

		return 0;
	default:
		comp_err(dev, "unsupported format %d for attenuation",
			 audio_stream_get_frm_fmt(&sink->stream));
		return -EINVAL;
	}
}

void copier_gain_set_basic_params(struct comp_dev *dev, struct dai_data *dd)
{
	struct copier_gain_params *gain_params = dd->gain_data;
	struct ipc4_copier_module_cfg *copier_cfg = dd->dai_spec_config;

	/* Set default gain coefficients */
	for (int i = 0; i < ARRAY_SIZE(gain_params->gain_coeffs); ++i)
		gain_params->gain_coeffs[i] = AE_MOVF16X4_FROMINT64(UNITY_GAIN_4X_Q10);

	gain_params->step_f16 = AE_ZERO16();
	gain_params->init_gain = AE_ZERO16();

	/* Set audio format */
	gain_params->container = copier_cfg->base.audio_fmt.depth;
	gain_params->chanels_count = copier_cfg->base.audio_fmt.channels_count;
}

int copier_gain_set_fade_params(struct comp_dev *dev, struct dai_data *dd,
				uint32_t fade_period, uint32_t frames)
{
	struct copier_gain_params *gain_params = dd->gain_data;
	struct ipc4_copier_module_cfg *copier_cfg = dd->dai_spec_config;
	uint16_t init_gain[4];
	ae_f16 step_f16;

	/* For backward compatibility add a case with default fade transition.
	 * Backward compatibility is referring to clock_on_delay in DMIC blob.
	 */
	if (fade_period == 0) {
		/* Default fade transition delay for speech */
		gain_params->fade_sg_length = frames * GAIN_DEFAULT_SPEECH_TRANS_MS;

		/* Default fade transition delay for high quality mode */
		if (copier_cfg->base.audio_fmt.sampling_frequency > IPC4_FS_16000HZ)
			gain_params->fade_sg_length = frames * GAIN_DEFAULT_HQ_TRANS_MS;

	} else if (fade_period == GAIN_ZERO_TRANS_MS) {
		/* Special case for GAIN_ZERO_TRANS_MS to support zero fade in transition time */
		gain_params->fade_sg_length = 0;
		return 0;
	}

	/* High precision step for fade-in calculation, keeps accurate precision */
	gain_params->step_i64 = MAX_INT64 / gain_params->fade_sg_length;
	uint16_t step_i64_to_i16 = (uint16_t)(gain_params->step_i64 >> I64_TO_I16_SHIFT);

	step_f16 = step_i64_to_i16 * (MAX_GAIN_COEFFS_CNT / gain_params->chanels_count);

	/* Lower precision step for HIFI SIMD fade-in calculation */
	gain_params->step_f16 = step_f16;

	/* Initialization gain for HIFI SIMD addition, depends on channel configuration */
	for (int i = 0; i < MAX_GAIN_COEFFS_CNT; i++)
		init_gain[i] = (i / gain_params->chanels_count) * step_i64_to_i16;

	int ret = memcpy_s(&gain_params->init_gain, sizeof(gain_params->init_gain), init_gain,
			   sizeof(init_gain));
	assert(!ret);

	return 0;
}

inline ae_int16x4 copier_load_slots_and_gain16(ae_int16x4 **addr,
					       ae_valign *align_in,
					       const ae_int16x4 gains)
{
	ae_int16x4 d16_1 = AE_ZERO16();
	ae_int32x2 d32_1 = AE_ZERO32();
	ae_int32x2 d32_2 = AE_ZERO32();

	AE_LA16X4_IC(d16_1, align_in[0], addr[0]);
	AE_MUL16X4(d32_1, d32_2, d16_1, gains);

	/* Saturate if exists by moving to Q31 */
	d32_1 = AE_SLAA32S(d32_1, Q10_TO_Q31_SHIFT);
	d32_2 = AE_SLAA32S(d32_2, Q10_TO_Q31_SHIFT);

	/* Returns desired samples selection */
	return AE_TRUNC16X4F32(d32_1, d32_2);
}

inline void copier_load_slots_and_gain32(ae_int32x2 **addr, ae_valign *align_in,
					 const ae_int16x4 gains, ae_int32x2 *out_d32_l,
					 ae_int32x2 *out_d32_h)
{
	ae_int32x2 d32tmp_h = AE_ZERO32();
	ae_int32x2 d32tmp_l = AE_ZERO32();

	AE_LA32X2_IC(d32tmp_h, align_in[0], addr[0]);
	AE_LA32X2_IC(d32tmp_l, align_in[0], addr[0]);

	/* Apply gains */
	d32tmp_h = AE_MULFP32X16X2RAS_H(d32tmp_h, gains);
	d32tmp_l = AE_MULFP32X16X2RAS_L(d32tmp_l, gains);

	/* Gain is Q10 but treated in AE_MULFP32X16 as Q15,
	 * so we need to compensate by shifting with saturation
	 */
	*out_d32_h = AE_SLAA32S(d32tmp_h, Q10_TO_Q15_SHIFT);
	*out_d32_l = AE_SLAA32S(d32tmp_l, Q10_TO_Q15_SHIFT);
}

int copier_gain_input16(struct comp_buffer *buff, enum copier_gain_state state,
			enum copier_gain_direction dir, struct copier_gain_params *gain_params,
			uint32_t samples)
{
	const size_t frame_bytes = audio_stream_frame_bytes(&buff->stream);
	uint32_t stream_bytes = audio_stream_sample_bytes(&buff->stream) * samples;
	uint32_t sg_count = stream_bytes / frame_bytes;
	uint16_t *dst = audio_stream_get_rptr(&buff->stream);
	const ae_int16x4 gain_i16 = gain_params->gain_coeffs[0];
	ae_valign align_in = AE_ZALIGN64();
	ae_valign align_out = AE_ZALIGN64();
	ae_int16x4 *out_ptr;
	ae_int16x4 *in_ptr;
	ae_int16x4 d_r;
	ae_int16x4 d16_1;
	size_t rest;
	size_t n;
	int nmax;

	switch (state) {
	case STATIC_GAIN:
		/* static gain */
		if (gain_params->unity_gain)
			return 0;

		while (samples) {
			nmax = audio_stream_samples_without_wrap_s16(&buff->stream, dst);
			out_ptr = (ae_int16x4 *)(dst);
			in_ptr = (ae_int16x4 *)(dst);

			AE_LA16X4POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;
			for (n = 0; n < (nmax >> 2); n++) {
				d16_1 = copier_load_slots_and_gain16(&in_ptr, &align_in, gain_i16);
				AE_SA16X4_IC(d16_1, align_out, out_ptr);
			}

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				d_r = copier_load_slots_and_gain16(&in_ptr, &align_in, gain_i16);
				AE_S16_0_IP(AE_MOVAD16_3(d_r), (ae_int16 *)(out_ptr),
							 sizeof(uint16_t));
				if (rest > 1) {
					AE_S16_0_IP(AE_MOVAD16_2(d_r), (ae_int16 *)(out_ptr),
						    sizeof(uint16_t));
					if (rest > 2)
						AE_S16_0_IP(AE_MOVAD16_1(d_r),
							    (ae_int16 *)(out_ptr), 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		break;
	case TRANS_MUTE:
		d16_1 = AE_ZERO16();

		while (samples) {
			nmax = audio_stream_samples_without_wrap_s16(&buff->stream, dst);
			out_ptr = (ae_int16x4 *)(dst);
			in_ptr = (ae_int16x4 *)(dst);

			AE_LA16X4POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;
			for (n = 0; n < (nmax >> 2); n++)
				AE_SA16X4_IC(d16_1, align_out, out_ptr);

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				d_r = AE_ZERO16();
				AE_S16_0_IP(AE_MOVAD16_3(d_r), (ae_int16 *)(out_ptr),
					    sizeof(uint16_t));
				if (rest > 1) {
					AE_S16_0_IP(AE_MOVAD16_2(d_r), (ae_int16 *)(out_ptr),
						    sizeof(uint16_t));
					if (rest > 2)
						AE_S16_0_IP(AE_MOVAD16_1(d_r),
							    (ae_int16 *)(out_ptr), 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		gain_params->silence_sg_count += sg_count;
		break;
	case TRANS_GAIN:
		ae_f16x4 gain_env = (int16_t)(gain_params->gain_env >> I64_TO_I16_SHIFT);

		gain_env = AE_ADD16S(gain_env, gain_params->init_gain);
		while (samples) {
			nmax = audio_stream_samples_without_wrap_s16(&buff->stream, dst);
			out_ptr = (ae_int16x4 *)(dst);
			in_ptr = (ae_int16x4 *)(dst);
			AE_LA16X4POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;
			for (n = 0; n < (nmax >> 2); n++) {
				/* static gain part */
				if (!gain_params->unity_gain)
					d16_1 = copier_load_slots_and_gain16(&in_ptr, &align_in,
									     gain_i16);
				else
					AE_LA16X4_IC(d16_1, align_in, in_ptr);

				/* quadratic fade-in part */
				d16_1 = AE_MULFP16X4S(d16_1, gain_env);
				d16_1 = AE_MULFP16X4S(d16_1, gain_env);

				AE_SA16X4_IC(d16_1, align_out, out_ptr);
				if (dir == ADDITION)
					gain_env = AE_ADD16S(gain_env, gain_params->step_f16);
				else
					gain_env = AE_SUB16S(gain_env, gain_params->step_f16);
			}

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				/* static gain part */
				if (!gain_params->unity_gain)
					d_r = copier_load_slots_and_gain16(&in_ptr, &align_in,
									   gain_i16);
				else
					AE_LA16X4_IC(d_r, align_in, in_ptr);

				/* quadratic fade-in part */
				d_r = AE_MULFP16X4S(d_r, gain_env);
				d_r = AE_MULFP16X4S(d_r, gain_env);

				AE_S16_0_IP(AE_MOVAD16_3(d_r), (ae_int16 *)(out_ptr),
					    sizeof(uint16_t));
				if (rest > 1) {
					AE_S16_0_IP(AE_MOVAD16_2(d_r), (ae_int16 *)(out_ptr),
						    sizeof(uint16_t));
					if (rest > 2)
						AE_S16_0_IP(AE_MOVAD16_1(d_r),
							    (ae_int16 *)(out_ptr), 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		gain_params->fade_in_sg_count += sg_count;

		if (dir == ADDITION)
			gain_params->gain_env += gain_params->step_i64 * sg_count;
		else
			gain_params->gain_env -= gain_params->step_i64 * sg_count;
	}
	return 0;
}

int copier_gain_input32(struct comp_buffer *buff, enum copier_gain_state state,
			enum copier_gain_direction dir, struct copier_gain_params *gain_params,
			uint32_t samples)
{
	const size_t frame_bytes = audio_stream_frame_bytes(&buff->stream);
	uint32_t stream_bytes = audio_stream_sample_bytes(&buff->stream) * samples;
	uint32_t sg_count = stream_bytes / frame_bytes;
	uint32_t rest; //= samples % 4;
	uint32_t *dst = audio_stream_get_rptr(&buff->stream);
	ae_int16x4 gain_i16 = gain_params->gain_coeffs[0];
	ae_valign align_in = AE_ZALIGN64();
	ae_valign align_out = AE_ZALIGN64();
	ae_int32x2 d32_h = AE_ZERO32();
	ae_int32x2 d32_l = AE_ZERO32();
	ae_int32x2 r_d32_h = AE_ZERO32();
	ae_int32x2 r_d32_l = AE_ZERO32();
	ae_int32x2 *out_ptr;
	ae_int32x2 *in_ptr;
	size_t n;
	int nmax;

	switch (state) {
	case STATIC_GAIN:
		if (gain_params->unity_gain)
			return 0;

		while (samples) {
			nmax = audio_stream_samples_without_wrap_s32(&buff->stream, dst);
			out_ptr = (ae_int32x2 *)(dst);
			in_ptr = (ae_int32x2 *)(dst);
			AE_LA32X2POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;

			for (n = 0; n < (nmax >> 2); n++) {
				/* static gain */
				copier_load_slots_and_gain32(&in_ptr, &align_in, gain_i16,
							     &d32_h, &d32_l);
				AE_SA32X2_IC(d32_h, align_out, out_ptr);
				AE_SA32X2_IC(d32_l, align_out, out_ptr);
			}

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				/* static gain */
				copier_load_slots_and_gain32(&in_ptr, &align_in, gain_i16,
							     &r_d32_h, &r_d32_l);

				if (rest > 1) {
					AE_SA32X2_IC(r_d32_l, align_out, out_ptr);
					if (rest > 2) {
						ae_int32 tmp = AE_MOVAD32_H(r_d32_h);

						AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
					}
				} else {
					ae_int32 tmp = AE_MOVAD32_H(r_d32_l);

					AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		break;
	case TRANS_MUTE:
		d32_l = AE_ZERO32();
		while (samples) {
			nmax = audio_stream_samples_without_wrap_s32(&buff->stream, dst);
			out_ptr = (ae_int32x2 *)(dst);
			in_ptr = (ae_int32x2 *)(dst);
			AE_LA32X2POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;

			for (size_t n = 0; n < (nmax >> 2); n++) {
				AE_SA32X2_IC(d32_l, align_out, out_ptr);
				AE_SA32X2_IC(d32_l, align_out, out_ptr);
			}

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				r_d32_l = AE_ZERO32();
				if (rest > 1) {
					AE_SA32X2_IC(r_d32_l, align_out, out_ptr);
					if (rest > 2) {
						ae_int32 tmp = AE_MOVAD32_H(r_d32_h);

						AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
					}
				} else {
					ae_int32 tmp = AE_MOVAD32_H(r_d32_l);

					AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		gain_params->silence_sg_count += sg_count;
		break;
	case TRANS_GAIN:
		ae_f16x4 gain_env = (int16_t)(gain_params->gain_env >> I64_TO_I16_SHIFT);

		gain_env = AE_ADD16S(gain_env, gain_params->init_gain);
		while (samples) {
			nmax = audio_stream_samples_without_wrap_s32(&buff->stream, dst);
			out_ptr = (ae_int32x2 *)(dst);
			in_ptr = (ae_int32x2 *)(dst);
			AE_LA32X2POS_PC(align_in, in_ptr);
			nmax = MIN(samples, nmax);
			rest = nmax & 0x3;
			for (n = 0; n < (nmax >> 2); n++) {
				/* static gain part */
				if (!gain_params->unity_gain) {
					copier_load_slots_and_gain32(&in_ptr, &align_in, gain_i16,
								     &d32_h, &d32_l);
				} else {
					AE_LA32X2_IC(d32_h, align_in, in_ptr);
					AE_LA32X2_IC(d32_l, align_in, in_ptr);
				}
				/* quadratic fade-in part */
				d32_h = AE_MULFP32X16X2RAS_H(d32_h, gain_env);
				d32_h = AE_MULFP32X16X2RAS_H(d32_h, gain_env);
				d32_l = AE_MULFP32X16X2RAS_L(d32_l, gain_env);
				d32_l = AE_MULFP32X16X2RAS_L(d32_l, gain_env);
				AE_SA32X2_IC(d32_h, align_out, out_ptr);
				AE_SA32X2_IC(d32_l, align_out, out_ptr);

				if (dir == ADDITION)
					gain_env = AE_ADD16S(gain_env, gain_params->step_f16);
				else
					gain_env = AE_SUB16S(gain_env, gain_params->step_f16);
			}

			AE_SA64POS_FP(align_out, out_ptr);
			if (rest) {
				/* static gain part */
				if (!gain_params->unity_gain) {
					copier_load_slots_and_gain32(&in_ptr, &align_in, gain_i16,
								     &r_d32_h, &r_d32_l);
				} else {
					AE_LA32X2_IC(r_d32_h, align_in, in_ptr);
					AE_LA32X2_IC(r_d32_l, align_in, in_ptr);
				}
				/* quadratic fade-in part */
				r_d32_h = AE_MULFP32X16X2RAS_H(r_d32_h, gain_env);
				r_d32_h = AE_MULFP32X16X2RAS_H(r_d32_h, gain_env);
				r_d32_l = AE_MULFP32X16X2RAS_L(r_d32_l, gain_env);
				r_d32_l = AE_MULFP32X16X2RAS_L(r_d32_l, gain_env);

				if (rest > 1) {
					AE_SA32X2_IC(r_d32_l, align_out, out_ptr);
					if (rest > 2) {
						ae_int32 tmp = AE_MOVAD32_H(r_d32_h);

						AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
					}
				} else {
					ae_int32 tmp = AE_MOVAD32_H(r_d32_l);

					AE_S32_L_XC(tmp, (ae_int32 *)out_ptr, 0);
				}
			}
			samples -= nmax;
			dst = audio_stream_wrap(&buff->stream, dst + nmax);
		}
		gain_params->fade_in_sg_count += sg_count;

		if (dir == ADDITION)
			gain_params->gain_env += gain_params->step_i64 * sg_count;
		else
			gain_params->gain_env -= gain_params->step_i64 * sg_count;
	}
	return 0;
}

#endif
