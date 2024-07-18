/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2024 Intel Corporation. All rights reserved.
 *
 * Author: Ievgen Ganakov <ievgen.ganakov@intel.com>
 */

#ifndef __SOF_COPIER_GAIN_H__
#define __SOF_COPIER_GAIN_H__

#include <sof/audio/buffer.h>
#include <xtensa/tie/xt_hifi3.h>
#include <ipc4/base_fw.h>
#include <ipc/dai.h>

/**
 * @file copier_gain.h
 * @brief Header file containing definitions and functions related to audio gain
 * processing for a copier module.
 *
 * This file provides functions, constants and structure defifnitions for applying gain to
 * input audio buffers, both in 16-bit and 32-bit formats. The gain can be applied in
 * different directions (addition or subtraction) and different states (mute, gain, static).
 *
 * @note This file assumes the presence of a `comp_buffer` structure and the `ae_int16x4`,
 * `ae_int32x2`, `ae_f16x4`, and `ae_valign` types.
 */

#define MAX_GAIN_COEFFS_CNT 4 /**< Maximum number of gain coefficients */

/* Common const values for applying gain feature */
#define Q10_TO_Q31_SHIFT 6
#define Q10_TO_Q15_SHIFT 5

/* 16x2 store operation requires shift to middle part of 32 bit register */
#define I64_TO_I16_SHIFT  48
#define MIDDLE_PART_SHIFT 8

/* Unit gain in q10 format applied by default */
#define UNITY_GAIN_4X_Q10  0x0400040004000400ULL
#define UNITY_GAIN_GENERIC 0x400
#define MAX_INT64          0x7FFFFFFFFFFFFFFFULL

/* Default volume transition delays */
#define GAIN_DEFAULT_HQ_TRANS_MS     500
#define GAIN_DEFAULT_SPEECH_TRANS_MS 100
#define GAIN_ZERO_TRANS_MS           0xFFFF

struct dai_data;

/**
 * @brief Enumeration representing the state of the copier gain processing.
 */
enum copier_gain_state {
	TRANS_MUTE = 0, /**< Transition to mute state */
	TRANS_GAIN = 1, /**< Transition to gain state */
	STATIC_GAIN = 2, /**< Static gain state */
};

/**
 * @brief Enumeration representing the direction of the copier gain processing.
 */
enum copier_gain_direction {
	ADDITION = 0,	/**< Addition gain direction */
	SUBTRACTION = 1 /**< Subtraction gain direction */
};

/**
 * @brief Structure representing the parameters for copier gain processing.
 */
struct copier_gain_params {
#if SOF_USE_HIFI(3, COPIER) || SOF_USE_HIFI(4, COPIER) || SOF_USE_HIFI(5, COPIER)
	/**< Input gain coefficients in Q10 format */
	ae_int16x4 gain_coeffs[ROUND_UP(MAX_GAIN_COEFFS_CNT, 4) >> 2];
	/**< Step for fade-in lower precision */
	ae_f16x4 step_f16;
	/**< Initial gain depending on the number of channels */
	ae_f16x4 init_gain;
#else /* Generic version of gain processing */
	/**< Input gain coefficients */
	uint16_t gain_coeffs[MAX_GAIN_COEFFS_CNT];
	/**< Step for fade-in */
	uint16_t step_f16;
	/**< Initial gain */
	uint16_t init_gain[MAX_GAIN_COEFFS_CNT];
#endif
	bool unity_gain; /**< Indicates unity gain coefficients, no processing is required */
	uint32_t silence_sg_count;  /**< Accumulates sample group spent on silence */
	uint32_t fade_in_sg_count;  /**< Accumulates sample group spent on fade-in */
	uint32_t silence_sg_length; /**< Total count of sample group spent on silence */
	uint32_t fade_sg_length;    /**< Total count of sample group spent on fade-in */
	uint64_t gain_env;  /**< Gain envelope for fade-in calculated in high precision */
	uint64_t step_i64;  /**< Step for fade-in envelope in high precision */
	uint16_t container; /**< Size of sample container (in number of bits) */
	uint16_t chanels_count; /**< Number of channels */
};

/** Gain Coefficients IO Control
 *
 * This parameter is sent by the driver to add/modify a static gain.
 * Coefficients are encoded in Q10 format.
 */
struct gain_dma_control_data {
	uint16_t gain_coeffs[MAX_GAIN_COEFFS_CNT];
} __packed __aligned(4);

/**
 * @brief Sets gain parameters.
 *
 * This function sets the gain parameters for the copier component specified by
 * the given device and DAI data.
 *
 * @param dev The pointer to the component device structure.
 * @param dd The pointer to the DAI data structure.
 * @return 0 on success, negative error code on failure.
 */
int copier_gain_set_params(struct comp_dev *dev, struct dai_data *dd);

/**
 * @brief Sets the basic gain parameters.
 *
 * This function sets the basic gain parameters for the copier component specified
 * by the given device and DAI data.
 *
 * @param dev The pointer to the component device structure.
 * @param dd The pointer to the DAI data structure.
 */
void copier_gain_set_basic_params(struct comp_dev *dev, struct dai_data *dd);

/**
 * @brief Sets the gain fade parameters.
 *
 * This function sets the fade gain parameters for the copier component specified
 * by the given device and DAI data.
 *
 * @param dev The pointer to the component device structure.
 * @param dd The pointer to the DAI data structure.
 * @param fade_period The fade period in milliseconds.
 * @param frames The number of frames to fade.
 * @return 0 on success, negative error code on failure.
 */
int copier_gain_set_fade_params(struct comp_dev *dev, struct dai_data *dd,
				uint32_t fade_period, uint32_t frames);

/**
 * @brief Applies gain to a 16-bit container size.
 *
 * This function applies gain to the input audio buffer. There are three gain modes
 * supported: static gain, mute, and gain transition (fade-in or fade-out).
 *
 * @param buff Pointer to the input audio buffer.
 * @param state The state of the gain processing.
 * @param dir The direction of the gain processing.
 * @param copy_samples The number of samples to be processed.
 */
int copier_gain_input16(struct comp_buffer *buff, enum copier_gain_state state,
			enum copier_gain_direction dir, struct copier_gain_params *gain_params,
			uint32_t samples);

/**
 * @brief Applies gain to a 32-bit container size.
 *
 * This function applies gain to the input audio buffer. There are three gain modes
 * supported: static gain, mute, and gain transition (fade-in or fade-out).
 *
 * @param buff Pointer to the input audio buffer.
 * @param state The state of the gain processing.
 * @param dir The direction of the gain processing.
 * @param copy_samples The number of samples to be processed.
 */
int copier_gain_input32(struct comp_buffer *buff, enum copier_gain_state state,
			enum copier_gain_direction dir,
			struct copier_gain_params *gain_params, uint32_t samples);

/**
 * @brief Applies gain to the input audio buffer, selects the appropriate gain method.
 *
 * @param dev The pointer to the comp_dev structure representing the audio component device.
 * @param buff The pointer to the comp_buffer structure representing the input buffer.
 * @param gain_params The pointer to the copier_gain_params structure.
 * @param dir The direction of the gain adjustment (Addition or Substraction).
 * @param stream_bytes The number of bytes in the input buffer.
 * @return 0 on success, negative error code on failure.
 */
int copier_gain_input(struct comp_dev *dev, struct comp_buffer *buff,
					  struct copier_gain_params *gain_params,
					  enum copier_gain_direction dir, uint32_t stream_bytes);

/**
 * Evaluates appropriate gain mode based on the current gain parameters
 *
 * @param gain_params The pointer to the copier_gain_params structure.
 * @return The state of the copier gain (enum copier_gain_state).
 */
enum copier_gain_state copier_gain_eval_state(struct copier_gain_params *gain_params);

/**
 * Sets/modofy gain for a copier module in runtime .
 *
 * @param dev The copier device structure.
 * @param dd The DAI data structure.
 * @param gain_data The gain control data structure.
 * @return 0 on success, otherwise a negative error code.
 */
int copier_set_gain(struct comp_dev *dev, struct dai_data *dd,
		    struct gain_dma_control_data *gain_data);

/**
 * Checks for uniti gain mode.
 *
 * @param gain_params The copier gain parameters structure.
 * @return true if the gain is set to unity gain, false otherwise.
 */
bool copier_is_unity_gain(struct copier_gain_params *gain_params);

/**
 * Controls the gain for a copier device using DMA Control IPC message.
 *
 * This function retrieves gain data from the DMA Control IPC message and updates
 * corresponding dai device gain params structure.
 *
 * @param node_id Gateway node id.
 * @param config_data The gain configuration data.
 * @param config_size The size of the gain configuration data.
 * @param dai_type The type of the DAI device.
 * @return 0 on success, otherwise a negative error code.
 */
int copier_gain_dma_control(uint32_t node_id, const uint32_t *config_data,
			    size_t config_size, enum sof_ipc_dai_type dai_type);

#endif /* __SOF_COPIER_GAIN_H__ */
