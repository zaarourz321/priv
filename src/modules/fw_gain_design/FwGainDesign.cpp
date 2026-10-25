/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FwGainDesign.cpp
 */

#include "FwGainDesign.hpp"

#include <cmath>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace
{
constexpr char kSdModelPath[] = "/fs/microsd/fw_gain_design_model.bin";
constexpr char kSdModelTemporaryPath[] = "/fs/microsd/fw_gain_design_model.tmp";
constexpr uint32_t kSdModelMagic = 0x46574744U; // "FWGD"
constexpr uint16_t kSdModelVersion = 3;
}

FwGainDesign::FwGainDesign() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_rate_p_param[ROLL] = param_find("FW_RR_P");
	_rate_p_param[PITCH] = param_find("FW_PR_P");
	_rate_p_param[YAW] = param_find("FW_YR_P");
	_rate_i_param[ROLL] = param_find("FW_RR_I");
	_rate_i_param[PITCH] = param_find("FW_PR_I");
	_rate_i_param[YAW] = param_find("FW_YR_I");
	_rate_ff_param[ROLL] = param_find("FW_RR_FF");
	_rate_ff_param[PITCH] = param_find("FW_PR_FF");
	_rate_ff_param[YAW] = param_find("FW_YR_FF");
	_attitude_tc_param[ROLL] = param_find("FW_R_TC");
	_attitude_tc_param[PITCH] = param_find("FW_P_TC");
	_control_surface_count_param = param_find("CA_SV_CS_COUNT");

	for (uint8_t index = 0; index < actuator_servos_s::NUM_CONTROLS; ++index) {
		char parameter_name[20];
		snprintf(parameter_name, sizeof(parameter_name), "CA_SV_CS%u_TYPE", index);
		_control_surface_type_param[index] = param_find(parameter_name);

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			const char axis_name[] = {'R', 'P', 'Y'};
			snprintf(parameter_name, sizeof(parameter_name), "CA_SV_CS%u_TRQ_%c", index, axis_name[axis]);
			_control_surface_torque_param[index][axis] = param_find(parameter_name);
		}
	}

	update_control_surface_mapping();
	updateParams();
	reset_schedule();
}

FwGainDesign::~FwGainDesign()
{
	bool changed = false;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		changed |= restore_axis(static_cast<Axis>(axis));
	}

	if (changed) {
		param_notify_changes();
	}
}

bool FwGainDesign::init()
{
	// The published design is always advisory until one of the explicit axis
	// apply switches is enabled. A 50 Hz publication also gives the optional
	// flight-test rate-setpoint injection a bounded, regular update interval.
	ScheduleOnInterval(20_ms);
	return true;
}

void FwGainDesign::update_control_surface_mapping()
{
	memset(_control_surface_weight, 0, sizeof(_control_surface_weight));
	memset(_control_surface_available, 0, sizeof(_control_surface_available));
	int32_t surface_count = 0;

	if (_control_surface_count_param == PARAM_INVALID
	    || param_get(_control_surface_count_param, &surface_count) != PX4_OK) {
		return;
	}

	const int32_t count = surface_count < 0 ? 0
					  : (surface_count > static_cast<int32_t>(actuator_servos_s::NUM_CONTROLS)
					     ? static_cast<int32_t>(actuator_servos_s::NUM_CONTROLS) : surface_count);

	for (int32_t index = 0; index < count; ++index) {
		int32_t type = 0;

		if (_control_surface_type_param[index] == PARAM_INVALID
		    || param_get(_control_surface_type_param[index], &type) != PX4_OK) {
			continue;
		}

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			const bool matching_type = (axis == ROLL && (type == 1 || type == 2 || type == 15))
					   || (axis == PITCH && type == 3) || (axis == YAW && type == 4);

			if (!matching_type || _control_surface_torque_param[index][axis] == PARAM_INVALID) {
				continue;
			}

			float torque_effectiveness = 0.f;

			if (param_get(_control_surface_torque_param[index][axis], &torque_effectiveness) == PX4_OK
			    && PX4_ISFINITE(torque_effectiveness) && fabsf(torque_effectiveness) > 1e-4f) {
				_control_surface_weight[axis][index] = torque_effectiveness;
				_control_surface_available[axis] = true;
			}
		}
	}
}

bool FwGainDesign::control_surface_input(Axis axis, float &control_input) const
{
	const uint8_t axis_index = static_cast<uint8_t>(axis);

	if (!_control_surface_available[axis_index]
	    || hrt_elapsed_time(&_actuator_servos_sub.get().timestamp) > 100_ms) {
		return false;
	}

	float weighted_command = 0.f;
	float total_weight = 0.f;

	for (uint8_t index = 0; index < actuator_servos_s::NUM_CONTROLS; ++index) {
		const float weight = _control_surface_weight[axis_index][index];

		if (fabsf(weight) <= 1e-4f) {
			continue;
		}

		const float command = _actuator_servos_sub.get().control[index];

		if (!PX4_ISFINITE(command) || fabsf(command) > 1.01f) {
			return false;
		}

		weighted_command += weight * command;
		total_weight += fabsf(weight);
	}

	if (total_weight <= FLT_EPSILON) {
		return false;
	}

	control_input = weighted_command / total_weight;
	return PX4_ISFINITE(control_input);
}

uint8_t FwGainDesign::validate_common_inputs() const
{
	const float values[] = {
		_param_fw_gd_rho.get(), _param_fw_gd_sref.get()
	};

	for (float value : values) {
		if (!PX4_ISFINITE(value) || value <= 0.f) {
			return fw_gain_design_s::AXIS_STATUS_INVALID_INPUT;
		}
	}

	return fw_gain_design_s::AXIS_STATUS_VALID;
}

void FwGainDesign::reset_schedule()
{
	_calculation_id++;
	_speed_index = 0;
	_speed_count = 0;
	_schedule_status = fw_gain_design_s::SCHEDULE_STATUS_INVALID_RANGE;

	const float vmin = _param_fw_gd_vmin.get();
	const float vmax = _param_fw_gd_vmax.get();
	const float vstep = _param_fw_gd_vstep.get();

	if (!PX4_ISFINITE(vmin) || !PX4_ISFINITE(vmax) || !PX4_ISFINITE(vstep)
	    || vmin <= 0.f || vmax < vmin || vstep <= 0.f) {
		return;
	}

	// The last point is clamped exactly to Vmax. This avoids dropping the
	// requested upper end when (Vmax - Vmin) is not an integer number of steps.
	const float point_count = ceilf((vmax - vmin) / vstep) + 1.f;

	if (!PX4_ISFINITE(point_count) || point_count < 1.f || point_count > UINT16_MAX) {
		return;
	}

	_speed_count = static_cast<uint16_t>(point_count);
	_schedule_status = fw_gain_design_s::SCHEDULE_STATUS_VALID;
}

bool FwGainDesign::model_source_is_estimated(Axis axis) const
{
	switch (axis) {
	case ROLL:
		return _param_fw_gd_r_src.get() == fw_gain_design_s::MODEL_SOURCE_FLIGHT_TEST;

	case PITCH:
		return _param_fw_gd_p_src.get() == fw_gain_design_s::MODEL_SOURCE_FLIGHT_TEST;

	case YAW:
		return _param_fw_gd_y_src.get() == fw_gain_design_s::MODEL_SOURCE_FLIGHT_TEST;

	default:
		return false;
	}
}

bool FwGainDesign::estimate_valid(Axis axis) const
{
	return _identified_valid[static_cast<uint8_t>(axis)];
}

void FwGainDesign::calculate_design(float true_airspeed, fw_gain_design_s &design) const
{
	design.airspeed = true_airspeed;
	design.px4_airspeed = NAN;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		design.aircraft_rate_bw[axis] = NAN;
		design.rate_cl_bw[axis] = NAN;
		design.rate_cl_zeta[axis] = NAN;
		design.rate_a[axis] = NAN;
		design.rate_b[axis] = NAN;
		design.surface_rate_p[axis] = NAN;
		design.surface_rate_i[axis] = NAN;
		design.surface_rate_ff[axis] = NAN;
		design.px4_rate_p[axis] = NAN;
		design.px4_rate_i[axis] = NAN;
		design.px4_rate_ff[axis] = NAN;
		design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_INVALID_INPUT;
	}

	for (uint8_t axis = 0; axis < 2; ++axis) {
		design.attitude_p[axis] = NAN;
		design.attitude_tc[axis] = NAN;
	}

	if (!PX4_ISFINITE(true_airspeed) || true_airspeed <= 0.f) {
		return;
	}

	const uint8_t common_input_status = validate_common_inputs();

	if (common_input_status == fw_gain_design_s::AXIS_STATUS_VALID) {
		const float rho = _param_fw_gd_rho.get();
		const float sref = _param_fw_gd_sref.get();
		const float bref = _param_fw_gd_bref.get();
		const float cref = _param_fw_gd_cref.get();

		float damping_coefficient[AXIS_COUNT] = {
			_param_fw_gd_clp.get(), _param_fw_gd_cmq.get(), _param_fw_gd_cnr.get()
		};
		float control_coefficient[AXIS_COUNT] = {
			_param_fw_gd_cldu.get(), _param_fw_gd_cmdu.get(), _param_fw_gd_cndu.get()
		};
		const float control_range[AXIS_COUNT] = {
			_param_fw_gd_r_max.get(), _param_fw_gd_p_max.get(), _param_fw_gd_y_max.get()
		};
		const float inertia[AXIS_COUNT] = {
			_param_fw_gd_ix.get(), _param_fw_gd_iy.get(), _param_fw_gd_iz.get()
		};
		const float reference_length[AXIS_COUNT] = {bref, cref, bref};
		const float bandwidth_ratio[AXIS_COUNT] = {
			_param_fw_gd_r_bwr.get(), _param_fw_gd_p_bwr.get(), _param_fw_gd_y_bwr.get()
		};
		const float damping_ratio[AXIS_COUNT] = {
			_param_fw_gd_r_zeta.get(), _param_fw_gd_p_zeta.get(), _param_fw_gd_y_zeta.get()
		};

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			const Axis model_axis = static_cast<Axis>(axis);

			if (model_source_is_estimated(model_axis)) {
				if (estimate_valid(model_axis)) {
					damping_coefficient[axis] = _identified_damping_derivative[axis];
					control_coefficient[axis] = _identified_control_derivative[axis];

				} else {
					// An estimate is deliberately volatile: it must never silently
					// fall back to a stale manual model when this source is selected.
					damping_coefficient[axis] = NAN;
					control_coefficient[axis] = NAN;
				}
			}
		}

		// The aerodynamic model uses true airspeed and local density. PX4's
		// airspeed scaler uses calibrated/equivalent airspeed, referenced to the
		// ISA sea-level density. Publish that conversion so the two gain forms
		// can be interpreted without an implicit sea-level assumption.
		design.px4_airspeed = design.airspeed * sqrtf(rho / 1.225f);

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			const float length = reference_length[axis];

			if (!PX4_ISFINITE(damping_coefficient[axis]) || !PX4_ISFINITE(control_coefficient[axis])
			    || !PX4_ISFINITE(bandwidth_ratio[axis]) || !PX4_ISFINITE(damping_ratio[axis])
			    || !PX4_ISFINITE(length) || !PX4_ISFINITE(control_range[axis]) || !PX4_ISFINITE(inertia[axis])
			    || length <= 0.f || control_range[axis] <= 0.f || inertia[axis] <= 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_INVALID_INPUT;
				continue;
			}

			// Standard non-dimensional derivatives are defined with p-hat,
			// q-hat or r-hat = body_rate * reference_length / (2 * V).
			// This gives x_dot = a * x + b * u, where u is a PX4 normalized
			// control request and delta = u * control_range.
			const float rate_a = rho * design.airspeed * sref * length * length * damping_coefficient[axis]
					     / (4.f * inertia[axis]);
			const float rate_b = 0.5f * rho * design.airspeed * design.airspeed * sref * length
					     * control_coefficient[axis] * control_range[axis] / inertia[axis];

			design.rate_a[axis] = rate_a;
			design.rate_b[axis] = rate_b;

			if (!PX4_ISFINITE(rate_a) || !PX4_ISFINITE(rate_b)) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_NONFINITE_RESULT;
				continue;
			}

			// The design requires a stable first-order rate plant.
			if (rate_a >= 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_NOT_DAMPED;
				continue;
			}

			// The PX4 rate P/I gains assume that a positive controller command
			// creates a positive body-axis angular acceleration. Put the actuator
			// sign in the allocator/mixer, not in these gains.
			if (rate_b <= 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_NO_CONTROL_AUTHORITY;
				continue;
			}

			const float aircraft_bw = -rate_a;
			const float closed_loop_bw = bandwidth_ratio[axis] * aircraft_bw;

			if (bandwidth_ratio[axis] <= 0.f || damping_ratio[axis] <= 0.f || !PX4_ISFINITE(closed_loop_bw)
			    || closed_loop_bw <= 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_INVALID_DESIGN;
				continue;
			}

			// Plant: G(s) = b / (s + aircraft_bw), controller: C(s) = Kp + Ki/s.
			// Matching s^2 + 2*zeta*wc*s + wc^2 gives the two gains below.
			const float surface_p = (2.f * damping_ratio[axis] * closed_loop_bw - aircraft_bw) / rate_b;
			const float surface_i = closed_loop_bw * closed_loop_bw / rate_b;
			// PX4's direct rate feed-forward has no setpoint derivative. The exact
			// useful term is therefore the equilibrium inverse, cancelling the
			// aerodynamic rate damping at a non-zero constant rate setpoint.
			const float surface_ff = aircraft_bw / rate_b;

			if (!PX4_ISFINITE(surface_p) || !PX4_ISFINITE(surface_i) || !PX4_ISFINITE(surface_ff)) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_NONFINITE_RESULT;
				continue;
			}

			if (surface_p < 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_INVALID_DESIGN;
				continue;
			}

			design.aircraft_rate_bw[axis] = aircraft_bw;
			design.rate_cl_bw[axis] = closed_loop_bw;
			design.rate_cl_zeta[axis] = damping_ratio[axis];
			design.surface_rate_p[axis] = surface_p;
			design.surface_rate_i[axis] = surface_i;
			design.surface_rate_ff[axis] = surface_ff;

			const float airspeed_scale = _param_fw_arsp_scale_en.get() ?
				(_param_fw_airspd_trim.get() / design.px4_airspeed) : 1.f;

			if (!PX4_ISFINITE(airspeed_scale) || airspeed_scale <= 0.f) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_INVALID_INPUT;
				continue;
			}

			// FixedwingRateControl scales P and I outputs by scale^2 and the
			// external FF path by scale. Divide so these are the parameter values
			// that reproduce the physical controller at this speed.
			design.px4_rate_p[axis] = surface_p / (airspeed_scale * airspeed_scale);
			design.px4_rate_i[axis] = surface_i / (airspeed_scale * airspeed_scale);
			design.px4_rate_ff[axis] = surface_ff / airspeed_scale;

			if (!PX4_ISFINITE(design.px4_rate_p[axis]) || !PX4_ISFINITE(design.px4_rate_i[axis])
			    || !PX4_ISFINITE(design.px4_rate_ff[axis])) {
				design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_NONFINITE_RESULT;
				continue;
			}

			design.axis_status[axis] = fw_gain_design_s::AXIS_STATUS_VALID;
		}

		const float attitude_bw_ratio[2] = {_param_fw_gd_r_abwr.get(), _param_fw_gd_p_abwr.get()};

		for (uint8_t axis = ROLL; axis <= PITCH; ++axis) {
			if (design.axis_status[axis] != fw_gain_design_s::AXIS_STATUS_VALID
			    || !PX4_ISFINITE(attitude_bw_ratio[axis]) || attitude_bw_ratio[axis] <= 0.f) {
				continue;
			}

			// ECL's fixed-wing attitude loop is a P controller that emits a rate
			// setpoint. With a sufficiently faster rate loop it behaves as a first
			// order loop; bandwidth separation is therefore the meaningful design
			// variable, rather than a separate outer-loop damping ratio.
			const float attitude_p = attitude_bw_ratio[axis] * design.rate_cl_bw[axis];

			if (PX4_ISFINITE(attitude_p) && attitude_p > 0.f) {
				design.attitude_p[axis] = attitude_p;
				design.attitude_tc[axis] = 1.f / attitude_p;
			}
		}

	} else {
		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			design.axis_status[axis] = common_input_status;
		}
	}
}

void FwGainDesign::reset_estimator(Axis axis)
{
	RateEstimator &estimator = _estimator[static_cast<uint8_t>(axis)];
	estimator = {};

	estimator.initialized = true;
	_identified_valid[static_cast<uint8_t>(axis)] = false;
	_identified_damping_derivative[static_cast<uint8_t>(axis)] = NAN;
	_identified_control_derivative[static_cast<uint8_t>(axis)] = NAN;
	_identified_rate_a[static_cast<uint8_t>(axis)] = NAN;
	_identified_rate_b[static_cast<uint8_t>(axis)] = NAN;
	_identified_airspeed[static_cast<uint8_t>(axis)] = NAN;
	_identification_fit[static_cast<uint8_t>(axis)] = NAN;
	_identification_reject_reason[static_cast<uint8_t>(axis)] = fw_gain_design_s::IDENTIFICATION_REJECT_NOT_RUN;
	_last_estimator_sample = 0;
}

uint32_t FwGainDesign::model_checksum(const PersistentModel &model)
{
	// FNV-1a across explicit fields only. This deliberately excludes any
	// compiler-inserted padding from the on-card file integrity check.
	uint32_t checksum = 2166136261U;
	auto hash_bytes = [&checksum](const void *data, size_t length) {
		const uint8_t *bytes = reinterpret_cast<const uint8_t *>(data);

		for (size_t i = 0; i < length; ++i) {
			checksum ^= bytes[i];
			checksum *= 16777619U;
		}
	};

	hash_bytes(&model.magic, sizeof(model.magic));
	hash_bytes(&model.version, sizeof(model.version));
	hash_bytes(&model.size, sizeof(model.size));
	hash_bytes(&model.valid_mask, sizeof(model.valid_mask));
	hash_bytes(&model.reserved, sizeof(model.reserved));
	hash_bytes(model.damping_derivative, sizeof(model.damping_derivative));
	hash_bytes(model.control_derivative, sizeof(model.control_derivative));
	hash_bytes(model.rate_a, sizeof(model.rate_a));
	hash_bytes(model.rate_b, sizeof(model.rate_b));
	hash_bytes(model.airspeed, sizeof(model.airspeed));

	return checksum;
}

bool FwGainDesign::load_model_from_sd()
{
	const int fd = ::open(kSdModelPath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	PersistentModel model{};
	const ssize_t bytes_read = ::read(fd, &model, sizeof(model));
	const int close_result = ::close(fd);

	if (bytes_read != static_cast<ssize_t>(sizeof(model)) || close_result != 0
	    || model.magic != kSdModelMagic || model.version != kSdModelVersion
	    || model.size != sizeof(PersistentModel) || model.checksum != model_checksum(model)
	    || (model.valid_mask & ~((1U << AXIS_COUNT) - 1U)) != 0 || model.valid_mask == 0) {
		return false;
	}

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		const uint8_t axis_mask = 1U << axis;

		if ((model.valid_mask & axis_mask) != 0) {
			if (!PX4_ISFINITE(model.damping_derivative[axis]) || !PX4_ISFINITE(model.control_derivative[axis])
			    || !PX4_ISFINITE(model.rate_a[axis]) || !PX4_ISFINITE(model.rate_b[axis])
			    || !PX4_ISFINITE(model.airspeed[axis]) || model.damping_derivative[axis] >= 0.f
			    || model.control_derivative[axis] <= 0.f || model.rate_a[axis] >= -0.02f
			    || model.rate_b[axis] <= 0.f || model.airspeed[axis] <= 0.f) {
				return false;
			}

			_identified_valid[axis] = true;
			_identified_damping_derivative[axis] = model.damping_derivative[axis];
			_identified_control_derivative[axis] = model.control_derivative[axis];
			_identified_rate_a[axis] = model.rate_a[axis];
			_identified_rate_b[axis] = model.rate_b[axis];
			_identified_airspeed[axis] = model.airspeed[axis];
		} else {
			_identified_valid[axis] = false;
			_identified_damping_derivative[axis] = NAN;
			_identified_control_derivative[axis] = NAN;
			_identified_rate_a[axis] = NAN;
			_identified_rate_b[axis] = NAN;
			_identified_airspeed[axis] = NAN;
		}
	}

	_sd_model_loaded = true;
	_sd_model_dirty = false;
	_calculation_id++;
	PX4_INFO("loaded identified model from SD card");
	return true;
}

bool FwGainDesign::save_model_to_sd()
{
	if (_param_fw_gd_sd_model.get() == 0) {
		return false;
	}

	PersistentModel model{};
	model.magic = kSdModelMagic;
	model.version = kSdModelVersion;
	model.size = static_cast<uint16_t>(sizeof(PersistentModel));

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		if (!_identified_valid[axis]) {
			continue;
		}

		const uint8_t axis_mask = 1U << axis;
		model.valid_mask |= axis_mask;
		model.damping_derivative[axis] = _identified_damping_derivative[axis];
		model.control_derivative[axis] = _identified_control_derivative[axis];
		model.rate_a[axis] = _identified_rate_a[axis];
		model.rate_b[axis] = _identified_rate_b[axis];
		model.airspeed[axis] = _identified_airspeed[axis];
	}

	if (model.valid_mask == 0) {
		return false;
	}

	model.checksum = model_checksum(model);
	const int fd = ::open(kSdModelTemporaryPath, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

	if (fd < 0) {
		return false;
	}

	const ssize_t bytes_written = ::write(fd, &model, sizeof(model));
	const int sync_result = bytes_written == static_cast<ssize_t>(sizeof(model)) ? ::fsync(fd) : -1;
	const int close_result = ::close(fd);

	if (bytes_written != static_cast<ssize_t>(sizeof(model)) || sync_result != 0 || close_result != 0
	    || ::rename(kSdModelTemporaryPath, kSdModelPath) != 0) {
		::unlink(kSdModelTemporaryPath);
		return false;
	}

	_sd_model_saved = true;
	_sd_model_dirty = false;
	PX4_INFO("saved identified model to SD card");
	return true;
}

bool FwGainDesign::fit_harmonic(const RateEstimator &estimator, const HarmonicSignal &signal, HarmonicFit &fit)
{
	const float sample_count = static_cast<float>(estimator.sample_count);

	if (sample_count < 3.f) {
		return false;
	}

	// Solve x = offset + cosine * cos(phase) + sine * sin(phase). Centering
	// the normal equations removes trim and slow navigation corrections.
	const float centered_cos_sq = estimator.sum_cos_sq - estimator.sum_cos * estimator.sum_cos / sample_count;
	const float centered_sin_sq = estimator.sum_sin_sq - estimator.sum_sin * estimator.sum_sin / sample_count;
	const float centered_cos_sin = estimator.sum_cos_sin - estimator.sum_cos * estimator.sum_sin / sample_count;
	const float centered_x_cos = signal.sum_cos - signal.sum * estimator.sum_cos / sample_count;
	const float centered_x_sin = signal.sum_sin - signal.sum * estimator.sum_sin / sample_count;
	const float determinant = centered_cos_sq * centered_sin_sq - centered_cos_sin * centered_cos_sin;

	if (!PX4_ISFINITE(determinant) || determinant <= FLT_EPSILON) {
		return false;
	}

	fit.cosine = (centered_x_cos * centered_sin_sq - centered_x_sin * centered_cos_sin) / determinant;
	fit.sine = (centered_x_sin * centered_cos_sq - centered_x_cos * centered_cos_sin) / determinant;
	const float mean = signal.sum / sample_count;
	const float total_variance = fmaxf(signal.sum_sq - signal.sum * mean, 0.f);
	const float explained_variance = fit.cosine * centered_x_cos + fit.sine * centered_x_sin;
	const float residual_variance = fmaxf(total_variance - explained_variance, 0.f);
	fit.residual_ratio = sqrtf(residual_variance / fmaxf(total_variance, 1e-6f));

	return PX4_ISFINITE(fit.cosine) && PX4_ISFINITE(fit.sine) && PX4_ISFINITE(fit.residual_ratio);
}

void FwGainDesign::update_estimator(Axis axis, float rate, float control_input, float true_airspeed, float dt, float phase,
				     float elapsed_s)
{
	const uint8_t axis_index = static_cast<uint8_t>(axis);
	RateEstimator &estimator = _estimator[axis_index];

	if (!estimator.initialized || !PX4_ISFINITE(rate) || !PX4_ISFINITE(control_input)
	    || !PX4_ISFINITE(true_airspeed) || true_airspeed <= 0.f
	    || !PX4_ISFINITE(dt) || dt <= 0.f) {
		return;
	}

	const float filter_cutoff_hz = _param_fw_gd_est_fc.get();

	if (!PX4_ISFINITE(filter_cutoff_hz) || filter_cutoff_hz <= 0.f || !PX4_ISFINITE(phase)
	    || !PX4_ISFINITE(elapsed_s)) {
		return;
	}

	// The sensor's angular acceleration is intentionally not used here. At the
	// low frequencies of an aircraft flight test it is derivative-noise
	// dominated. Instead, measure the rate-to-surface frequency response at
	// the explicitly injected sine frequency.
	const float filter_time_constant = 1.f / (2.f * M_PI_F * filter_cutoff_hz);
	const float filter_alpha = math::constrain(dt / (dt + filter_time_constant), 0.001f, 0.95f);

	if (!estimator.filters_initialized) {
		estimator.filtered_rate = rate;
		estimator.filtered_control = control_input;
		estimator.filters_initialized = true;
		return;
	}

	estimator.filtered_rate += filter_alpha * (rate - estimator.filtered_rate);
	estimator.filtered_control += filter_alpha * (control_input - estimator.filtered_control);

	// Exclude the rate-setpoint ramp and one complete sine cycle. This lets the
	// rate loop and surface command reach their periodic response before the
	// frequency response is measured.
	if (elapsed_s < kExcitationRampTimeS + 1.f / _excitation_frequency) {
		return;
	}

	const float cosine = cosf(phase);
	const float sine = sinf(phase);
	estimator.sum_cos += cosine;
	estimator.sum_sin += sine;
	estimator.sum_cos_sq += cosine * cosine;
	estimator.sum_sin_sq += sine * sine;
	estimator.sum_cos_sin += cosine * sine;

	estimator.rate.sum += estimator.filtered_rate;
	estimator.rate.sum_sq += estimator.filtered_rate * estimator.filtered_rate;
	estimator.rate.sum_cos += estimator.filtered_rate * cosine;
	estimator.rate.sum_sin += estimator.filtered_rate * sine;
	estimator.control.sum += estimator.filtered_control;
	estimator.control.sum_sq += estimator.filtered_control * estimator.filtered_control;
	estimator.control.sum_cos += estimator.filtered_control * cosine;
	estimator.control.sum_sin += estimator.filtered_control * sine;
	estimator.airspeed_sum += true_airspeed;
	estimator.accumulation_duration_s += dt;
	estimator.sample_count++;
}

void FwGainDesign::finalize_estimate(Axis axis)
{
	const uint8_t axis_index = static_cast<uint8_t>(axis);
	const RateEstimator &estimator = _estimator[axis_index];
	const float min_samples = static_cast<float>(_param_fw_gd_est_min.get());

	if (!estimator.initialized || estimator.sample_count < min_samples
	    || estimator.accumulation_duration_s * _excitation_frequency < 2.f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_INSUFFICIENT_SAMPLES;
		return;
	}

	const float rho = _param_fw_gd_rho.get();
	const float sref = _param_fw_gd_sref.get();
	const float reference_length[AXIS_COUNT] = {
		_param_fw_gd_bref.get(), _param_fw_gd_cref.get(), _param_fw_gd_bref.get()
	};
	const float inertia[AXIS_COUNT] = {
		_param_fw_gd_ix.get(), _param_fw_gd_iy.get(), _param_fw_gd_iz.get()
	};
	const float control_range[AXIS_COUNT] = {
		_param_fw_gd_r_max.get(), _param_fw_gd_p_max.get(), _param_fw_gd_y_max.get()
	};
	HarmonicFit rate_fit{};
	HarmonicFit control_fit{};

	if (!fit_harmonic(estimator, estimator.rate, rate_fit)
	    || !fit_harmonic(estimator, estimator.control, control_fit)) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_INVALID_INPUT;
		return;
	}

	const float excitation_frequency = _excitation_frequency;
	const float input_delay = _param_fw_gd_est_dly.get();
	const float omega = 2.f * M_PI_F * excitation_frequency;
	const float control_amplitude = sqrtf(control_fit.cosine * control_fit.cosine + control_fit.sine * control_fit.sine);
	const float complex_control_real = control_fit.cosine;
	const float complex_control_imag = -control_fit.sine;
	const float complex_rate_real = rate_fit.cosine;
	const float complex_rate_imag = -rate_fit.sine;
	const float control_power = complex_control_real * complex_control_real + complex_control_imag * complex_control_imag;

	if (!PX4_ISFINITE(excitation_frequency) || excitation_frequency <= 0.f || !PX4_ISFINITE(input_delay)
	    || input_delay < 0.f || !PX4_ISFINITE(control_amplitude) || control_amplitude < 0.01f
	    || !PX4_ISFINITE(control_power) || control_power <= FLT_EPSILON) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_LOW_EXCITATION;
		return;
	}

	const float response_real = (complex_rate_real * complex_control_real
				     + complex_rate_imag * complex_control_imag) / control_power;
	const float response_imag = (complex_rate_imag * complex_control_real
				     - complex_rate_real * complex_control_imag) / control_power;
	const float delay_phase = omega * input_delay;
	const float corrected_response_real = response_real * cosf(delay_phase) - response_imag * sinf(delay_phase);
	const float corrected_response_imag = response_real * sinf(delay_phase) + response_imag * cosf(delay_phase);
	const float response_power = corrected_response_real * corrected_response_real
				     + corrected_response_imag * corrected_response_imag;

	if (!PX4_ISFINITE(response_power) || response_power <= FLT_EPSILON) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_INVALID_INPUT;
		return;
	}

	const float inverse_response_real = corrected_response_real / response_power;
	const float inverse_response_imag = -corrected_response_imag / response_power;

	if (!PX4_ISFINITE(inverse_response_imag) || fabsf(inverse_response_imag) <= FLT_EPSILON) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_LOW_EXCITATION;
		return;
	}

	const float rate_b = omega / inverse_response_imag;
	const float rate_a = -rate_b * inverse_response_real;
	const float airspeed = estimator.airspeed_sum / estimator.sample_count;
	const float length = reference_length[axis_index];
	const float fit = fmaxf(rate_fit.residual_ratio, control_fit.residual_ratio);

	// Publish candidates even when rejected so the flight log tells the operator
	// why a result failed. Only identification_valid permits their use as a
	// model source or for live gain scheduling.
	_identified_airspeed[axis_index] = airspeed;
	_identified_rate_a[axis_index] = rate_a;
	_identified_rate_b[axis_index] = rate_b;
	_identification_fit[axis_index] = fit;
	if (!PX4_ISFINITE(rho) || !PX4_ISFINITE(sref) || !PX4_ISFINITE(length) || !PX4_ISFINITE(inertia[axis_index])
	    || !PX4_ISFINITE(control_range[axis_index]) || !PX4_ISFINITE(airspeed) || !PX4_ISFINITE(rate_a)
	    || !PX4_ISFINITE(rate_b) || !PX4_ISFINITE(fit) || rho <= 0.f || sref <= 0.f || length <= 0.f
	    || inertia[axis_index] <= 0.f || control_range[axis_index] <= 0.f || airspeed <= 0.f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_INVALID_INPUT;
		return;
	}

	if (rate_a >= -0.02f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_UNSTABLE_DAMPING;
		return;
	}

	if (rate_b <= 0.f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_CONTROL_DIRECTION;
		return;
	}

	if (fit > 0.5f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_POOR_FIT;
		return;
	}

	const float damping_derivative = 4.f * inertia[axis_index] * rate_a
						 / (rho * airspeed * sref * length * length);
	const float control_derivative = 2.f * inertia[axis_index] * rate_b
						 / (rho * airspeed * airspeed * sref * length * control_range[axis_index]);
	_identified_damping_derivative[axis_index] = damping_derivative;
	_identified_control_derivative[axis_index] = control_derivative;

	if (!PX4_ISFINITE(damping_derivative) || !PX4_ISFINITE(control_derivative)
	    || damping_derivative >= 0.f || control_derivative <= 0.f) {
		_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_INVALID_DERIVATIVE;
		return;
	}

	_identified_valid[axis_index] = true;
	_identification_reject_reason[axis_index] = fw_gain_design_s::IDENTIFICATION_REJECT_VALID;
	_sd_model_dirty = true;

	if (_param_fw_gd_sd_model.get() != 0) {
		_last_sd_model_save_attempt = hrt_absolute_time();

		if (!save_model_to_sd()) {
			PX4_WARN("could not save identified model to SD card");
		}
	}
}

bool FwGainDesign::flight_test_conditions_met(Axis axis, float &true_airspeed) const
{
	true_airspeed = _airspeed_validated_sub.get().true_airspeed_m_s;
	const vehicle_status_s &vehicle_status = _vehicle_status_sub.get();
	const vehicle_control_mode_s &control_mode = _vehicle_control_mode_sub.get();
	const vehicle_land_detected_s &land_detected = _vehicle_land_detected_sub.get();
	const actuator_armed_s &armed = _actuator_armed_sub.get();

	if (!armed.armed || vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	    || vehicle_status.is_vtol || vehicle_status.in_transition_mode || vehicle_status.failsafe
	    || land_detected.landed || !control_mode.flag_control_rates_enabled
	    || !control_mode.flag_control_attitude_enabled || control_mode.flag_control_termination_enabled) {
		return false;
	}

	if (axis == YAW && _param_fw_yaw_damp_en.get() != 0) {
		return false;
	}

	// Identify all axes from their actual allocated control surfaces. The
	// mapping uses CA surface types and torque signs, not abstract torque
	// requests, so aileron, elevator, and rudder inputs have physical meaning.
	float surface_command = NAN;

	if (!control_surface_input(axis, surface_command)) {
		return false;
	}

	const float vmin = _param_fw_gd_vmin.get();
	const float vmax = _param_fw_gd_vmax.get();
	const airspeed_validated_s &airspeed = _airspeed_validated_sub.get();
	const vehicle_angular_velocity_s &angular_velocity = _vehicle_angular_velocity_sub.get();

	return PX4_ISFINITE(true_airspeed) && PX4_ISFINITE(vmin) && PX4_ISFINITE(vmax)
	       && hrt_elapsed_time(&airspeed.timestamp) < 500_ms
	       && hrt_elapsed_time(&angular_velocity.timestamp) < 100_ms
	       && true_airspeed >= vmin && true_airspeed <= vmax;
}

bool FwGainDesign::request_excitation(Axis axis)
{
	if (_excitation_command.load() != 0) {
		return false;
	}

	_excitation_command.store(static_cast<uint8_t>(axis) + 1U);
	return true;
}

void FwGainDesign::abort_excitation(uint8_t reason)
{
	_excitation_rate_sp = 0.f;
	_excitation_start_requested = false;
	_excitation_abort_reason = reason;
	_excitation_state = fw_gain_design_s::EXCITATION_STATE_ABORTED;
	_calculation_id++;
}

void FwGainDesign::update_flight_test()
{
	_actuator_armed_sub.update();
	_airspeed_validated_sub.update();
	_vehicle_angular_velocity_sub.update();
	_actuator_servos_sub.update();
	_vehicle_status_sub.update();
	_vehicle_control_mode_sub.update();
	_vehicle_land_detected_sub.update();
	_manual_control_setpoint_sub.update();

	const uint8_t command = _excitation_command.load();

	if (command == 4) {
		_excitation_command.store(0);
		abort_excitation(fw_gain_design_s::EXCITATION_ABORT_STOP_REQUESTED);
		return;
	}

	if (command >= 1 && command <= 3) {
		_excitation_command.store(0);

		if (_excitation_state != fw_gain_design_s::EXCITATION_STATE_RUNNING
		    && _excitation_state != fw_gain_design_s::EXCITATION_STATE_WAITING_FOR_CONDITIONS) {
			_excitation_axis = static_cast<Axis>(command - 1);
			_excitation_start_requested = true;
			_excitation_abort_reason = fw_gain_design_s::EXCITATION_ABORT_NONE;
			_excitation_rate_sp = 0.f;
			_excitation_state = fw_gain_design_s::EXCITATION_STATE_WAITING_FOR_CONDITIONS;
		}
	}

	if (_excitation_start_requested) {
		const float amplitude = _param_fw_gd_exc_amp.get();
		const float frequency = _param_fw_gd_exc_freq.get();
		const float duration = _param_fw_gd_exc_time.get();

		if (!PX4_ISFINITE(amplitude) || !PX4_ISFINITE(frequency) || !PX4_ISFINITE(duration)
		    || amplitude <= 0.f || frequency <= 0.f || duration <= kExcitationRampTimeS) {
			abort_excitation(fw_gain_design_s::EXCITATION_ABORT_INVALID_CONFIGURATION);
			return;
		}

		if (_excitation_axis == YAW && _param_fw_yaw_damp_en.get() != 0) {
			abort_excitation(fw_gain_design_s::EXCITATION_ABORT_YAW_DAMPER);
			return;
		}

		_excitation_amplitude = math::radians(amplitude);
		_excitation_frequency = frequency;
		_excitation_duration = duration;
		_excitation_start_requested = false;
		_excitation_state = fw_gain_design_s::EXCITATION_STATE_WAITING_FOR_CONDITIONS;
	}

	if (_excitation_state == fw_gain_design_s::EXCITATION_STATE_WAITING_FOR_CONDITIONS) {
		float true_airspeed = NAN;

		if (!flight_test_conditions_met(_excitation_axis, true_airspeed)) {
			return;
		}

		if (_manual_control_setpoint_sub.get().sticks_moving) {
			return;
		}

		reset_estimator(_excitation_axis);

		_excitation_start = hrt_absolute_time();
		_excitation_state = fw_gain_design_s::EXCITATION_STATE_RUNNING;
		_excitation_abort_reason = fw_gain_design_s::EXCITATION_ABORT_NONE;
		_calculation_id++;
	}

	if (_excitation_state != fw_gain_design_s::EXCITATION_STATE_RUNNING) {
		return;
	}

	if (_manual_control_setpoint_sub.get().sticks_moving) {
		abort_excitation(fw_gain_design_s::EXCITATION_ABORT_PILOT_INPUT);
		return;
	}

	float true_airspeed = NAN;

	if (!flight_test_conditions_met(_excitation_axis, true_airspeed)) {
		abort_excitation(_excitation_axis == YAW && _param_fw_yaw_damp_en.get() != 0
				 ? fw_gain_design_s::EXCITATION_ABORT_YAW_DAMPER
				 : fw_gain_design_s::EXCITATION_ABORT_FLIGHT_CONDITIONS);
		return;
	}

	const vehicle_angular_velocity_s &angular_velocity = _vehicle_angular_velocity_sub.get();
	const uint8_t axis_index = static_cast<uint8_t>(_excitation_axis);
	const float rate_limit = math::radians(_param_fw_gd_exc_rmax.get());

	if (!PX4_ISFINITE(rate_limit) || rate_limit <= 0.f || !PX4_ISFINITE(angular_velocity.xyz[axis_index])
	    || fabsf(angular_velocity.xyz[axis_index]) > rate_limit) {
		abort_excitation(fw_gain_design_s::EXCITATION_ABORT_RATE_LIMIT);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float elapsed = (now - _excitation_start) * 1e-6f;

	if (elapsed >= _excitation_duration) {
		_excitation_rate_sp = 0.f;
		finalize_estimate(_excitation_axis);

		_excitation_state = fw_gain_design_s::EXCITATION_STATE_COMPLETE;
		_calculation_id++;
		return;
	}

	const float ramp = math::constrain(elapsed / kExcitationRampTimeS, 0.f, 1.f);
	const float phase = 2.f * M_PI_F * _excitation_frequency * elapsed;
	_excitation_rate_sp = _excitation_amplitude * ramp * sinf(phase);

	if (_last_estimator_sample != angular_velocity.timestamp_sample && _last_estimator_sample != 0) {
		const float dt = (angular_velocity.timestamp_sample - _last_estimator_sample) * 1e-6f;
		float control_input = NAN;
		const bool input_valid = control_surface_input(_excitation_axis, control_input);

		if (PX4_ISFINITE(dt) && dt >= 0.002f && dt <= 0.1f && input_valid) {
			update_estimator(_excitation_axis, angular_velocity.xyz[axis_index], control_input, true_airspeed, dt, phase, elapsed);
		}
	}

	_last_estimator_sample = angular_velocity.timestamp_sample;
}

void FwGainDesign::fill_flight_test_status(fw_gain_design_s &design) const
{
	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		const Axis model_axis = static_cast<Axis>(axis);
		design.model_source[axis] = model_source_is_estimated(model_axis) ?
				     fw_gain_design_s::MODEL_SOURCE_FLIGHT_TEST : fw_gain_design_s::MODEL_SOURCE_MANUAL;
		design.excitation_rate_sp[axis] = (_excitation_state == fw_gain_design_s::EXCITATION_STATE_RUNNING
						   && axis == static_cast<uint8_t>(_excitation_axis)) ? _excitation_rate_sp : 0.f;
		design.identification_sample_count[axis] = _estimator[axis].sample_count;
		design.identified_airspeed[axis] = _identified_airspeed[axis];
		design.identified_rate_a[axis] = _identified_rate_a[axis];
		design.identified_rate_b[axis] = _identified_rate_b[axis];
		design.identified_damping_derivative[axis] = _identified_damping_derivative[axis];
		design.identified_control_derivative[axis] = _identified_control_derivative[axis];
		design.identification_valid[axis] = _identified_valid[axis];

		design.identification_fit[axis] = _identification_fit[axis];

		design.identification_reject_reason[axis] = _identification_reject_reason[axis];
	}

	design.excitation_state = _excitation_state;
	design.excitation_axis = static_cast<uint8_t>(_excitation_axis);
	design.excitation_abort_reason = _excitation_abort_reason;
	design.excitation_elapsed_s = _excitation_start > 0 && _excitation_state == fw_gain_design_s::EXCITATION_STATE_RUNNING ?
					  (hrt_absolute_time() - _excitation_start) * 1e-6f : 0.f;
}


void FwGainDesign::publish_next_design_point()
{
	fw_gain_design_s design{};
	design.timestamp = hrt_absolute_time();
	design.calculation_id = _calculation_id;
	design.speed_count = _speed_count;
	design.speed_index = _speed_index;
	design.schedule_status = _schedule_status;

	if (_schedule_status == fw_gain_design_s::SCHEDULE_STATUS_VALID && _speed_count > 0) {
		const float vmin = _param_fw_gd_vmin.get();
		const float vmax = _param_fw_gd_vmax.get();
		const float vstep = _param_fw_gd_vstep.get();
		const float true_airspeed = (_speed_index + 1U >= _speed_count) ? vmax : vmin + _speed_index * vstep;
		calculate_design(true_airspeed, design);

		_speed_index++;

		if (_speed_index >= _speed_count) {
			_speed_index = 0;
		}

	} else {
		calculate_design(NAN, design);
	}

	fill_flight_test_status(design);

	_gain_design_pub.publish(design);
}

bool FwGainDesign::apply_enabled(Axis axis) const
{
	switch (axis) {
	case ROLL:
		return _param_fw_gd_apply_r.get() != 0;

	case PITCH:
		return _param_fw_gd_apply_p.get() != 0;

	case YAW:
		// Aileron yaw damping owns this aircraft's yaw response and disables
		// the yaw-rate controller. Never let a selected scheduling switch
		// appear to tune an inactive loop.
		return _param_fw_gd_apply_y.get() != 0 && _param_fw_yaw_damp_en.get() == 0;

	default:
		return false;
	}
}

bool FwGainDesign::capture_axis_baseline(Axis axis)
{
	const uint8_t index = static_cast<uint8_t>(axis);

	if (_axis_overridden[index]) {
		return true;
	}

	if (param_get(_rate_p_param[index], &_baseline_rate_p[index]) != PX4_OK
	    || param_get(_rate_i_param[index], &_baseline_rate_i[index]) != PX4_OK
	    || param_get(_rate_ff_param[index], &_baseline_rate_ff[index]) != PX4_OK) {
		return false;
	}

	if (axis != YAW && param_get(_attitude_tc_param[index], &_baseline_attitude_tc[index]) != PX4_OK) {
		return false;
	}

	_axis_overridden[index] = true;
	return true;
}

bool FwGainDesign::apply_parameter(param_t parameter, float value, bool restore)
{
	if (parameter == PARAM_INVALID || !PX4_ISFINITE(value)) {
		return false;
	}

	float current_value = NAN;

	if (param_get(parameter, &current_value) != PX4_OK) {
		return false;
	}

	const float tolerance = restore ? 1e-6f : fmaxf(1e-4f, 0.005f * fmaxf(fabsf(current_value), fabsf(value)));

	if (PX4_ISFINITE(current_value) && fabsf(current_value - value) <= tolerance) {
		return false;
	}

	return param_set_no_notification(parameter, &value) == PX4_OK;
}

bool FwGainDesign::apply_axis(Axis axis, const fw_gain_design_s &design)
{
	const uint8_t index = static_cast<uint8_t>(axis);

	if (design.axis_status[index] != fw_gain_design_s::AXIS_STATUS_VALID
	    || !capture_axis_baseline(axis)) {
		return false;
	}

	bool changed = false;
	changed |= apply_parameter(_rate_p_param[index], design.px4_rate_p[index], false);
	changed |= apply_parameter(_rate_i_param[index], design.px4_rate_i[index], false);
	changed |= apply_parameter(_rate_ff_param[index], design.px4_rate_ff[index], false);

	if (axis != YAW) {
		changed |= apply_parameter(_attitude_tc_param[index], design.attitude_tc[index], false);
	}

	return changed;
}

bool FwGainDesign::restore_axis(Axis axis)
{
	const uint8_t index = static_cast<uint8_t>(axis);

	if (!_axis_overridden[index]) {
		return false;
	}

	bool changed = false;
	changed |= apply_parameter(_rate_p_param[index], _baseline_rate_p[index], true);
	changed |= apply_parameter(_rate_i_param[index], _baseline_rate_i[index], true);
	changed |= apply_parameter(_rate_ff_param[index], _baseline_rate_ff[index], true);

	if (axis != YAW) {
		changed |= apply_parameter(_attitude_tc_param[index], _baseline_attitude_tc[index], true);
	}

	_axis_overridden[index] = false;
	return changed;
}

void FwGainDesign::restore_inactive_axes(bool armed)
{
	bool changed = false;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		const Axis gain_axis = static_cast<Axis>(axis);

		if (_axis_overridden[axis] && (!armed || !apply_enabled(gain_axis))) {
			changed |= restore_axis(gain_axis);
		}
	}

	if (changed) {
		param_notify_changes();
	}
}

void FwGainDesign::publish_live_gains()
{
	_actuator_armed_sub.update();
	const bool armed = _actuator_armed_sub.get().armed;
	restore_inactive_axes(armed);

	fw_gain_design_gains_s gains{};
	gains.timestamp = hrt_absolute_time();
	gains.airspeed = NAN;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		gains.rate_p[axis] = NAN;
		gains.rate_i[axis] = NAN;
		gains.rate_ff[axis] = NAN;
		gains.attitude_tc[axis] = NAN;
	}

	const bool any_apply_enabled = apply_enabled(ROLL) || apply_enabled(PITCH) || apply_enabled(YAW);

	if (!armed || !any_apply_enabled) {
		_gain_design_gains_pub.publish(gains);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	_airspeed_validated_sub.update();
	const airspeed_validated_s &airspeed = _airspeed_validated_sub.get();
	const bool airspeed_valid = PX4_ISFINITE(airspeed.true_airspeed_m_s)
				     && hrt_elapsed_time(&airspeed.timestamp) < 1_s;
	const float vmin = _param_fw_gd_vmin.get();
	const float vmax = _param_fw_gd_vmax.get();

	if (!airspeed_valid || !PX4_ISFINITE(vmin) || !PX4_ISFINITE(vmax)
	    || airspeed.true_airspeed_m_s < vmin || airspeed.true_airspeed_m_s > vmax) {
		bool changed = false;

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			changed |= restore_axis(static_cast<Axis>(axis));
		}

		if (changed) {
			param_notify_changes();
		}

		_gain_design_gains_pub.publish(gains);
		return;
	}

	fw_gain_design_s design{};
	calculate_design(airspeed.true_airspeed_m_s, design);
	gains.airspeed = airspeed.true_airspeed_m_s;
	bool parameter_changed = false;

	if (_last_gain_parameter_sync == 0) {
		// The fast path below is already active. Delay the first legacy parameter
		// update by one minute so it cannot become a 50 Hz parameter-write path.
		_last_gain_parameter_sync = now;
	}

	const bool parameter_sync_due = now - _last_gain_parameter_sync >= 60_s;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		const Axis gain_axis = static_cast<Axis>(axis);
		const bool attitude_design_valid = gain_axis == YAW || PX4_ISFINITE(design.attitude_tc[axis]);

		if (apply_enabled(gain_axis)) {
			// Keep the known, baseline controller fixed while it is the plant
			// under test. An identified source becomes eligible immediately after
			// a valid completed test, not during its excitation.
			const bool axis_is_under_test = _excitation_state == fw_gain_design_s::EXCITATION_STATE_RUNNING
							   && gain_axis == _excitation_axis;

			if (!axis_is_under_test && design.axis_status[axis] == fw_gain_design_s::AXIS_STATUS_VALID && attitude_design_valid) {
				gains.rate_valid[axis] = true;
				gains.rate_p[axis] = design.px4_rate_p[axis];
				gains.rate_i[axis] = design.px4_rate_i[axis];
				gains.rate_ff[axis] = design.px4_rate_ff[axis];

				if (gain_axis != YAW) {
					gains.attitude_valid[axis] = true;
					gains.attitude_tc[axis] = design.attitude_tc[axis];
				}

				if (parameter_sync_due && capture_axis_baseline(gain_axis)) {
					parameter_changed |= apply_axis(gain_axis, design);
				}

			} else {
				// A direct uORB override is removed immediately if its validity gate
				// fails. This is safety behavior, not periodic synchronization.
				parameter_changed |= restore_axis(gain_axis);
			}
		}
	}

	if (parameter_sync_due) {
		_last_gain_parameter_sync = now;
	}

	if (parameter_changed) {
		param_notify_changes();
	}

	_gain_design_gains_pub.publish(gains);
}

void FwGainDesign::Run()
{
	if (should_exit()) {
		// Consumers fall back immediately instead of waiting for the stale-topic
		// timeout, and any once-per-minute compatibility mirror is put back.
		restore_inactive_axes(false);
		fw_gain_design_gains_s gains{};
		gains.timestamp = hrt_absolute_time();
		gains.airspeed = NAN;

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			gains.rate_p[axis] = NAN;
			gains.rate_i[axis] = NAN;
			gains.rate_ff[axis] = NAN;
			gains.attitude_tc[axis] = NAN;
		}

		_gain_design_gains_pub.publish(gains);
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		update_control_surface_mapping();
		reset_schedule();
	}

	// The SD card may mount after this module starts. Retry quietly while the
	// opt-in persistence setting is enabled, but never overwrite parameters or
	// enable a model source merely because a file was found.
	if (_param_fw_gd_sd_model.get() != 0 && !_sd_model_loaded
	    && (_last_sd_model_load_attempt == 0 || hrt_elapsed_time(&_last_sd_model_load_attempt) > 2_s)) {
		_last_sd_model_load_attempt = hrt_absolute_time();
		load_model_from_sd();
	}

	// Enabling persistence after a completed identification also saves that
	// in-RAM model. Failed SD writes retry slowly, never from the control path.
	if (_param_fw_gd_sd_model.get() != 0 && _sd_model_dirty
	    && (_last_sd_model_save_attempt == 0 || hrt_elapsed_time(&_last_sd_model_save_attempt) > 2_s)) {
		_last_sd_model_save_attempt = hrt_absolute_time();

		if (!save_model_to_sd()) {
			PX4_WARN("could not save identified model to SD card");
		}
	}

	update_flight_test();
	publish_live_gains();
	publish_next_design_point();
}

int FwGainDesign::task_spawn(int argc, char *argv[])
{
	FwGainDesign *instance = new FwGainDesign();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int FwGainDesign::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && strcmp(argv[0], "excite") == 0) {
		FwGainDesign *instance = get_instance();

		if (!instance) {
			return PX4_ERROR;
		}

		if (argc == 2 && strcmp(argv[1], "stop") == 0) {
			instance->_excitation_command.store(4);
			return PX4_OK;
		}

		if (argc >= 2 && strcmp(argv[1], "start") == 0) {
			int32_t selected_axis = instance->_param_fw_gd_exc_axis.get();

			if (argc == 3) {
				if (strcmp(argv[2], "roll") == 0) {
					selected_axis = 1;

				} else if (strcmp(argv[2], "pitch") == 0) {
					selected_axis = 2;

				} else if (strcmp(argv[2], "yaw") == 0) {
					selected_axis = 3;

				} else {
					return print_usage("axis must be roll, pitch, or yaw");
				}
			}

			if (selected_axis < 1 || selected_axis > 3) {
				return print_usage("set FW_GD_EXC_AXIS or specify roll, pitch, or yaw");
			}

			if (!instance->request_excitation(static_cast<Axis>(selected_axis - 1))) {
				PX4_WARN("excitation already active");
				return PX4_ERROR;
			}

			PX4_INFO("waiting to excite %s", selected_axis == 1 ? "roll" : selected_axis == 2 ? "pitch" : "yaw");
			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int FwGainDesign::print_status()
{
	PX4_INFO("calculation id: %u, point %u/%u", (unsigned)_calculation_id,
			 (unsigned)(_speed_index + 1U), (unsigned)_speed_count);
	PX4_INFO("live apply: roll %s, pitch %s, yaw %s", apply_enabled(ROLL) ? "enabled" : "disabled",
		 apply_enabled(PITCH) ? "enabled" : "disabled", apply_enabled(YAW) ? "enabled" : "disabled");
	PX4_INFO("surface ID inputs: roll %s, pitch %s, yaw %s", _control_surface_available[ROLL] ? "available" : "missing",
		 _control_surface_available[PITCH] ? "available" : "missing", _control_surface_available[YAW] ? "available" : "missing");
	PX4_INFO("SD model: %s, loaded %s, saved %s", _param_fw_gd_sd_model.get() != 0 ? "enabled" : "disabled",
		 _sd_model_loaded ? "yes" : "no", _sd_model_saved ? "yes" : "no");
	PX4_INFO("excitation: state %u axis %u samples %u", (unsigned)_excitation_state,
		 (unsigned)_excitation_axis, (unsigned)_estimator[static_cast<uint8_t>(_excitation_axis)].sample_count);
	return PX4_OK;
}

int FwGainDesign::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Calculates a speed-indexed fixed-wing rate and attitude gain design from a
decoupled aerodynamic model. It always publishes `fw_gain_design`. The
optional per-axis live-apply switches use fresh true airspeed only while
armed, and restore the prior parameters when disarmed or disabled.

`excite start [roll|pitch|yaw]` begins one explicitly requested, bounded
flight test after fixed-wing, armed, airborne, stabilized conditions are met.
`excite stop` removes the excitation immediately. The test is aborted by any
pilot stick movement, rate-limit breach, loss of conditions, or enabled
aileron yaw damper on the yaw axis.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_gain_design", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("excite start", "Start the FW_GD_EXC_AXIS flight test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("excite start roll|pitch|yaw", "Start one selected flight test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("excite stop", "Abort the active flight test");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_gain_design_main(int argc, char *argv[])
{
	return FwGainDesign::main(argc, argv);
}
