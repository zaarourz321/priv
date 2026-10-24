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

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

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
	// apply switches is enabled.
	ScheduleOnInterval(100_ms);
	return true;
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

		const float damping_coefficient[AXIS_COUNT] = {
			_param_fw_gd_clp.get(), _param_fw_gd_cmq.get(), _param_fw_gd_cnr.get()
		};
		const float control_coefficient[AXIS_COUNT] = {
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

		// The aerodynamic model uses true airspeed and local density. PX4's
		// airspeed scaler uses calibrated/equivalent airspeed, referenced to the
		// ISA sea-level density. Publish that conversion so the two gain forms
		// can be interpreted without an implicit sea-level assumption.
		design.px4_airspeed = design.airspeed * sqrtf(rho / 1.225f);

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			const float length = reference_length[axis];

			if (!PX4_ISFINITE(damping_coefficient[axis]) || !PX4_ISFINITE(control_coefficient[axis])
			    || !PX4_ISFINITE(bandwidth_ratio[axis]) || !PX4_ISFINITE(damping_ratio[axis])) {
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

			// The design is intentionally restricted to a stable, decoupled rate
			// mode. A coupled Dutch-roll or short-period model must first be reduced
			// to a valid SISO equivalent before these gains are meaningful.
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
		return _param_fw_gd_apply_y.get() != 0;

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

void FwGainDesign::apply_live_gains()
{
	_actuator_armed_sub.update();
	const bool armed = _actuator_armed_sub.get().armed;
	restore_inactive_axes(armed);

	const bool any_apply_enabled = apply_enabled(ROLL) || apply_enabled(PITCH) || apply_enabled(YAW);

	if (!armed || !any_apply_enabled) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_last_gain_apply != 0 && now - _last_gain_apply < 500_ms) {
		return;
	}

	_last_gain_apply = now;
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

		return;
	}

	fw_gain_design_s design{};
	calculate_design(airspeed.true_airspeed_m_s, design);
	bool changed = false;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		const Axis gain_axis = static_cast<Axis>(axis);
		const bool attitude_design_valid = gain_axis == YAW || PX4_ISFINITE(design.attitude_tc[axis]);

		if (apply_enabled(gain_axis)) {
			if (design.axis_status[axis] == fw_gain_design_s::AXIS_STATUS_VALID && attitude_design_valid) {
				changed |= apply_axis(gain_axis, design);

			} else {
				changed |= restore_axis(gain_axis);
			}
		}
	}

	if (changed) {
		param_notify_changes();
	}
}

void FwGainDesign::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		reset_schedule();
	}

	apply_live_gains();
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
	return print_usage("unknown command");
}

int FwGainDesign::print_status()
{
	PX4_INFO("calculation id: %u, point %u/%u", (unsigned)_calculation_id,
			 (unsigned)(_speed_index + 1U), (unsigned)_speed_count);
	PX4_INFO("live apply: roll %s, pitch %s, yaw %s", apply_enabled(ROLL) ? "enabled" : "disabled",
		 apply_enabled(PITCH) ? "enabled" : "disabled", apply_enabled(YAW) ? "enabled" : "disabled");
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
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_gain_design", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_gain_design_main(int argc, char *argv[])
{
	return FwGainDesign::main(argc, argv);
}
