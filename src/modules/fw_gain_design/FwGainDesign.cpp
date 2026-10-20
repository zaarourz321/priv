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
	updateParams();
	reset_schedule();
}

bool FwGainDesign::init()
{
	// The module has no flight-control subscriptions or publications. Running
	// periodically makes all parameter edits live without touching the values.
	ScheduleOnInterval(100_ms);
	return true;
}

uint8_t FwGainDesign::validate_common_inputs() const
{
	const float values[] = {
		_param_fw_gd_rho.get(), _param_fw_gd_sref.get(), _param_fw_gd_bref.get(), _param_fw_gd_cref.get(),
		_param_fw_gd_ix.get(), _param_fw_gd_iy.get(), _param_fw_gd_iz.get(),
		_param_fw_gd_r_max.get(), _param_fw_gd_p_max.get(), _param_fw_gd_y_max.get()
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

void FwGainDesign::publish_next_design_point()
{
	fw_gain_design_s design{};
	design.timestamp = hrt_absolute_time();
	design.calculation_id = _calculation_id;
	design.speed_count = _speed_count;
	design.speed_index = _speed_index;
	design.schedule_status = _schedule_status;
	design.airspeed = NAN;
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

	if (_schedule_status != fw_gain_design_s::SCHEDULE_STATUS_VALID || _speed_count == 0) {
		_gain_design_pub.publish(design);
		return;
	}

	const float vmin = _param_fw_gd_vmin.get();
	const float vmax = _param_fw_gd_vmax.get();
	const float vstep = _param_fw_gd_vstep.get();
	design.airspeed = (_speed_index + 1U >= _speed_count) ? vmax : vmin + _speed_index * vstep;

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

	_gain_design_pub.publish(design);

	_speed_index++;

	if (_speed_index >= _speed_count) {
		_speed_index = 0;
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
	PX4_INFO("advisory only: does not write FW_* parameters");
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
decoupled aerodynamic model. It only publishes `fw_gain_design`; it never
changes flight-controller gains or sends a control command.
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
