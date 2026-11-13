/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "FwTrimEstimator.hpp"

#include <cmath>
#include <cstdio>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/Quaternion.hpp>
#include <matrix/matrix/Euler.hpp>
#include <px4_platform_common/defines.h>

using matrix::Eulerf;
using matrix::Quatf;
using namespace time_literals;

FwTrimEstimator::FwTrimEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_trim_parameter[ROLL] = param_find("TRIM_ROLL");
	_trim_parameter[PITCH] = param_find("TRIM_PITCH");
	updateParams();
	_estimate[ROLL] = _param_trim_roll.get();
	_estimate[PITCH] = _param_trim_pitch.get();
	update_pwm_mapping();
	refresh_pwm_baselines();
}

bool FwTrimEstimator::init()
{
	ScheduleOnInterval(200_ms);
	return true;
}

int FwTrimEstimator::task_spawn(int argc, char *argv[])
{
	FwTrimEstimator *instance = new FwTrimEstimator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int FwTrimEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

float FwTrimEstimator::scheduled_trim_increment(Axis axis, float airspeed) const
{
	const float airspeed_trim = _param_fw_airspd_trim.get();
	const float airspeed_min = _param_fw_airspd_min.get();
	const float airspeed_max = _param_fw_airspd_max.get();

	if (!PX4_ISFINITE(airspeed) || !PX4_ISFINITE(airspeed_trim)) {
		return 0.f;
	}

	if (airspeed < airspeed_trim && airspeed_min < airspeed_trim) {
		const float increment = axis == ROLL ? _param_fw_dtrim_r_vmin.get() : _param_fw_dtrim_p_vmin.get();
		return math::interpolate(airspeed, airspeed_min, airspeed_trim, increment, 0.f);
	}

	if (airspeed_max > airspeed_trim) {
		const float increment = axis == ROLL ? _param_fw_dtrim_r_vmax.get() : _param_fw_dtrim_p_vmax.get();
		return math::interpolate(airspeed, airspeed_trim, airspeed_max, 0.f, increment);
	}

	return 0.f;
}

bool FwTrimEstimator::common_conditions_met(float &airspeed, float &roll, float &pitch, uint8_t &reason) const
{
	using Status = fw_trim_status_s;
	reason = Status::REJECT_NONE;
	airspeed = NAN;
	roll = NAN;
	pitch = NAN;

	if (!_param_fw_trim_est_en.get()) {
		reason = Status::REJECT_DISABLED;
		return false;
	}

	if (_vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING || _vehicle_status.in_transition_mode) {
		reason = Status::REJECT_NOT_FIXED_WING;
		return false;
	}

	if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		reason = Status::REJECT_NOT_ARMED;
		return false;
	}

	if (_land_detected.landed || _land_detected.ground_contact) {
		reason = Status::REJECT_LANDED;
		return false;
	}

	if (!_control_mode.flag_control_rates_enabled || !_control_mode.flag_control_attitude_enabled || _vehicle_status.failsafe) {
		reason = Status::REJECT_CONTROL_MODE;
		return false;
	}

	if (hrt_elapsed_time(&_attitude.timestamp) > 300_ms || hrt_elapsed_time(&_angular_velocity.timestamp) > 300_ms
	    || hrt_elapsed_time(&_rates_sp.timestamp) > 300_ms || hrt_elapsed_time(&_torque_sp.timestamp) > 300_ms
	    || hrt_elapsed_time(&_local_position.timestamp) > 500_ms || hrt_elapsed_time(&_airspeed.timestamp) > 500_ms) {
		reason = Status::REJECT_STALE_DATA;
		return false;
	}

	airspeed = _airspeed.calibrated_airspeed_m_s;

	if (!PX4_ISFINITE(airspeed) || airspeed < _param_fw_trim_aspd_min.get()) {
		reason = Status::REJECT_AIRSPEED;
		return false;
	}

	if (!_local_position.v_z_valid || !PX4_ISFINITE(_local_position.vz)
	    || fabsf(_local_position.vz) > _param_fw_trim_vzmax.get()) {
		reason = Status::REJECT_VERTICAL_MOTION;
		return false;
	}

	if (fabsf(_angular_velocity.xyz[0]) > math::radians(_param_fw_trim_rate.get())
	    || fabsf(_angular_velocity.xyz[1]) > math::radians(_param_fw_trim_rate.get())
	    || fabsf(_rates_sp.roll - _angular_velocity.xyz[0]) > math::radians(_param_fw_trim_err.get())
	    || fabsf(_rates_sp.pitch - _angular_velocity.xyz[1]) > math::radians(_param_fw_trim_err.get())) {
		reason = Status::REJECT_RATE_MOTION;
		return false;
	}

	if (_control_mode.flag_control_manual_enabled && _manual_control.timestamp != 0
	    && hrt_elapsed_time(&_manual_control.timestamp) < 500_ms
	    && (_manual_control.sticks_moving || fabsf(_manual_control.roll) > _param_fw_trim_stick.get()
		|| fabsf(_manual_control.pitch) > _param_fw_trim_stick.get())) {
		reason = Status::REJECT_PILOT_INPUT;
		return false;
	}

	const Quatf q(_attitude.q);
	const Eulerf euler(q);
	roll = euler.phi();
	pitch = euler.theta();

	if (!PX4_ISFINITE(roll) || !PX4_ISFINITE(pitch)) {
		reason = Status::REJECT_ATTITUDE;
		return false;
	}

	return true;
}

bool FwTrimEstimator::axis_conditions_met(Axis axis, float roll, float pitch, uint8_t common_reason, uint8_t &reason) const
{
	using Status = fw_trim_status_s;
	reason = common_reason;

	if (common_reason != Status::REJECT_NONE) {
		return false;
	}

	const float roll_limit = math::radians(_param_fw_trim_rmax.get());
	const float pitch_limit = math::radians(_param_fw_trim_pmax.get());

	if (axis == ROLL && _param_fw_trim_loit.get()
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
		if (fabsf(roll) > math::radians(_param_fw_trim_loit_bk.get()) || fabsf(pitch) > pitch_limit) {
			reason = Status::REJECT_ATTITUDE;
			return false;
		}

		return true;
	}

	if (fabsf(roll) > roll_limit || fabsf(pitch) > pitch_limit) {
		reason = Status::REJECT_ATTITUDE;
		return false;
	}

	return true;
}

void FwTrimEstimator::update_estimate(Axis axis, float candidate, float dt)
{
	if (!PX4_ISFINITE(candidate) || fabsf(candidate) > _param_fw_trim_lim.get()) {
		return;
	}

	if (!PX4_ISFINITE(_estimate[axis])) {
		_estimate[axis] = candidate;
	} else {
		const float time_constant = math::max(_param_fw_trim_tau.get(), 1.f);
		const float alpha = math::constrain(dt / time_constant, 0.f, 0.2f);
		_estimate[axis] += alpha * (candidate - _estimate[axis]);
	}

	_sample_count[axis]++;
}

void FwTrimEstimator::apply_estimate(Axis axis, hrt_abstime now)
{
	const bool apply_enabled = axis == ROLL ? _param_fw_trim_r_apply.get() : _param_fw_trim_p_apply.get();

	if (!apply_enabled || _trim_parameter[axis] == PARAM_INVALID || !PX4_ISFINITE(_estimate[axis])
	    || (_last_apply[axis] != 0 && hrt_elapsed_time(&_last_apply[axis]) < static_cast<hrt_abstime>(_param_fw_trim_int.get() * 1e6f))) {
		return;
	}

	const float current = _active_trim[axis];

	if (!PX4_ISFINITE(current)) {
		return;
	}

	const float max_step = math::max(_param_fw_trim_step.get(), 0.f);
	const float target = math::constrain(_estimate[axis], -_param_fw_trim_lim.get(), _param_fw_trim_lim.get());
	const float updated = math::constrain(current + math::constrain(target - current, -max_step, max_step),
						-_param_fw_trim_lim.get(), _param_fw_trim_lim.get());

	if (fabsf(updated - current) >= 1e-4f && param_set(_trim_parameter[axis], &updated) == PX4_OK) {
		_active_trim[axis] = updated;
		_update_count[axis]++;
	}

	_last_apply[axis] = now;
}

bool FwTrimEstimator::pwm_expansion_enabled(Axis axis) const
{
	return _param_fw_trim_pwm_en.get()
	       && (axis == ROLL ? _param_fw_trim_r_pwm_en.get() : _param_fw_trim_p_pwm_en.get());
}

float FwTrimEstimator::pwm_trim_threshold(Axis axis) const
{
	return axis == ROLL ? _param_fw_trim_r_pwm_thr.get() : _param_fw_trim_p_pwm_thr.get();
}

void FwTrimEstimator::update_pwm_mapping()
{
	for (PwmOutput &output : _pwm_outputs) {
		output.axis_mask = 0;
	}

	uint8_t surface_axis_mask[actuator_servos_s::NUM_CONTROLS] {};
	const int32_t surface_count = math::constrain(_param_ca_sv_cs_count.get(), int32_t{0},
							  static_cast<int32_t>(actuator_servos_s::NUM_CONTROLS));

	for (int32_t surface = 0; surface < surface_count; ++surface) {
		char parameter_name[20] {};
		snprintf(parameter_name, sizeof(parameter_name), "CA_SV_CS%ld_TYPE", static_cast<long>(surface));
		const param_t type_parameter = param_find(parameter_name);
		int32_t type = 0;

		if (type_parameter == PARAM_INVALID || param_get(type_parameter, &type) != PX4_OK) {
			continue;
		}

		// These are conventional ailerons and elevators. Elevons are intentionally
		// excluded: expanding their PWM endpoints for a pitch offset also alters
		// their roll authority and needs a separate mixed-surface design.
		if (type == 1 || type == 2 || type == 15) {
			surface_axis_mask[surface] |= 1u << ROLL;

		} else if (type == 3) {
			surface_axis_mask[surface] |= 1u << PITCH;
		}
	}

	for (uint8_t output_index = 0; output_index < MAX_PWM_OUTPUTS; ++output_index) {
		PwmOutput &output = _pwm_outputs[output_index];
		const bool main_output = output_index < 8;
		const uint8_t channel = (output_index % 8) + 1;
		const char *prefix = main_output ? "PWM_MAIN" : "PWM_AUX";
		char parameter_name[20] {};

		snprintf(parameter_name, sizeof(parameter_name), "%s_FUNC%u", prefix, channel);
		output.function = param_find(parameter_name);
		snprintf(parameter_name, sizeof(parameter_name), "%s_MIN%u", prefix, channel);
		output.minimum = param_find(parameter_name);
		snprintf(parameter_name, sizeof(parameter_name), "%s_MAX%u", prefix, channel);
		output.maximum = param_find(parameter_name);

		int32_t function = 0;
		if (output.function == PARAM_INVALID || param_get(output.function, &function) != PX4_OK) {
			continue;
		}

		const int32_t surface = function - OUTPUT_FUNCTION_SERVO1;
		if (surface >= 0 && surface < actuator_servos_s::NUM_CONTROLS) {
			output.axis_mask = surface_axis_mask[surface];
		}
	}

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		_pwm_channel_mask[axis] = 0;
	}

	for (uint8_t output_index = 0; output_index < MAX_PWM_OUTPUTS; ++output_index) {
		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			if ((_pwm_outputs[output_index].axis_mask & (1u << axis)) != 0) {
				_pwm_channel_mask[axis] |= 1u << output_index;
			}
		}
	}
}

void FwTrimEstimator::refresh_pwm_baselines()
{
	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		_pwm_expansion_us[axis] = 0;
	}

	for (PwmOutput &output : _pwm_outputs) {
		if (output.axis_mask == 0 || output.minimum == PARAM_INVALID || output.maximum == PARAM_INVALID) {
			continue;
		}

		int32_t minimum = 0;
		int32_t maximum = 0;

		if (param_get(output.minimum, &minimum) != PX4_OK || param_get(output.maximum, &maximum) != PX4_OK
		    || minimum >= maximum) {
			output.baseline_valid = false;
			continue;
		}

		const bool matches_last_module_value = output.baseline_valid
							       && minimum == output.baseline_min - output.expansion_us
							       && maximum == output.baseline_max + output.expansion_us;

		if (!output.baseline_valid || !matches_last_module_value) {
			// Treat a manual PWM edit as a new baseline. This prevents the module
			// from overwriting an in-flight adjustment made by the operator.
			output.baseline_min = minimum;
			output.baseline_max = maximum;
			output.expansion_us = 0;
			output.baseline_valid = true;
		}

		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			if ((output.axis_mask & (1u << axis)) != 0) {
				_pwm_expansion_us[axis] = math::max(_pwm_expansion_us[axis], output.expansion_us);
			}
		}
	}
}

void FwTrimEstimator::apply_pwm_expansion(Axis axis, hrt_abstime now)
{
	const bool trim_apply_enabled = axis == ROLL ? _param_fw_trim_r_apply.get() : _param_fw_trim_p_apply.get();

	if (!trim_apply_enabled || !pwm_expansion_enabled(axis) || !PX4_ISFINITE(_estimate[axis])
	    || fabsf(_estimate[axis]) < pwm_trim_threshold(axis)
	    || _axis_stable_since[axis] == 0
	    || hrt_elapsed_time(&_axis_stable_since[axis]) < static_cast<hrt_abstime>(_param_fw_trim_time.get() * 1e6f)
	    || (_last_pwm_apply[axis] != 0
		&& hrt_elapsed_time(&_last_pwm_apply[axis]) < static_cast<hrt_abstime>(_param_fw_trim_pwm_int.get() * 1e6f))) {
		return;
	}

	const int32_t maximum_expansion = math::max(_param_fw_trim_pwm_max.get(), int32_t{0});
	const int32_t step = math::max(_param_fw_trim_pwm_step.get(), int32_t{0});
	int32_t largest_expansion = 0;

	if (step == 0 || maximum_expansion == 0) {
		return;
	}

	for (uint8_t output_index = 0; output_index < MAX_PWM_OUTPUTS; ++output_index) {
		PwmOutput &output = _pwm_outputs[output_index];

		if ((output.axis_mask & (1u << axis)) == 0 || !output.baseline_valid) {
			continue;
		}

		const int32_t expansion = math::min(output.expansion_us + step, maximum_expansion);
		// PWM output parameter metadata restricts min and max to these physical
		// bounds. Keeping the same hard bounds here also prevents a bad parameter
		// combination from expanding beyond a supported PWM signal.
		const int32_t new_minimum = math::max(output.baseline_min - expansion, int32_t{800});
		const int32_t new_maximum = math::min(output.baseline_max + expansion, int32_t{2200});
		const int32_t actual_expansion = math::min(output.baseline_min - new_minimum,
									   new_maximum - output.baseline_max);

		if (actual_expansion > output.expansion_us) {
			if (param_set(output.minimum, &new_minimum) == PX4_OK && param_set(output.maximum, &new_maximum) == PX4_OK) {
				output.expansion_us = actual_expansion;
			}
		}

		largest_expansion = math::max(largest_expansion, output.expansion_us);
	}

	_pwm_expansion_us[axis] = largest_expansion;
	_last_pwm_apply[axis] = now;
}

void FwTrimEstimator::publish_status(float airspeed, float roll, float pitch, uint8_t common_reason)
{
	fw_trim_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.airspeed = airspeed;
	status.roll = roll;
	status.pitch = pitch;
	status.vertical_speed = _local_position.vz;
	status.common_reject_reason = common_reason;

	for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
		status.axis_conditions_met[axis] = _axis_conditions[axis];
		status.estimate_valid[axis] = _axis_stable_since[axis] != 0
						      && hrt_elapsed_time(&_axis_stable_since[axis]) >= static_cast<hrt_abstime>(_param_fw_trim_time.get() * 1e6f);
		status.apply_enabled[axis] = axis == ROLL ? _param_fw_trim_r_apply.get() : _param_fw_trim_p_apply.get();
		status.candidate_trim[axis] = _candidate[axis];
		status.estimated_trim[axis] = _estimate[axis];
		status.active_trim[axis] = _active_trim[axis];
		status.stable_time_s[axis] = _axis_stable_since[axis] == 0 ? 0.f : hrt_elapsed_time(&_axis_stable_since[axis]) * 1e-6f;
		status.sample_count[axis] = _sample_count[axis];
		status.update_count[axis] = _update_count[axis];
		status.reject_reason[axis] = _axis_reason[axis];
		status.pwm_expansion_enabled[axis] = pwm_expansion_enabled(static_cast<Axis>(axis));
		status.pwm_expansion_active[axis] = _pwm_expansion_us[axis] > 0;
		status.pwm_channel_mask[axis] = _pwm_channel_mask[axis];
		status.pwm_expansion_us[axis] = _pwm_expansion_us[axis];
	}

	_status_pub.publish(status);
}

void FwTrimEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();

		const float updated_trim[AXIS_COUNT] {_param_trim_roll.get(), _param_trim_pitch.get()};

		// Preserve an explicit external trim adjustment. Parameter writes made by
		// this module have already updated _active_trim in apply_estimate().
		for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
			if (PX4_ISFINITE(_active_trim[axis]) && fabsf(updated_trim[axis] - _active_trim[axis]) > 1e-5f) {
				_estimate[axis] = updated_trim[axis];
				_sample_count[axis] = 0;
				_axis_stable_since[axis] = 0;
			}
		}

		update_pwm_mapping();
		refresh_pwm_baselines();
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_control_mode_sub.update(&_control_mode);
	_land_detected_sub.update(&_land_detected);
	_attitude_sub.update(&_attitude);
	_angular_velocity_sub.update(&_angular_velocity);
	_rates_sp_sub.update(&_rates_sp);
	_torque_sp_sub.update(&_torque_sp);
	_local_position_sub.update(&_local_position);
	_airspeed_sub.update(&_airspeed);
	_manual_control_sub.update(&_manual_control);

	_active_trim[ROLL] = _param_trim_roll.get();
	_active_trim[PITCH] = _param_trim_pitch.get();

	const hrt_abstime now = hrt_absolute_time();
	const float dt = _last_run == 0 ? 0.2f : math::constrain((now - _last_run) * 1e-6f, 0.02f, 1.f);
	_last_run = now;

	float airspeed = NAN;
	float roll = NAN;
	float pitch = NAN;
	uint8_t common_reason = fw_trim_status_s::REJECT_NONE;
	common_conditions_met(airspeed, roll, pitch, common_reason);

	for (uint8_t index = 0; index < AXIS_COUNT; ++index) {
		const Axis axis = static_cast<Axis>(index);
		uint8_t reason = common_reason;
		_axis_conditions[index] = axis_conditions_met(axis, roll, pitch, common_reason, reason);
		_axis_reason[index] = reason;
		_candidate[index] = NAN;

		if (_axis_conditions[index]) {
			_candidate[index] = _torque_sp.xyz[index] - scheduled_trim_increment(axis, airspeed);

			if (PX4_ISFINITE(_candidate[index]) && fabsf(_candidate[index]) <= _param_fw_trim_lim.get()) {
				if (_axis_stable_since[index] == 0) {
					_axis_stable_since[index] = now;
				}

				update_estimate(axis, _candidate[index], dt);

				if (hrt_elapsed_time(&_axis_stable_since[index]) >= static_cast<hrt_abstime>(_param_fw_trim_time.get() * 1e6f)) {
					apply_estimate(axis, now);
					apply_pwm_expansion(axis, now);
				}

			} else {
				_axis_conditions[index] = false;
				_axis_reason[index] = fw_trim_status_s::REJECT_OUTPUT_LIMIT;
				_axis_stable_since[index] = 0;
			}

		} else {
			_axis_stable_since[index] = 0;
		}
	}

	publish_status(airspeed, roll, pitch, common_reason);
}

int FwTrimEstimator::print_status()
{
	PX4_INFO("roll: estimate %.4f active %.4f samples %lu updates %lu", (double)_estimate[ROLL],
		 (double)_active_trim[ROLL], (unsigned long)_sample_count[ROLL], (unsigned long)_update_count[ROLL]);
	PX4_INFO("pitch: estimate %.4f active %.4f samples %lu updates %lu", (double)_estimate[PITCH],
		 (double)_active_trim[PITCH], (unsigned long)_sample_count[PITCH], (unsigned long)_update_count[PITCH]);
	PX4_INFO("PWM expansion: roll %ld us, pitch %ld us", (long)_pwm_expansion_us[ROLL],
		 (long)_pwm_expansion_us[PITCH]);
	return PX4_OK;
}

int FwTrimEstimator::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Learns fixed-wing roll and pitch trim from stable, nearly level flight.
Automatic parameter writes are disabled by default.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_trim_estimator", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start trim estimator");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fw_trim_estimator_main(int argc, char *argv[])
{
	return FwTrimEstimator::main(argc, argv);
}
