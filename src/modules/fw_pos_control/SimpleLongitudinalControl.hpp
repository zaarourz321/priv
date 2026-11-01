/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef SIMPLE_LONGITUDINAL_CONTROL_HPP
#define SIMPLE_LONGITUDINAL_CONTROL_HPP

#include <float.h>
#include <math.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

class SimpleLongitudinalControl
{
public:
	struct Parameters {
		float altitude_p;
		float height_rate_p;
		float height_rate_i;
		float height_rate_i_limit;
		float height_rate_ff;
		float airspeed_p;
		float airspeed_i;
		float airspeed_i_limit;
		float speed_error_deadband;
		float speed_pitch_p;
		float pitch_slew_rate;
		float throttle_slew_rate;
	};

	struct Input {
		float dt;
		float altitude;
		float altitude_setpoint;
		float height_rate;
		float height_rate_setpoint;
		float airspeed;
		float airspeed_setpoint;
		float airspeed_min;
		float pitch_min;
		float pitch_max;
		float throttle_min;
		float throttle_max;
		float throttle_trim;
		float max_sink_rate;
		float max_climb_rate;
		float pitch_offset;
	};

	struct Output {
		float pitch_setpoint;
		float throttle_setpoint;
		float height_rate_setpoint;
		float speed_error;
		float altitude_error;
	};

	void reset()
	{
		_initialized = false;
		_airspeed_integrator = 0.f;
		_height_rate_integrator = 0.f;
		_last_pitch = NAN;
		_last_throttle = NAN;
	}

	Output update(const Input &input, const Parameters &params)
	{
		const float dt = math::constrain(input.dt, 0.002f, 0.1f);
		Output output{};
		output.altitude_error = input.altitude_setpoint - input.altitude;
		output.speed_error = input.airspeed_setpoint - input.airspeed;

		float height_rate_setpoint = PX4_ISFINITE(input.height_rate_setpoint) ? input.height_rate_setpoint :
						    params.altitude_p * output.altitude_error;
		height_rate_setpoint = math::constrain(height_rate_setpoint, -input.max_sink_rate, input.max_climb_rate);

		// Airspeed has priority over altitude. If the aircraft is below the
		// requested speed, stop demanding climb and lower pitch in proportion to
		// the speed deficit. Throttle PI remains active at the same time.
		const float speed_deficit = math::max(output.speed_error - params.speed_error_deadband, 0.f);

		if (speed_deficit > FLT_EPSILON) {
			height_rate_setpoint = math::min(height_rate_setpoint, 0.f);
		}

		output.height_rate_setpoint = height_rate_setpoint;
		const float height_rate_error = height_rate_setpoint - input.height_rate;
		const float airspeed_for_ff = math::max(input.airspeed, input.airspeed_min);
		const float flight_path_angle = asinf(math::constrain(height_rate_setpoint / airspeed_for_ff, -0.7f, 0.7f));
		const float pitch_ff = params.height_rate_ff * flight_path_angle;
		const float speed_pitch_correction = params.speed_pitch_p * speed_deficit;

		if (!_initialized) {
			_airspeed_integrator = math::constrain(_last_throttle - input.throttle_trim - params.airspeed_p * output.speed_error,
							       -params.airspeed_i_limit, params.airspeed_i_limit);
			_height_rate_integrator = math::constrain(_last_pitch - input.pitch_offset - pitch_ff
									       - params.height_rate_p * height_rate_error + speed_pitch_correction,
									       -params.height_rate_i_limit, params.height_rate_i_limit);
			_initialized = true;
		}

		const float throttle_without_integrator = input.throttle_trim + params.airspeed_p * output.speed_error;
		const float airspeed_integrator_delta = params.airspeed_i * output.speed_error * dt;
		integrate_if_not_winding_up(_airspeed_integrator, airspeed_integrator_delta, throttle_without_integrator,
						    input.throttle_min, input.throttle_max, params.airspeed_i_limit);

		const float pitch_without_integrator = input.pitch_offset + pitch_ff + params.height_rate_p * height_rate_error
							      - speed_pitch_correction;
		const float height_rate_integrator_delta = params.height_rate_i * height_rate_error * dt;
		integrate_if_not_winding_up(_height_rate_integrator, height_rate_integrator_delta, pitch_without_integrator,
						    input.pitch_min, input.pitch_max, params.height_rate_i_limit);

		float pitch_setpoint = math::constrain(pitch_without_integrator + _height_rate_integrator,
						       input.pitch_min, input.pitch_max);
		float throttle_setpoint = math::constrain(throttle_without_integrator + _airspeed_integrator,
							 input.throttle_min, input.throttle_max);

		if (params.pitch_slew_rate > FLT_EPSILON && PX4_ISFINITE(_last_pitch)) {
			pitch_setpoint = math::constrain(pitch_setpoint, _last_pitch - params.pitch_slew_rate * dt,
							 _last_pitch + params.pitch_slew_rate * dt);
		}

		if (params.throttle_slew_rate > FLT_EPSILON && PX4_ISFINITE(_last_throttle)) {
			throttle_setpoint = math::constrain(throttle_setpoint, _last_throttle - params.throttle_slew_rate * dt,
							    _last_throttle + params.throttle_slew_rate * dt);
		}

		_last_pitch = pitch_setpoint;
		_last_throttle = throttle_setpoint;
		output.pitch_setpoint = pitch_setpoint;
		output.throttle_setpoint = throttle_setpoint;
		return output;
	}

	void initialize(float pitch, float throttle)
	{
		_last_pitch = PX4_ISFINITE(pitch) ? pitch : 0.f;
		_last_throttle = PX4_ISFINITE(throttle) ? throttle : 0.f;
		_initialized = false;
	}

private:
	static void integrate_if_not_winding_up(float &integrator, const float delta, const float output_without_integrator,
			const float output_min, const float output_max, const float integrator_limit)
	{
		if (fabsf(delta) <= FLT_EPSILON || integrator_limit <= FLT_EPSILON) {
			return;
		}

		const float unconstrained_output = output_without_integrator + integrator;

		if ((unconstrained_output > output_min && unconstrained_output < output_max)
		    || (unconstrained_output <= output_min && delta > 0.f)
		    || (unconstrained_output >= output_max && delta < 0.f)) {
			integrator = math::constrain(integrator + delta, -integrator_limit, integrator_limit);
		}
	}

	bool _initialized{false};
	float _airspeed_integrator{0.f};
	float _height_rate_integrator{0.f};
	float _last_pitch{NAN};
	float _last_throttle{NAN};
};

#endif // SIMPLE_LONGITUDINAL_CONTROL_HPP
