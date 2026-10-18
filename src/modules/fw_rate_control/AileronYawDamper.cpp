/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.
 *
 ****************************************************************************/

#include "AileronYawDamper.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

void AileronYawDamper::setParameters(float gain, float cutoff_hz, float max_output, int direction)
{
	_gain = PX4_ISFINITE(gain) ? gain : 0.f;
	_cutoff_hz = PX4_ISFINITE(cutoff_hz) ? math::max(cutoff_hz, 0.f) : 0.f;
	_max_output = PX4_ISFINITE(max_output) ? math::constrain(fabsf(max_output), 0.f, 1.f) : 0.f;
	_direction = direction < 0 ? -1.f : 1.f;
}

float AileronYawDamper::update(float yaw_rate_error, float dt)
{
	if (!PX4_ISFINITE(yaw_rate_error)) {
		return 0.f;
	}

	if (!_filter_initialized) {
		_filtered_yaw_rate_error = yaw_rate_error;
		_filter_initialized = true;

	} else if (_cutoff_hz > 0.f && PX4_ISFINITE(dt) && dt > 0.f) {
		const float time_constant = 1.f / (2.f * M_PI_F * _cutoff_hz);
		const float alpha = dt / (time_constant + dt);
		_filtered_yaw_rate_error += alpha * (yaw_rate_error - _filtered_yaw_rate_error);

	} else {
		_filtered_yaw_rate_error = yaw_rate_error;
	}

	return math::constrain(_direction * _gain * _filtered_yaw_rate_error, -_max_output, _max_output);
}

void AileronYawDamper::reset()
{
	_filtered_yaw_rate_error = 0.f;
	_filter_initialized = false;
}
