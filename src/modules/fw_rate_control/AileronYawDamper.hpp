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

/**
 * @file AileronYawDamper.hpp
 *
 * A yaw-rate damper for fixed-wing airframes without a rudder. The damper
 * drives the roll control surface (ailerons) to use its adverse-yaw coupling
 * to oppose body yaw-rate disturbances.
 */

#pragma once

class AileronYawDamper
{
public:
	/**
	 * Configure the damper.
	 *
	 * @param gain Normalized aileron command per rad/s of yaw rate.
	 * @param cutoff_hz Low-pass cutoff applied to the yaw-rate error. A
	 *                  non-positive cutoff bypasses the filter.
	 * @param max_output Maximum absolute normalized aileron contribution.
	 * @param direction +1 when positive roll aileron deflection produces
	 *                  negative yaw; -1 for the reverse installation.
	 */
	void setParameters(float gain, float cutoff_hz, float max_output, int direction);

	/**
	 * Calculate the normalized aileron contribution.
	 *
	 * @param yaw_rate_error Measured body yaw rate minus the coordinated-turn
	 *                       body yaw-rate target [rad/s].
	 * @param dt Controller time step [s].
	 * @return Normalized roll/aileron command in [-max_output, max_output].
	 */
	float update(float yaw_rate_error, float dt);

	void reset();

private:
	float _gain{0.f};
	float _cutoff_hz{0.f};
	float _max_output{0.f};
	float _direction{1.f};
	float _filtered_yaw_rate_error{0.f};
	bool _filter_initialized{false};
};
