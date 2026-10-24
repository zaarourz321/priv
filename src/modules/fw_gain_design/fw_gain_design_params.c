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
 * @file fw_gain_design_params.c
 *
 * Parameters for the advisory fixed-wing aerodynamic gain design module.
 */

/**
 * Minimum scheduled airspeed
 *
 * The gain schedule starts at this true airspeed model point.
 * This module is advisory only and does not change any flight gains.
 *
 * @unit m/s
 * @min 0.1
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_VMIN, 10.0f);

/**
 * Maximum scheduled airspeed
 *
 * The final published true airspeed point is exactly this speed, even if the step
 * does not divide the interval exactly.
 *
 * @unit m/s
 * @min 0.1
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_VMAX, 25.0f);

/**
 * Gain schedule airspeed step
 *
 * One `fw_gain_design` uORB sample is published per step and the module
 * continuously cycles through the complete schedule.
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_VSTEP, 1.0f);

/**
 * Air density used by the linear model
 *
 * Use the density representative of the schedule altitude. The supplied
 * aerodynamic coefficients are assumed to be non-dimensional. The module
 * also publishes the corresponding ISA calibrated/equivalent airspeed used
 * when it calculates PX4 airspeed-scaled gain equivalents.
 *
 * @min 0.1
 * @max 1.5
 * @decimal 3
 * @increment 0.025
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_RHO, 1.225f);

/**
 * Reference wing area
 *
 * Reference area S used by the supplied aerodynamic moment coefficients.
 * Set this and the other physical-model inputs before trusting a result.
 *
 * @unit m^2
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_SREF, 0.0f);

/**
 * Reference span
 *
 * Wing span b used for the roll and yaw coefficient normalization.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_BREF, 0.0f);

/**
 * Reference chord
 *
 * Mean aerodynamic chord c used for pitch coefficient normalization.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_CREF, 0.0f);

/**
 * Roll moment of inertia
 *
 * Aircraft inertia Ixx about the body roll axis.
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 4
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_IX, 0.0f);

/**
 * Pitch moment of inertia
 *
 * Aircraft inertia Iyy about the body pitch axis.
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 4
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_IY, 0.0f);

/**
 * Yaw moment of inertia
 *
 * Aircraft inertia Izz about the body yaw axis.
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 4
 * @group FW Gain Design
 */
PARAM_DEFINE_FLOAT(FW_GD_IZ, 0.0f);

/**
 * Roll damping derivative
 *
 * Non-dimensional Cl_p using p-hat = p*b/(2*V). A statically damped roll
 * rate has a negative value.
 *
 * @min -100.0
 * @max 0.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CLP, 0.0f);

/**
 * Pitch damping derivative
 *
 * Non-dimensional Cm_q using q-hat = q*c/(2*V). A damped pitch rate has a
 * negative value.
 *
 * @min -100.0
 * @max 0.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CMQ, 0.0f);

/**
 * Yaw damping derivative
 *
 * Non-dimensional Cn_r using r-hat = r*b/(2*V). A damped yaw rate has a
 * negative value.
 *
 * @min -100.0
 * @max 0.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CNR, 0.0f);

/**
 * Roll control derivative
 *
 * C_l per radian of physical roll-control deflection. Define the sign after
 * mixer/allocation so a positive PX4 roll command has a positive value.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CLDU, 0.0f);

/**
 * Pitch control derivative
 *
 * C_m per radian of physical pitch-control deflection. Define the sign after
 * mixer/allocation so a positive PX4 pitch command has a positive value.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CMDU, 0.0f);

/**
 * Yaw control derivative
 *
 * C_n per radian of physical yaw-control deflection. Define the sign after
 * mixer/allocation so a positive PX4 yaw command has a positive value.
 * A rudderless aircraft can leave this at zero; yaw gains will then be marked
 * invalid rather than invented.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_CNDU, 0.0f);

/**
 * Roll control travel
 *
 * Physical deflection produced by a normalized positive roll-axis command.
 * This turns C_l control derivative per radian into PX4 normalized-command
 * effectiveness. Use the effective differential-aileron travel.
 *
 * @unit rad
 * @min 0.0
 * @max 3.14
 * @decimal 3
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_R_MAX, 0.0f);

/**
 * Pitch control travel
 *
 * Physical deflection produced by a normalized positive pitch-axis command.
 *
 * @unit rad
 * @min 0.0
 * @max 3.14
 * @decimal 3
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_P_MAX, 0.0f);

/**
 * Yaw control travel
 *
 * Physical deflection produced by a normalized positive yaw-axis command.
 * Set to zero on a rudderless aircraft; this marks only the yaw result as
 * having no control authority and does not prevent roll or pitch calculations.
 *
 * @unit rad
 * @min 0.0
 * @max 3.14
 * @decimal 3
 * @increment 0.01
 * @group FW Gain Design Aerodynamics
 */
PARAM_DEFINE_FLOAT(FW_GD_Y_MAX, 0.0f);

/**
 * Roll rate bandwidth ratio
 *
 * Requested roll closed-loop natural frequency divided by the estimated
 * open-loop roll rate bandwidth.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_R_BWR, 3.0f);

/**
 * Pitch rate bandwidth ratio
 *
 * Requested pitch closed-loop natural frequency divided by the estimated
 * open-loop pitch rate bandwidth.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_P_BWR, 3.0f);

/**
 * Yaw rate bandwidth ratio
 *
 * Requested yaw closed-loop natural frequency divided by the estimated
 * open-loop yaw rate bandwidth.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_Y_BWR, 3.0f);

/**
 * Roll rate damping ratio
 *
 * Target closed-loop damping ratio used by the roll PI pole placement.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_R_ZETA, 0.8f);

/**
 * Pitch rate damping ratio
 *
 * Target closed-loop damping ratio used by the pitch PI pole placement.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_P_ZETA, 0.8f);

/**
 * Yaw rate damping ratio
 *
 * Target closed-loop damping ratio used by the yaw PI pole placement.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Rate
 */
PARAM_DEFINE_FLOAT(FW_GD_Y_ZETA, 0.8f);

/**
 * Roll attitude bandwidth ratio
 *
 * Outer attitude bandwidth divided by its designed roll-rate bandwidth.
 * Keep this well below one so the inner rate loop remains faster.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Attitude
 */
PARAM_DEFINE_FLOAT(FW_GD_R_ABWR, 0.25f);

/**
 * Pitch attitude bandwidth ratio
 *
 * Outer attitude bandwidth divided by its designed pitch-rate bandwidth.
 * Keep this well below one so the inner rate loop remains faster.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Attitude
 */
PARAM_DEFINE_FLOAT(FW_GD_P_ABWR, 0.25f);

/**
 * Roll model source
 *
 * Selects the roll model used by the gain calculation.
 *
 * @value 0 Manual: use FW_GD_CLP and FW_GD_CLDU
 * @value 1 Flight estimate: use the most recent valid roll excitation result
 * @group FW Gain Design Model
 */
PARAM_DEFINE_INT32(FW_GD_R_SRC, 0);

/**
 * Pitch model source
 *
 * Selects the pitch model used by the gain calculation.
 *
 * @value 0 Manual: use FW_GD_CMQ and FW_GD_CMDU
 * @value 1 Flight estimate: use the most recent valid pitch excitation result
 * @group FW Gain Design Model
 */
PARAM_DEFINE_INT32(FW_GD_P_SRC, 0);

/**
 * Yaw model source
 *
 * Selects the yaw model used by the gain calculation.
 *
 * @value 0 Manual: use FW_GD_CNR and FW_GD_CNDU
 * @value 1 Flight estimate: use the most recent valid yaw excitation result
 * @group FW Gain Design Model
 */
PARAM_DEFINE_INT32(FW_GD_Y_SRC, 0);

/**
 * Flight-test excitation axis
 *
 * Selects the one axis that `fw_gain_design excite start` may test. An
 * excitation is never started merely by changing this parameter. Test one
 * axis at a time in level, stable flight.
 *
 * @value 0 Disabled
 * @value 1 Roll
 * @value 2 Pitch
 * @value 3 Yaw
 * @min 0
 * @max 3
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_INT32(FW_GD_EXC_AXIS, 0);

/**
 * Flight-test rate-setpoint amplitude
 *
 * Peak amplitude of the zero-mean sine rate setpoint injected during an
 * explicitly started test. Start small and increase only after reviewing a
 * log. The module ramps this amplitude over two seconds.
 *
 * @unit deg/s
 * @min 0.5
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_FLOAT(FW_GD_EXC_AMP, 4.0f);

/**
 * Flight-test excitation frequency
 *
 * Sine-wave frequency used to persistently excite the selected rate axis.
 * Keep it below the closed-loop rate bandwidth and above slow navigation
 * corrections.
 *
 * @unit Hz
 * @min 0.1
 * @max 3.0
 * @decimal 2
 * @increment 0.05
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_FLOAT(FW_GD_EXC_FREQ, 0.7f);

/**
 * Flight-test excitation duration
 *
 * Length of an explicitly started excitation. The command is removed on
 * completion, on any safety abort, and when `fw_gain_design excite stop` is
 * issued.
 *
 * @unit s
 * @min 8.0
 * @max 90.0
 * @decimal 1
 * @increment 1.0
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_FLOAT(FW_GD_EXC_TIME, 30.0f);

/**
 * Flight-test rate safety limit
 *
 * Abort the test if the measured selected-axis body rate exceeds this value.
 * This is a safety limit, not the normal test amplitude.
 *
 * @unit deg/s
 * @min 5.0
 * @max 180.0
 * @decimal 1
 * @increment 1.0
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_FLOAT(FW_GD_EXC_RMAX, 45.0f);

/**
 * Identification forgetting time constant
 *
 * Recursive least-squares memory used during a flight test. A longer value
 * gives a quieter estimate but needs more excitation data.
 *
 * @unit s
 * @min 2.0
 * @max 120.0
 * @decimal 1
 * @increment 1.0
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_FLOAT(FW_GD_EST_TC, 20.0f);

/**
 * Minimum identification samples
 *
 * A completed test needs at least this many accepted samples before its
 * estimated derivatives can be used as a model source.
 *
 * @min 100
 * @max 4000
 * @group FW Gain Design Excitation
 */
PARAM_DEFINE_INT32(FW_GD_EST_MIN, 500);

/**
 * Apply roll gains from airspeed
 *
 * When enabled and the aircraft is armed, apply the designed roll rate P, I,
 * and feed-forward gains, plus the roll attitude time constant, from fresh
 * true airspeed. This only operates between FW_GD_VMIN and FW_GD_VMAX.
 * The original values are restored when disarmed or when this option is
 * disabled. Disabled by default.
 *
 * @boolean
 * @group FW Gain Design Apply
 */
PARAM_DEFINE_INT32(FW_GD_APPLY_R, 0);

/**
 * Apply pitch gains from airspeed
 *
 * When enabled and the aircraft is armed, apply the designed pitch rate P, I,
 * and feed-forward gains, plus the pitch attitude time constant, from fresh
 * true airspeed. This only operates between FW_GD_VMIN and FW_GD_VMAX.
 * The original values are restored when disarmed or when this option is
 * disabled. Disabled by default.
 *
 * @boolean
 * @group FW Gain Design Apply
 */
PARAM_DEFINE_INT32(FW_GD_APPLY_P, 0);

/**
 * Apply yaw gains from airspeed
 *
 * When enabled and the aircraft is armed, apply the designed yaw rate P, I,
 * and feed-forward gains from fresh true airspeed. This only operates between
 * FW_GD_VMIN and FW_GD_VMAX. The original values are restored when disarmed
 * or when this option is disabled. Disabled by default.
 *
 * Do not enable on an aircraft using FW_YAW_DAMP_EN: yaw-rate parameters are
 * intentionally inactive while the aileron yaw damper owns yaw damping.
 *
 * @boolean
 * @group FW Gain Design Apply
 */
PARAM_DEFINE_INT32(FW_GD_APPLY_Y, 0);
