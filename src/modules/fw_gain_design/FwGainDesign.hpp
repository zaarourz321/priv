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
 * @file FwGainDesign.hpp
 *
 * Parameter-driven fixed-wing controller gain design.
 *
 * It always publishes a calculated speed schedule on uORB. Its three
 * per-axis apply switches are disabled by default. When enabled, an axis is
 * updated from valid true airspeed while armed and restored to its prior
 * values when disarmed or disabled.
 */

#pragma once

#include <cmath>

#include <lib/parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fw_gain_design.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class FwGainDesign : public ModuleBase<FwGainDesign>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FwGainDesign();
	~FwGainDesign() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum Axis : uint8_t {
		ROLL = 0,
		PITCH = 1,
		YAW = 2,
		AXIS_COUNT = 3,
	};

	void Run() override;
	void reset_schedule();
	void publish_next_design_point();
	void calculate_design(float true_airspeed, fw_gain_design_s &design) const;
	void apply_live_gains();
	bool apply_axis(Axis axis, const fw_gain_design_s &design);
	bool restore_axis(Axis axis);
	void restore_inactive_axes(bool armed);
	bool apply_parameter(param_t parameter, float value, bool restore);
	bool apply_enabled(Axis axis) const;
	bool capture_axis_baseline(Axis axis);
	uint8_t validate_common_inputs() const;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Publication<fw_gain_design_s> _gain_design_pub{ORB_ID(fw_gain_design)};

	uint32_t _calculation_id{0};
	uint16_t _speed_index{0};
	uint16_t _speed_count{0};
	uint8_t _schedule_status{fw_gain_design_s::SCHEDULE_STATUS_INVALID_RANGE};
	hrt_abstime _last_gain_apply{0};

	param_t _rate_p_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _rate_i_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _rate_ff_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _attitude_tc_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	float _baseline_rate_p[AXIS_COUNT] {NAN, NAN, NAN};
	float _baseline_rate_i[AXIS_COUNT] {NAN, NAN, NAN};
	float _baseline_rate_ff[AXIS_COUNT] {NAN, NAN, NAN};
	float _baseline_attitude_tc[AXIS_COUNT] {NAN, NAN, NAN};
	bool _axis_overridden[AXIS_COUNT] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_GD_VMIN>) _param_fw_gd_vmin,
		(ParamFloat<px4::params::FW_GD_VMAX>) _param_fw_gd_vmax,
		(ParamFloat<px4::params::FW_GD_VSTEP>) _param_fw_gd_vstep,
		(ParamFloat<px4::params::FW_GD_RHO>) _param_fw_gd_rho,
		(ParamFloat<px4::params::FW_GD_SREF>) _param_fw_gd_sref,
		(ParamFloat<px4::params::FW_GD_BREF>) _param_fw_gd_bref,
		(ParamFloat<px4::params::FW_GD_CREF>) _param_fw_gd_cref,
		(ParamFloat<px4::params::FW_GD_IX>) _param_fw_gd_ix,
		(ParamFloat<px4::params::FW_GD_IY>) _param_fw_gd_iy,
		(ParamFloat<px4::params::FW_GD_IZ>) _param_fw_gd_iz,
		(ParamFloat<px4::params::FW_GD_CLP>) _param_fw_gd_clp,
		(ParamFloat<px4::params::FW_GD_CMQ>) _param_fw_gd_cmq,
		(ParamFloat<px4::params::FW_GD_CNR>) _param_fw_gd_cnr,
		(ParamFloat<px4::params::FW_GD_CLDU>) _param_fw_gd_cldu,
		(ParamFloat<px4::params::FW_GD_CMDU>) _param_fw_gd_cmdu,
		(ParamFloat<px4::params::FW_GD_CNDU>) _param_fw_gd_cndu,
		(ParamFloat<px4::params::FW_GD_R_MAX>) _param_fw_gd_r_max,
		(ParamFloat<px4::params::FW_GD_P_MAX>) _param_fw_gd_p_max,
		(ParamFloat<px4::params::FW_GD_Y_MAX>) _param_fw_gd_y_max,
		(ParamFloat<px4::params::FW_GD_R_BWR>) _param_fw_gd_r_bwr,
		(ParamFloat<px4::params::FW_GD_P_BWR>) _param_fw_gd_p_bwr,
		(ParamFloat<px4::params::FW_GD_Y_BWR>) _param_fw_gd_y_bwr,
		(ParamFloat<px4::params::FW_GD_R_ZETA>) _param_fw_gd_r_zeta,
		(ParamFloat<px4::params::FW_GD_P_ZETA>) _param_fw_gd_p_zeta,
		(ParamFloat<px4::params::FW_GD_Y_ZETA>) _param_fw_gd_y_zeta,
		(ParamFloat<px4::params::FW_GD_R_ABWR>) _param_fw_gd_r_abwr,
		(ParamFloat<px4::params::FW_GD_P_ABWR>) _param_fw_gd_p_abwr,
		(ParamInt<px4::params::FW_GD_APPLY_R>) _param_fw_gd_apply_r,
		(ParamInt<px4::params::FW_GD_APPLY_P>) _param_fw_gd_apply_p,
		(ParamInt<px4::params::FW_GD_APPLY_Y>) _param_fw_gd_apply_y,

		// Existing PX4 settings used to report and, if explicitly enabled, apply
		// directly comparable FW_* gains.
		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim
	)
};
