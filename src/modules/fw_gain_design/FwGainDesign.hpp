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
#include <cstdint>

#include <lib/parameters/param.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fw_gain_design.h>
#include <uORB/topics/fw_gain_design_gains.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

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
	void update_flight_test();
	void reset_estimator(Axis axis);
	void update_estimator(Axis axis, float rate, float control_input, float true_airspeed, float dt);
	void finalize_estimate(Axis axis);
	bool flight_test_conditions_met(Axis axis, float &true_airspeed) const;
	bool control_surface_input(Axis axis, float &control_input) const;
	void update_control_surface_mapping();
	bool model_source_is_estimated(Axis axis) const;
	bool estimate_valid(Axis axis) const;
	bool request_excitation(Axis axis);
	void abort_excitation(uint8_t reason);
	void fill_flight_test_status(fw_gain_design_s &design) const;
	bool load_model_from_sd();
	bool save_model_to_sd();
	void publish_live_gains();
	bool apply_axis(Axis axis, const fw_gain_design_s &design);
	bool restore_axis(Axis axis);
	void restore_inactive_axes(bool armed);
	bool apply_parameter(param_t parameter, float value, bool restore);
	bool apply_enabled(Axis axis) const;
	bool capture_axis_baseline(Axis axis);
	uint8_t validate_common_inputs() const;

	static constexpr uint8_t kTorqueHistoryLength = 20;
	static constexpr uint8_t kMaxModelOrder = 3;
	static constexpr uint32_t kEstimatorWarmupSamples = 100;

	struct RateEstimator {
		// rate[k] = A * rate[k-1] + B * command[k-delay] + offset.
		// The input is an allocated physical control-surface command.
		float theta[kMaxModelOrder] {};
		float covariance[kMaxModelOrder][kMaxModelOrder] {};
		float residual_sum_sq{0.f};
		float output_sum_sq{0.f};
		float output_sum{0.f};
		float command_sum{0.f};
		float command_sum_sq{0.f};
		float airspeed_sum{0.f};
		float dt_sum{0.f};
		float filtered_rate{NAN};
		float previous_filtered_rate{NAN};
		float filtered_torque{NAN};
		float torque_history[kTorqueHistoryLength] {};
		uint32_t sample_count{0};
		uint32_t fit_sample_count{0};
		uint8_t torque_history_index{0};
		bool filters_initialized{false};
		bool initialized{false};
	};

	// Stored on the SD card only when an excitation produces one or more
	// validated axes. The file is versioned and checksummed before it is used.
	struct PersistentModel {
		uint32_t magic;
		uint16_t version;
		uint16_t size;
		uint8_t valid_mask;
		uint8_t reserved[3];
		float damping_derivative[AXIS_COUNT];
		float control_derivative[AXIS_COUNT];
		float rate_a[AXIS_COUNT];
		float rate_b[AXIS_COUNT];
		float airspeed[AXIS_COUNT];
		uint32_t checksum;
	};

	static bool update_rls(float theta[kMaxModelOrder], float covariance[kMaxModelOrder][kMaxModelOrder],
			       const float phi[kMaxModelOrder], uint8_t model_order, float output,
			       float forgetting_factor, float &fitted_output);
	static uint32_t model_checksum(const PersistentModel &model);

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionData<actuator_servos_s> _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::SubscriptionData<vehicle_angular_velocity_s> _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_control_mode_s> _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionData<vehicle_land_detected_s> _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionData<manual_control_setpoint_s> _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<fw_gain_design_s> _gain_design_pub{ORB_ID(fw_gain_design)};
	uORB::Publication<fw_gain_design_gains_s> _gain_design_gains_pub{ORB_ID(fw_gain_design_gains)};

	uint32_t _calculation_id{0};
	uint16_t _speed_index{0};
	uint16_t _speed_count{0};
	uint8_t _schedule_status{fw_gain_design_s::SCHEDULE_STATUS_INVALID_RANGE};
	hrt_abstime _last_gain_parameter_sync{0};
	hrt_abstime _last_estimator_sample{0};

	RateEstimator _estimator[AXIS_COUNT] {};
	float _identified_damping_derivative[AXIS_COUNT] {NAN, NAN, NAN};
	float _identified_control_derivative[AXIS_COUNT] {NAN, NAN, NAN};
	float _identified_rate_a[AXIS_COUNT] {NAN, NAN, NAN};
	float _identified_rate_b[AXIS_COUNT] {NAN, NAN, NAN};
	float _identified_airspeed[AXIS_COUNT] {NAN, NAN, NAN};
	bool _identified_valid[AXIS_COUNT] {};
	uint8_t _identification_reject_reason[AXIS_COUNT] {
		fw_gain_design_s::IDENTIFICATION_REJECT_NOT_RUN,
		fw_gain_design_s::IDENTIFICATION_REJECT_NOT_RUN,
		fw_gain_design_s::IDENTIFICATION_REJECT_NOT_RUN
	};

	uint8_t _excitation_state{fw_gain_design_s::EXCITATION_STATE_IDLE};
	uint8_t _excitation_abort_reason{fw_gain_design_s::EXCITATION_ABORT_NONE};
	Axis _excitation_axis{ROLL};
	bool _excitation_start_requested{false};
	// Set by the shell command path and consumed by the work-queue thread.
	// 1..3 request roll/pitch/yaw, 4 requests an immediate stop.
	px4::atomic<uint8_t> _excitation_command{0};
	float _excitation_amplitude{0.f};
	float _excitation_frequency{0.f};
	float _excitation_duration{0.f};
	float _excitation_rate_sp{0.f};
	hrt_abstime _excitation_start{0};

	bool _sd_model_loaded{false};
	bool _sd_model_saved{false};
	bool _sd_model_dirty{false};
	hrt_abstime _last_sd_model_load_attempt{0};
	hrt_abstime _last_sd_model_save_attempt{0};

	// The test uses existing attitude/rate control. It adds this bounded rate
	// setpoint only while the state below is RUNNING; it never publishes a
	// competing torque setpoint.
	static constexpr float kExcitationRampTimeS = 2.f;

	param_t _rate_p_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _rate_i_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _rate_ff_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _attitude_tc_param[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _control_surface_count_param{PARAM_INVALID};
	param_t _control_surface_type_param[actuator_servos_s::NUM_CONTROLS] {};
	param_t _control_surface_torque_param[actuator_servos_s::NUM_CONTROLS][AXIS_COUNT] {};
	float _control_surface_weight[AXIS_COUNT][actuator_servos_s::NUM_CONTROLS] {};
	bool _control_surface_available[AXIS_COUNT] {};
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
		(ParamInt<px4::params::FW_GD_R_SRC>) _param_fw_gd_r_src,
		(ParamInt<px4::params::FW_GD_P_SRC>) _param_fw_gd_p_src,
		(ParamInt<px4::params::FW_GD_Y_SRC>) _param_fw_gd_y_src,
		(ParamInt<px4::params::FW_GD_EXC_AXIS>) _param_fw_gd_exc_axis,
		(ParamFloat<px4::params::FW_GD_EXC_AMP>) _param_fw_gd_exc_amp,
		(ParamFloat<px4::params::FW_GD_EXC_FREQ>) _param_fw_gd_exc_freq,
		(ParamFloat<px4::params::FW_GD_EXC_TIME>) _param_fw_gd_exc_time,
		(ParamFloat<px4::params::FW_GD_EXC_RMAX>) _param_fw_gd_exc_rmax,
		(ParamFloat<px4::params::FW_GD_EST_TC>) _param_fw_gd_est_tc,
		(ParamFloat<px4::params::FW_GD_EST_FC>) _param_fw_gd_est_fc,
		(ParamFloat<px4::params::FW_GD_EST_DLY>) _param_fw_gd_est_dly,
		(ParamInt<px4::params::FW_GD_SD_MODEL>) _param_fw_gd_sd_model,
		(ParamInt<px4::params::FW_GD_EST_MIN>) _param_fw_gd_est_min,
		(ParamInt<px4::params::FW_GD_APPLY_R>) _param_fw_gd_apply_r,
		(ParamInt<px4::params::FW_GD_APPLY_P>) _param_fw_gd_apply_p,
		(ParamInt<px4::params::FW_GD_APPLY_Y>) _param_fw_gd_apply_y,

		// Existing PX4 settings used to report and, if explicitly enabled, apply
		// directly comparable FW_* gains.
		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamInt<px4::params::FW_YAW_DAMP_EN>) _param_fw_yaw_damp_en
	)
};
