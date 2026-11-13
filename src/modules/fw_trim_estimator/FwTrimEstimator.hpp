/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fw_trim_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace time_literals;

class FwTrimEstimator : public ModuleBase<FwTrimEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FwTrimEstimator();
	~FwTrimEstimator() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum Axis : uint8_t {
		ROLL = 0,
		PITCH = 1,
		AXIS_COUNT = 2,
	};

	static constexpr uint8_t MAX_PWM_OUTPUTS = 16; // PWM_MAIN1..8 then PWM_AUX1..8
	static constexpr int32_t OUTPUT_FUNCTION_SERVO1 = 201;

	void Run() override;
	float scheduled_trim_increment(Axis axis, float airspeed) const;
	bool common_conditions_met(float &airspeed, float &roll, float &pitch, uint8_t &reason) const;
	bool axis_conditions_met(Axis axis, float roll, float pitch, uint8_t common_reason, uint8_t &reason) const;
	void update_estimate(Axis axis, float candidate, float dt);
	void apply_estimate(Axis axis, hrt_abstime now);
	void update_pwm_mapping();
	void refresh_pwm_baselines();
	void apply_pwm_expansion(Axis axis, hrt_abstime now);
	void publish_status(float airspeed, float roll, float pitch, uint8_t common_reason);
	bool pwm_expansion_enabled(Axis axis) const;
	float pwm_trim_threshold(Axis axis) const;

	struct PwmOutput {
		param_t function{PARAM_INVALID};
		param_t minimum{PARAM_INVALID};
		param_t maximum{PARAM_INVALID};
		uint8_t axis_mask{0};
		int32_t baseline_min{0};
		int32_t baseline_max{0};
		int32_t expansion_us{0};
		bool baseline_valid{false};
	};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _torque_sp_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<fw_trim_status_s> _status_pub{ORB_ID(fw_trim_status)};

	vehicle_status_s _vehicle_status{};
	vehicle_control_mode_s _control_mode{};
	vehicle_land_detected_s _land_detected{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_rates_setpoint_s _rates_sp{};
	vehicle_torque_setpoint_s _torque_sp{};
	vehicle_local_position_s _local_position{};
	airspeed_validated_s _airspeed{};
	manual_control_setpoint_s _manual_control{};

	param_t _trim_parameter[AXIS_COUNT] {PARAM_INVALID, PARAM_INVALID};
	float _estimate[AXIS_COUNT] {NAN, NAN};
	float _candidate[AXIS_COUNT] {NAN, NAN};
	float _active_trim[AXIS_COUNT] {NAN, NAN};
	hrt_abstime _axis_stable_since[AXIS_COUNT] {};
	hrt_abstime _last_apply[AXIS_COUNT] {};
	uint32_t _sample_count[AXIS_COUNT] {};
	uint32_t _update_count[AXIS_COUNT] {};
	PwmOutput _pwm_outputs[MAX_PWM_OUTPUTS] {};
	hrt_abstime _last_pwm_apply[AXIS_COUNT] {};
	uint16_t _pwm_channel_mask[AXIS_COUNT] {};
	int32_t _pwm_expansion_us[AXIS_COUNT] {};
	bool _axis_conditions[AXIS_COUNT] {};
	uint8_t _axis_reason[AXIS_COUNT] {};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FW_TRIM_EST_EN>) _param_fw_trim_est_en,
		(ParamBool<px4::params::FW_TRIM_R_APPLY>) _param_fw_trim_r_apply,
		(ParamBool<px4::params::FW_TRIM_P_APPLY>) _param_fw_trim_p_apply,
		(ParamFloat<px4::params::FW_TRIM_TIME>) _param_fw_trim_time,
		(ParamFloat<px4::params::FW_TRIM_TAU>) _param_fw_trim_tau,
		(ParamFloat<px4::params::FW_TRIM_INT>) _param_fw_trim_int,
		(ParamFloat<px4::params::FW_TRIM_STEP>) _param_fw_trim_step,
		(ParamFloat<px4::params::FW_TRIM_LIM>) _param_fw_trim_lim,
		(ParamFloat<px4::params::FW_TRIM_RMAX>) _param_fw_trim_rmax,
		(ParamFloat<px4::params::FW_TRIM_PMAX>) _param_fw_trim_pmax,
		(ParamFloat<px4::params::FW_TRIM_LOIT_BK>) _param_fw_trim_loit_bk,
		(ParamBool<px4::params::FW_TRIM_LOIT>) _param_fw_trim_loit,
		(ParamFloat<px4::params::FW_TRIM_VZMAX>) _param_fw_trim_vzmax,
		(ParamFloat<px4::params::FW_TRIM_RATE>) _param_fw_trim_rate,
		(ParamFloat<px4::params::FW_TRIM_ERR>) _param_fw_trim_err,
		(ParamFloat<px4::params::FW_TRIM_STICK>) _param_fw_trim_stick,
		(ParamFloat<px4::params::FW_TRIM_ASPD_MIN>) _param_fw_trim_aspd_min,
		(ParamBool<px4::params::FW_TRIM_PWM_EN>) _param_fw_trim_pwm_en,
		(ParamBool<px4::params::FW_TRIM_R_PWM_EN>) _param_fw_trim_r_pwm_en,
		(ParamBool<px4::params::FW_TRIM_P_PWM_EN>) _param_fw_trim_p_pwm_en,
		(ParamFloat<px4::params::FW_TRIM_RPWM_THR>) _param_fw_trim_r_pwm_thr,
		(ParamFloat<px4::params::FW_TRIM_PPWM_THR>) _param_fw_trim_p_pwm_thr,
		(ParamFloat<px4::params::FW_TRIM_PWM_INT>) _param_fw_trim_pwm_int,
		(ParamInt<px4::params::FW_TRIM_PWM_STEP>) _param_fw_trim_pwm_step,
		(ParamInt<px4::params::FW_TRIM_PWM_MAX>) _param_fw_trim_pwm_max,

		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_DTRIM_R_VMIN>) _param_fw_dtrim_r_vmin,
		(ParamFloat<px4::params::FW_DTRIM_R_VMAX>) _param_fw_dtrim_r_vmax,
		(ParamFloat<px4::params::FW_DTRIM_P_VMIN>) _param_fw_dtrim_p_vmin,
		(ParamFloat<px4::params::FW_DTRIM_P_VMAX>) _param_fw_dtrim_p_vmax,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamInt<px4::params::CA_SV_CS_COUNT>) _param_ca_sv_cs_count
	)
};
