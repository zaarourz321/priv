uint64 timestamp                         # time since system start (microseconds)

# A monotonically increasing identifier. It changes after any relevant
# parameter update, so consumers can distinguish a new schedule from an old one.
uint32 calculation_id
uint16 speed_index                       # zero-based index within this schedule
uint16 speed_count                       # total number of speed points in this schedule

float32 airspeed                         # true airspeed for this model point [m/s]
float32 px4_airspeed                     # equivalent/calibrated airspeed used for PX4 scaling [m/s]

# Estimated open-loop rate-mode bandwidths, derived from the aerodynamic
# damping derivatives. The axes are roll, pitch, yaw. [rad/s]
float32[3] aircraft_rate_bw

# Requested closed-loop rate bandwidths and damping ratios. [rad/s], [-]
float32[3] rate_cl_bw
float32[3] rate_cl_zeta

# Linear rate-plant coefficients for x_dot = a_rate * x + b_rate * u.
# u is the normalized PX4 control-axis command before FW airspeed scaling.
float32[3] rate_a
float32[3] rate_b

# PI plus direct steady-state feed-forward gains for the unscaled normalized
# surface command. P units: command/(rad/s), I: command/rad, FF: command/(rad/s).
float32[3] surface_rate_p
float32[3] surface_rate_i
float32[3] surface_rate_ff

# Equivalent values for FW_RR_*, FW_PR_*, and FW_YR_* at this airspeed. These
# include the existing PX4 fixed-wing airspeed scaling when it is enabled.
float32[3] px4_rate_p
float32[3] px4_rate_i
float32[3] px4_rate_ff

# Recommended outer-loop attitude values for roll then pitch. The existing
# ECL attitude controllers use attitude_p = 1 / attitude_tc. [rad/s], [s]
float32[2] attitude_p
float32[2] attitude_tc

# Per-axis design result: 0 valid, otherwise one of AXIS_STATUS_* below.
uint8[3] axis_status

uint8 AXIS_STATUS_VALID = 0
uint8 AXIS_STATUS_INVALID_INPUT = 1
uint8 AXIS_STATUS_NOT_DAMPED = 2
uint8 AXIS_STATUS_NO_CONTROL_AUTHORITY = 3
uint8 AXIS_STATUS_INVALID_DESIGN = 4
uint8 AXIS_STATUS_NONFINITE_RESULT = 5

uint8 SCHEDULE_STATUS_VALID = 0
uint8 SCHEDULE_STATUS_INVALID_RANGE = 1
uint8 schedule_status

# Model source used independently for roll, pitch, yaw: 0 manual parameters,
# 1 retained flight-test estimate.
uint8[3] model_source
uint8 MODEL_SOURCE_MANUAL = 0
uint8 MODEL_SOURCE_FLIGHT_TEST = 1

# Coupled manual-model inputs, published so they can be directly compared to
# their identified equivalents. Axes are roll, pitch, yaw.
float32[3] manual_static_derivative      # Cl_beta, Cm_alpha, Cn_beta
float32[3] manual_cross_rate_derivative  # Cl_r, NaN, Cn_p

# Flight-test excitation telemetry. Excitation is a bounded rate-setpoint
# addition handled by fixed-wing attitude control; this module never publishes
# a competing torque setpoint.
uint8 excitation_state
uint8 EXCITATION_STATE_IDLE = 0
uint8 EXCITATION_STATE_WAITING_FOR_CONDITIONS = 1
uint8 EXCITATION_STATE_RUNNING = 2
uint8 EXCITATION_STATE_COMPLETE = 3
uint8 EXCITATION_STATE_ABORTED = 4

uint8 excitation_axis                 # 0 roll, 1 pitch, 2 yaw
float32[3] excitation_rate_sp         # injected body-rate setpoint [rad/s]
float32 excitation_elapsed_s
uint8 excitation_abort_reason
uint8 EXCITATION_ABORT_NONE = 0
uint8 EXCITATION_ABORT_STOP_REQUESTED = 1
uint8 EXCITATION_ABORT_INVALID_CONFIGURATION = 2
uint8 EXCITATION_ABORT_FLIGHT_CONDITIONS = 3
uint8 EXCITATION_ABORT_PILOT_INPUT = 4
uint8 EXCITATION_ABORT_RATE_LIMIT = 5
uint8 EXCITATION_ABORT_YAW_DAMPER = 6

# Identified linear rate models. The estimator fits a low-pass-filtered,
# delay-compensated discrete rate model and converts it to rate_dot = a * rate
# + b * command. Candidate values are published even when they are rejected;
# only identification_valid permits them to be used for gain design.
uint32[3] identification_sample_count
float32[3] identified_airspeed
float32[3] identified_rate_a
float32[3] identified_rate_b
float32[3] identified_damping_derivative # Cl_p, Cm_q, Cn_r
float32[3] identified_control_derivative # Cl_du, Cm_du, Cn_du [per rad]
float32[3] identification_fit            # normalized RMS residual, lower is better
bool[3] identification_valid

# Pitch-only short-period identification diagnostics. When enabled, the pitch
# estimator uses estimated angle of attack as a separate state so Cmq is not
# contaminated by Cm_alpha coupling. Angle of attack is in radians.
bool identified_pitch_short_period
float32 identified_pitch_cm_alpha
float32 identified_pitch_angle_of_attack

# Coupled-model diagnostics. Roll and yaw tests use beta/p/r; pitch tests use
# alpha/q. These values are advisory model-identification telemetry only and
# are never automatically applied as controller gains.
bool[3] identified_coupled_model
float32[3] identified_airflow_angle          # beta for roll/yaw, alpha for pitch [rad]
float32[3] identified_static_derivative      # Cl_beta, Cm_alpha, Cn_beta
float32[3] identified_cross_rate_derivative  # Cl_r, NaN, Cn_p
float32[3] identified_airflow_rate_a         # alpha/beta state self coefficient [1/s]
float32[3] identified_airflow_rate_b         # alpha/beta coupling from p/q/r [1/s]
float32[3] identified_airflow_control_b      # alpha/beta control coefficient [1/s]
float32[3] identification_airflow_fit        # normalized RMS of alpha/beta fit, lower is better

# Per-axis reason why a completed candidate was rejected. A value of VALID is
# only meaningful when the corresponding identification_valid element is true.
uint8[3] identification_reject_reason
uint8 IDENTIFICATION_REJECT_NOT_RUN = 0
uint8 IDENTIFICATION_REJECT_VALID = 1
uint8 IDENTIFICATION_REJECT_INSUFFICIENT_SAMPLES = 2
uint8 IDENTIFICATION_REJECT_INVALID_INPUT = 3
uint8 IDENTIFICATION_REJECT_UNSTABLE_DAMPING = 4
uint8 IDENTIFICATION_REJECT_CONTROL_DIRECTION = 5
uint8 IDENTIFICATION_REJECT_LOW_EXCITATION = 6
uint8 IDENTIFICATION_REJECT_POOR_FIT = 7
uint8 IDENTIFICATION_REJECT_INVALID_DERIVATIVE = 8
