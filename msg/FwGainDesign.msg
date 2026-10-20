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
