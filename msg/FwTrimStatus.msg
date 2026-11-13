uint64 timestamp

# Current common flight-condition inputs.
float32 airspeed                         # calibrated airspeed [m/s]
float32 roll                             # measured roll angle [rad]
float32 pitch                            # measured pitch angle [rad]
float32 vertical_speed                   # local NED down velocity [m/s]
uint8 common_reject_reason

# Arrays are ordered Roll, Pitch.
bool[2] axis_conditions_met
bool[2] estimate_valid
bool[2] apply_enabled
float32[2] candidate_trim                # instantaneous base trim candidate
float32[2] estimated_trim                # filtered base-trim estimate
float32[2] active_trim                   # current TRIM_ROLL, TRIM_PITCH values
float32[2] stable_time_s
uint32[2] sample_count
uint32[2] update_count
uint8[2] reject_reason

# PWM headroom expansion. The channel bit mask uses PWM_MAIN1..8 at bits 0..7
# and PWM_AUX1..8 at bits 8..15. Expansion is applied symmetrically to the
# stored channel range: MIN decreases and MAX increases by this many us.
bool[2] pwm_expansion_enabled
bool[2] pwm_expansion_active
uint16[2] pwm_channel_mask
int32[2] pwm_expansion_us

uint8 REJECT_NONE = 0
uint8 REJECT_DISABLED = 1
uint8 REJECT_NOT_FIXED_WING = 2
uint8 REJECT_NOT_ARMED = 3
uint8 REJECT_LANDED = 4
uint8 REJECT_CONTROL_MODE = 5
uint8 REJECT_STALE_DATA = 6
uint8 REJECT_AIRSPEED = 7
uint8 REJECT_VERTICAL_MOTION = 8
uint8 REJECT_RATE_MOTION = 9
uint8 REJECT_PILOT_INPUT = 10
uint8 REJECT_ATTITUDE = 11
uint8 REJECT_OUTPUT_LIMIT = 12
