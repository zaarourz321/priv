# Fixed-wing GPS-less navigation controller status.

uint64 timestamp

bool enabled                              # FW_GPSNAV_EN
bool active                               # controller has taken ownership after AUTO takeoff
bool baro_valid                           # fresh barometric altitude is available
bool gps_available                        # fresh raw GPS data is available (display only)
bool gps_ignored                          # always true while active

float32 baro_altitude                     # barometric altitude used by TECS [m]
float32 baro_climb_rate                   # barometric climb rate used by TECS [m/s]
float32 baro_altitude_setpoint            # selected barometric/AMSL altitude setpoint used by TECS [m]
float32 next_waypoint_altitude            # next waypoint altitude captured before mode entry [m]
uint8 altitude_mode                       # FW_GPSNAV_ALTMOD: 0 next waypoint, 1 fixed parameter

float32 airspeed                          # measured equivalent airspeed [m/s]
float32 airspeed_setpoint                 # GPS-less navigation airspeed demand [m/s]
float32 heading_setpoint                  # FW_GPSNAV_HDG [rad]
float32 heading_error                     # heading setpoint minus measured yaw [rad]
float32 roll_setpoint                     # bank command for heading capture [rad]

# Air-relative velocity estimated only from measured airspeed and attitude.
# It is diagnostic dead reckoning; it deliberately contains no GPS or wind correction.
float32 air_velocity_north
float32 air_velocity_east
float32 air_velocity_down
float32 dead_reckoned_north
float32 dead_reckoned_east

float32 raw_gps_eph                       # diagnostic only; never used for GPS-less NAV control [m]
float32 raw_gps_epv                       # diagnostic only; never used for GPS-less NAV control [m]
uint8 satellites_used                     # diagnostic only
