# Fixed-wing GPS fallback and altitude-source status.

uint64 timestamp

bool enabled
bool gps_bad
bool sensor_gps_lost
bool vehicle_gps_lost
bool raw_gps_fresh
bool sensor_gps_fresh
bool baro_valid
bool fused_altitude_valid
bool fallback_suppressed_by_gpsless_takeoff

uint8 altitude_source
uint8 ALTITUDE_SOURCE_FUSED = 0
uint8 ALTITUDE_SOURCE_BARO = 1

bool yaw_hold_active
bool loiter_active

float32 raw_gps_eph
float32 raw_gps_epv
uint8 satellites_used

float32 fused_altitude
float32 baro_altitude
float32 used_altitude
float32 gps_good_elapsed_s

float32 held_yaw
float32 yaw_error
float32 fallback_roll_setpoint
