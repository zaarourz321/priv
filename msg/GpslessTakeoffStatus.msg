# GPS-less fixed-wing tube-launch altitude-source status.

uint64 timestamp                         # time since system start (microseconds)

bool enabled                             # FW_GPSL_TKO_EN and FW_LAUN_DETCN_ON are enabled
bool post_takeoff_hold_enabled           # FW_GPSL_POST_EN
bool active                              # barometric altitude source is currently active
bool using_baro_altitude                 # TECS is currently using barometric altitude
bool baro_valid                          # fresh finite barometric altitude is available
bool gps_healthy                          # fused global position meets all configured handover quality checks
bool altitude_converged                  # estimator and barometer are within FW_GPSL_ALT_ERR
bool handover_ready                      # all time, health, and convergence conditions are met
bool predefined_heading_active            # FW_GPSFB_TK_HDG is controlling roll after tube takeoff
bool gps_recovered                        # qualified GPS/estimator and altitude handover completed after tube takeoff

uint8 state
uint8 STATE_DISABLED = 0
uint8 STATE_WAITING_FOR_BARO = 1
uint8 STATE_TAKEOFF = 2
uint8 STATE_POST_TAKEOFF_HOLD = 3
uint8 STATE_ESTIMATOR = 4

float32 global_altitude                  # fused global/estimator altitude AMSL [m]
float32 baro_altitude                    # QNH-referenced barometric altitude AMSL [m]
float32 used_altitude                    # altitude currently passed to TECS [m]
float32 hold_altitude_setpoint            # barometric altitude captured at AUTO takeoff handover [m]
float32 launch_baro_altitude             # barometric altitude captured in launch tube [m]
float32 altitude_difference              # absolute global/barometric altitude difference [m]

float32 global_eph                       # fused global horizontal uncertainty [m]
float32 global_epv                       # fused global vertical uncertainty [m]

float32 post_takeoff_elapsed_s           # elapsed time since AUTO takeoff completed [s]
float32 post_takeoff_hold_timeout_s      # FW_GPSL_HOLD_T [s]
float32 gps_healthy_elapsed_s            # continuous healthy and converged estimator time [s]
float32 gps_healthy_timeout_s            # FW_GPSL_GPS_T [s]
