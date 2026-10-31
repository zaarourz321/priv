# Electromagnetic safety and deployment telemetry.

uint64 timestamp                         # time since system start [us]

float32 acc                              # longitudinal acceleration [m/s^2]
float32 pressure_diff                    # differential pressure [Pa]
float32 capacitor_volt                   # capacitor voltage [V]
float32 pressure                         # static pressure [Pa]

uint8 status_flags                       # EM_FLAG_* bitmask
uint8 fsm_state                          # EM_STATE_*
uint32 mcu_timestamp                     # source MCU timestamp [us]

# Status flags
uint8 EM_FLAG_AP_EN = 1
uint8 EM_FLAG_ARMED = 2
uint8 EM_FLAG_ACC_TH = 4
uint8 EM_FLAG_BARO_TH = 8

# State machine states
uint8 EM_STATE_IDLE = 0
uint8 EM_STATE_WAIT_FOR_ARM = 1
uint8 EM_STATE_ARMED = 2
uint8 EM_STATE_CHARGE = 3
uint8 EM_STATE_SHOOT = 4
