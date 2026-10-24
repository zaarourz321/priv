
uint64 timestamp            # time since system start (microseconds)

uint64 timestamp_sample     # the timestamp of the raw data (microseconds)

uint32 baro_device_id       # unique device ID for the selected barometer

float32 baro_alt_meter			# Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH.
float32 baro_temp_celcius		# Temperature in degrees Celsius
float32 baro_pressure_pa		# Absolute pressure in Pascals

float32 rho				# air density

uint8 calibration_count     # Calibration changed counter. Monotonically increases whenever calibration changes.

# Preflight automatic QNH calibration status (BQNH_EN): 0 disabled, 1 waiting
# for arming state, 2 held because armed, 3 waiting for GPS, 4 rejected GPS
# sample, 5 learning, 6 tracking, 7 parameter write error.
uint8 qnh_calibration_state
float32 qnh_estimate_hpa    # Filtered preflight QNH estimate, hPa (NaN when unavailable)
