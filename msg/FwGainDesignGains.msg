# Active fixed-wing gains selected by fw_gain_design.
#
# This is the fast control-path interface. Controllers only use an axis while
# its corresponding valid flag is true and the sample is fresh. The ordinary
# FW_* parameters remain the fallback values and are synchronized only for
# compatibility/inspection at a slow rate.

uint64 timestamp

float32 airspeed

# Axes are roll, pitch, yaw.
bool[3] rate_valid
bool[3] attitude_valid

float32[3] rate_p
float32[3] rate_i
float32[3] rate_ff
float32[3] attitude_tc
