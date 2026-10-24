# Fixed-wing aerodynamic gain design

`fw_gain_design` is an advisory calculation module by default. Start it with
`fw_gain_design start`; it then publishes one `fw_gain_design` sample every
100 ms and cycles continuously through the configured speed schedule. It never
publishes a torque or actuator setpoint.

## Optional live application

`FW_GD_APPLY_R`, `FW_GD_APPLY_P`, and `FW_GD_APPLY_Y` independently enable
live roll, pitch, and yaw scheduling. All default to disabled. An enabled axis
uses only a fresh `airspeed_validated.true_airspeed_m_s` sample while the
vehicle is armed and the airspeed remains inside `FW_GD_VMIN` through
`FW_GD_VMAX`.

The roll and pitch switches update their respective `FW_RR_*` / `FW_PR_*`
rate gains and `FW_R_TC` / `FW_P_TC`; yaw updates `FW_YR_*`. Updates are
batched, rate-limited, and only sent when a calculated gain changes
meaningfully. The module captures the prior values when it first takes over an
axis and restores them when that axis is disabled, true airspeed becomes
unavailable or leaves the design range, the vehicle disarms, or the module is
stopped. Do not enable yaw application with `FW_YAW_DAMP_EN`, because the
aileron yaw damper intentionally disables the yaw-rate loop.

## Model and required inputs

The module uses independent linear rate models. For roll (the pitch and yaw
forms substitute `c, Iyy, Cm_q, Cm_du` and `b, Izz, Cn_r, Cn_du` respectively):

```
p_dot = a_p p + b_p u_roll
a_p = rho * V * S * b^2 * Cl_p / (4 * Ixx)
b_p = rho * V^2 * S * b * Cl_du * delta_roll_max / (2 * Ixx)
```

`Cl_p`, `Cm_q`, and `Cn_r` are the normal non-dimensional damping derivatives,
where the normalized rate is body rate multiplied by reference length divided
by `2V`. The `*_DU` inputs are moment-coefficient change per radian of physical
surface deflection. `*_MAX` is the physical travel corresponding to normalized
PX4 command `+1` after the mixer/control allocator. The signs therefore must
be expressed in PX4 body-moment convention: a positive normalized command must
produce a positive body-axis angular acceleration.

The model is only valid in the linear region and treats roll, pitch, and yaw as
decoupled. It will explicitly mark an axis invalid if its damping derivative is
not stabilizing or its control effectiveness is not positive. It does not turn
a coupled Dutch-roll, short-period, or rudderless aileron-yaw-damper model into
a false SISO yaw model.

## Rate and attitude design

For `G(s) = b/(s + omega_aircraft)`, where `omega_aircraft = -a`, the requested
rate natural frequency is `omega_c = bandwidth_ratio * omega_aircraft`. The PI
gains are exact coefficient matching of the closed-loop denominator:

```
Kp = (2*zeta*omega_c - omega_aircraft) / b
Ki = omega_c^2 / b
Kff = omega_aircraft / b
```

`Kff` is the direct steady-state inverse required by PX4's direct rate
feed-forward path. It removes the integrator load for a constant non-zero rate;
PX4 does not provide a setpoint-derivative feed-forward path.

The normal ECL roll/pitch attitude loop is a P-to-rate-setpoint controller.
Consequently it has no independent outer-loop damping knob. The module uses
bandwidth separation: `attitude_p = attitude_bandwidth_ratio * rate_bw`, and
`FW_R_TC` or `FW_P_TC = 1 / attitude_p`. A 0.2 to 0.3 outer-to-inner ratio is a
conservative starting range.

The schedule input is true airspeed and uses `FW_GD_RHO` in the aerodynamic
model. `px4_airspeed` reports the matching ISA equivalent/calibrated speed
(`TAS * sqrt(rho / 1.225)`) used for PX4's airspeed scaling.

`surface_rate_*` are direct physical surface-command gains. `px4_rate_*` are
the equivalent values for `FW_RR_*`, `FW_PR_*`, and `FW_YR_*`, after accounting
for the currently selected PX4 fixed-wing airspeed scaling and trim airspeed.
The results are published for review only; copying any of them into flight
parameters remains a deliberate manual action.
