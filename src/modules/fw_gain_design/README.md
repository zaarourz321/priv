# Fixed-wing aerodynamic gain design

`fw_gain_design` is an advisory calculation module by default. Start it with
`fw_gain_design start`; it publishes `fw_gain_design` at 50 Hz and cycles
continuously through the configured speed schedule. It never publishes a
torque or actuator setpoint.

## Optional live application

`FW_GD_APPLY_R`, `FW_GD_APPLY_P`, and `FW_GD_APPLY_Y` independently enable
live roll, pitch, and yaw scheduling. All default to disabled. An enabled axis
uses only a fresh `airspeed_validated.true_airspeed_m_s` sample while the
vehicle is armed and the airspeed remains inside `FW_GD_VMIN` through
`FW_GD_VMAX`.

The active values are published at 50 Hz on the dedicated
`fw_gain_design_gains` uORB topic. Fixed-wing rate control consumes its P, I,
and FF gains directly; fixed-wing attitude control consumes its roll/pitch
time constants directly. This is the control path, so controller parameters
are not rewritten on every schedule update. A controller rejects a stale
(over 100 ms), invalid, disabled, unarmed, out-of-range, or excitation-axis
sample and immediately falls back to its configured `FW_*` parameters.

For compatibility and QGroundControl inspection, the selected values are
mirrored to `FW_RR_*` / `FW_PR_*` / `FW_YR_*` and `FW_R_TC` / `FW_P_TC` at
most once per minute. If those legacy values were written, their saved
baseline is restored immediately when an axis is disabled, the true airspeed
becomes unavailable or leaves the design range, the vehicle disarms, or the
module stops. Do not enable yaw application with `FW_YAW_DAMP_EN`, because
the aileron yaw damper intentionally disables the yaw-rate loop.

## Manual or identified model, per axis

The existing aerodynamic parameters remain the manual model inputs. `FW_GD_R_SRC`,
`FW_GD_P_SRC`, and `FW_GD_Y_SRC` select their source independently:

- `0` — Manual parameters (`FW_GD_CLP` / `FW_GD_CLDU`, `FW_GD_CMQ` /
  `FW_GD_CMDU`, or `FW_GD_CNR` / `FW_GD_CNDU`). This is the default.
- `1` — The most recent valid in-flight estimate for that one axis.

An identified model exists in RAM only. It is published on `fw_gain_design`,
but does not overwrite your manually entered derivatives or survive a reboot.
That is intentional: an unreviewed flight estimate must not become a permanent
airframe parameter. If an estimated source is selected but no valid estimate
exists, that axis is invalid and its live gains are restored rather than
falling back silently to manual values.

## Optional SD-card model persistence

`FW_GD_SD_MODEL=0` is the default: identified models remain in RAM only. Set
`FW_GD_SD_MODEL=1` to retain the latest validated model at:

```
/fs/microsd/fw_gain_design_model.bin
```

On startup the module retries loading that file while the SD card becomes
available. It accepts only the current file version with a valid checksum and
valid, stabilizing diagonal derivatives. Each later completed identification
with at least one valid axis writes a complete replacement through a temporary
file and atomic rename; a partial SD write will therefore not replace the last
valid model.

If you enable `FW_GD_SD_MODEL` after completing an identification, the module
also saves the already validated in-RAM result; it does not require repeating
the excitation.

Loading is intentionally passive: it restores the in-RAM flight-test estimate
only. It does **not** change `FW_GD_R_SRC`, `FW_GD_P_SRC`, or `FW_GD_Y_SRC`,
and it does **not** enable any `FW_GD_APPLY_*` option. You must still review
the restored output and explicitly select an estimated source before it can
influence a gain design.

## Flight-test excitation and identification

Roll and yaw identify the single-axis rate plants used by this gain designer:

```
p_dot = L_p * p + L_delta * u_roll + bias
r_dot = N_r * r + N_delta * u_yaw + bias
```

Pitch is deliberately different. A pitch motion contains both angle of attack
and pitch-rate dynamics, so treating it as a one-state `q/elevator` plant can
fold `Cm_alpha` into `Cm_q`. A pitch test instead identifies the physical
short-period equation:

```
q_dot = M_alpha * alpha + M_q * q + M_delta * u_elevator + bias
```

It runs two **sequential** single-frequency sine tests: first
`FW_GD_EXC_FREQ`, then `FW_GD_EXC_FREQ * FW_GD_P_FRAT`. There is never a
simultaneous multi-sine command. Between the two tests the command ramps back
to zero; samples are taken only after settling and before each ramp-out. The
module obtains `alpha` from the EKF local velocity rotated into body axes and
subtracts a fresh EKF wind estimate when available. A zero-wind fallback is
provided for HIL, but a valid wind estimate is recommended for real-flight
physical `Cm_alpha` accuracy.

During a test it adds one small, zero-mean, single-frequency sinusoidal
**body-rate setpoint** to the normal fixed-wing attitude controller. It never
publishes a competing `vehicle_torque_setpoint`.

The estimator always uses actual allocated `actuator_servos` commands, never a
virtual torque request. It reads the configured control-allocation surface
types and their configured torque effectiveness. Pitch uses Elevator surfaces;
yaw uses Rudder surfaces; and roll uses Left Aileron, Right Aileron, or
Single-channel Aileron surfaces. Multiple matching surfaces are combined with
their configured torque signs, so the input has the direction of the requested
body moment. The test waits rather than injects if the selected axis has no
usable physical surface command. This is essential: a virtual torque request
is not a physical deflection and cannot produce a trustworthy control
derivative.

To prevent raw gyro angular-acceleration noise from invalidating slow aircraft
tests, the estimator low-pass filters rate and the corresponding physical
control input equally. After the ramp and one settling cycle, it synchronously
fits the sine component of both signals. Their complex ratio is the measured
open-loop rate-to-surface response:

```
H(jω) = rate(jω) / surface_command(jω)
```

For roll and yaw, the first-order plant `H(jω) = b / (jω - a)` gives `a` and
`b` directly from the inverse response. Pitch solves the two measured complex
short-period equations for `M_alpha`, `M_q`, and `M_delta`; it publishes the
additional diagnostic `identified_pitch_cm_alpha`. `FW_GD_EST_FC` is the
filter cutoff and `FW_GD_EST_DLY` compensates the command-to-response delay as
a phase shift. The defaults are 2 Hz and zero delay, appropriate for HIL or
direct surface-command models. On real hardware, enter a measured non-zero
servo transport delay. At the end of a qualifying test, the module converts
the result at the mean tested true airspeed into the normal physical
derivatives:

```
Cl_p/Cm_q/Cn_r = 4 I a / (rho V S L^2)
Cl_du/Cm_du/Cn_du = 2 I b / (rho V^2 S L delta_max)
```

Here `L` is span for roll/yaw and chord for pitch; `delta_max` is the existing
physical travel parameter for that axis. Therefore area, reference dimensions,
inertia, density, and surface travel must be correct even when using flight
identification.

### Safe test sequence

1. Keep the manual model source selected and verify that the existing gains
   fly safely. Do not enable live scheduling for the axis being tested.
2. Set the applicable geometry, inertia, density, and `*_MAX` surface travel.
   Choose `FW_GD_EXC_AXIS`, a modest `FW_GD_EXC_AMP` (default 4 deg/s),
   frequency (default 0.7 Hz), duration, and rate safety limit.
3. In stable, level, fixed-wing, attitude-controlled flight with fresh true
   airspeed inside `FW_GD_VMIN..FW_GD_VMAX`, run one of:

   ```
   fw_gain_design excite start
   fw_gain_design excite start roll
   fw_gain_design excite start pitch
   fw_gain_design excite start yaw
   ```

   The bare `start` uses `FW_GD_EXC_AXIS`; the named form selects that one
   test without changing the stored parameter. Stop it at any time with:

   ```
   fw_gain_design excite stop
   ```

4. Check `listener fw_gain_design 1`. A usable result has
    `excitation_state = COMPLETE`, a non-zero `identification_sample_count`,
    `identification_valid[axis] = true`, negative identified damping, positive
    identified control derivative, and a small `identification_fit`. For pitch,
    also confirm `identified_pitch_cm_alpha` is finite and negative. The pitch
    fit includes the two harmonic fits and the short-period equation residual;
    lower is better.
    If a result is rejected, inspect
    `identification_reject_reason[axis]` and the published candidate values;
    candidates are diagnostic only until `identification_valid` is true.
5. Set only the tested axis' source to `1`. Review the generated gains. To
   schedule and apply those gains in flight, additionally enable the existing
   corresponding `FW_GD_APPLY_R`, `FW_GD_APPLY_P`, or `FW_GD_APPLY_Y` switch.
   The gains are then applied only while the aircraft is armed, airspeed is
   valid and in range, and the estimate remains valid.

The test waits without injecting until all conditions are true. It injects
nothing on the ground, during a transition, in failsafe, outside the speed
range, or unless both attitude and rate control are active. It ramps in over
two seconds, runs one axis only, and automatically removes the setpoint on
completion. It aborts immediately on pilot stick movement, loss of any flight
condition, a selected-axis rate breach, an explicit stop, or a stale input.

Yaw excitation is refused while `FW_YAW_DAMP_EN=1`. That is required for your
rudderless aileron-yaw-damper aircraft: it has no physical yaw-axis control
surface to identify as `Cn_du`. Keep `FW_GD_Y_SRC=0` and `FW_GD_APPLY_Y=0` on
that airframe.

For `fmuv6x`, the older PX4 fixed-wing autotune module is disabled to reserve
flash for this physical-model estimator. The simulation PWM output remains
enabled.

## Model and required inputs

The gain calculation uses the validated diagonal rate model. For roll (the
pitch and yaw forms substitute `c, Iyy, Cm_q, Cm_du` and `b, Izz, Cn_r,
Cn_du` respectively):

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

The model is only valid in the linear region. Identification and gain design
both use this validated diagonal dynamics model. It will explicitly mark an axis invalid if
its damping derivative is not stabilizing or its control effectiveness is not
positive. It does not turn a rudderless aileron-yaw-damper aircraft into a
controllable yaw-rate axis.

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
The results are published for review. They remain advisory unless the existing
per-axis `FW_GD_APPLY_*` switch is deliberately enabled.
