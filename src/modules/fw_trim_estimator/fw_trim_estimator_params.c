/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Enable fixed-wing in-flight trim estimation
 *
 * When enabled, the module continuously estimates trim during stable flight.
 * It never writes TRIM_ROLL or TRIM_PITCH unless that axis's Apply parameter
 * is also enabled.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_EST_EN, 1);

/**
 * Apply estimated roll trim
 *
 * Allows periodic in-flight writes to TRIM_ROLL after the stable-flight time.
 * Disabled by default.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_R_APPLY, 0);

/**
 * Apply estimated pitch trim
 *
 * Allows periodic in-flight writes to TRIM_PITCH after the stable-flight time.
 * Disabled by default.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_P_APPLY, 0);

/**
 * Required continuous stable-flight time
 *
 * Both the flight-condition gate and the selected axis gate must remain valid
 * for this long before a learned value can be applied.
 *
 * @unit s
 * @min 5
 * @max 1800
 * @decimal 0
 * @increment 10
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_TIME, 60.f);

/**
 * Trim estimate filter time constant
 *
 * Larger values reject transient controller effort more strongly.
 *
 * @unit s
 * @min 5
 * @max 3600
 * @decimal 0
 * @increment 10
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_TAU, 120.f);

/**
 * Minimum interval between trim parameter writes
 *
 * @unit s
 * @min 5
 * @max 3600
 * @decimal 0
 * @increment 10
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_INT, 60.f);

/**
 * Maximum change per parameter write
 *
 * This bounds each update of TRIM_ROLL or TRIM_PITCH.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @increment 0.001
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_STEP, 0.01f);

/**
 * Absolute learned trim limit
 *
 * Samples and applied values outside this normalized control range are rejected.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_LIM, 0.25f);

/**
 * Maximum bank angle for normal trim learning
 *
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 0
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_RMAX, 12.f);

/**
 * Maximum pitch angle for trim learning
 *
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 0
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_PMAX, 12.f);

/**
 * Allow roll trim learning in auto loiter
 *
 * Pitch trim always requires near-level flight because a banked turn needs
 * different lift. When enabled, roll trim can additionally be learned in a
 * steady AUTO.LOITER up to FW_TRIM_LOIT_BK bank.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_LOIT, 0);

/**
 * Maximum bank angle for roll learning in auto loiter
 *
 * @unit deg
 * @min 1
 * @max 60
 * @decimal 0
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_LOIT_BK, 35.f);

/**
 * Maximum absolute vertical speed while learning trim
 *
 * Learning pauses while climbing or descending faster than this limit.
 *
 * @unit m/s
 * @min 0.05
 * @max 5
 * @decimal 2
 * @increment 0.05
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_VZMAX, 0.3f);

/**
 * Maximum body roll or pitch rate while learning trim
 *
 * @unit deg/s
 * @min 0.1
 * @max 45
 * @decimal 1
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_RATE, 3.f);

/**
 * Maximum roll or pitch rate tracking error while learning trim
 *
 * @unit deg/s
 * @min 0.1
 * @max 45
 * @decimal 1
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_ERR, 2.f);

/**
 * Maximum manual roll or pitch stick displacement while learning trim
 *
 * @min 0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_STICK, 0.05f);

/**
 * Minimum calibrated airspeed for trim learning
 *
 * @unit m/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.5
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_ASPD_MIN, 12.f);

/**
 * Enable PWM headroom expansion from learned trim
 *
 * When enabled with an axis selector, the module expands the configured PWM
 * MIN and MAX values for conventional aileron or elevator outputs only after
 * the corresponding trim estimate is valid and above its threshold. It never
 * changes unrelated outputs. Disabled by default.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_PWM_EN, 0);

/**
 * Enable aileron PWM headroom expansion
 *
 * Requires FW_TRIM_PWM_EN and a valid persistent roll trim estimate.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_R_PWM_EN, 0);

/**
 * Enable elevator PWM headroom expansion
 *
 * Requires FW_TRIM_PWM_EN and a valid persistent pitch trim estimate.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_P_PWM_EN, 0);

/**
 * Persistent roll-trim threshold for PWM expansion
 *
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_RPWM_THR, 0.10f);

/**
 * Persistent pitch-trim threshold for PWM expansion
 *
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_PPWM_THR, 0.10f);

/**
 * Minimum interval between PWM range expansions
 *
 * @unit s
 * @min 5
 * @max 3600
 * @decimal 0
 * @increment 10
 * @group FW Trim Estimator
 */
PARAM_DEFINE_FLOAT(FW_TRIM_PWM_INT, 60.f);

/**
 * Symmetric PWM range expansion step
 *
 * For each selected output, MIN is decreased and MAX is increased by this
 * amount at each expansion interval.
 *
 * @unit us
 * @min 1
 * @max 100
 * @increment 1
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_PWM_STEP, 10);

/**
 * Maximum symmetric PWM range expansion
 *
 * Maximum total expansion from the channel range recorded when the module
 * starts or when the operator changes the channel PWM range.
 *
 * @unit us
 * @min 1
 * @max 200
 * @increment 1
 * @group FW Trim Estimator
 */
PARAM_DEFINE_INT32(FW_TRIM_PWM_MAX, 100);
