#pragma once

#include "DCMotorWithEncoder.h"
#include "Speed.h"
#include "Direction.h"

/**
 * @brief Mecanum-wheel drivetrain: computes per-wheel velocities and drives
 *        four DCMotorWithEncoder instances.
 *
 * ### Coordinate convention
 * - **x** — lateral axis, positive = right.
 * - **y** — longitudinal axis, positive = forward.
 * - **rotation (ω)** — positive = CCW when viewed from above.
 *
 * All motion vectors passed to strafe() are normalised to [-1, 1] before
 * computing wheel speeds.  If the resulting wheel-speed vector would exceed
 * magnitude 1 on any wheel the entire vector is scaled down proportionally so
 * the fastest wheel runs at exactly the requested @p speed.
 *
 * ### Kinematics
 * @code
 *   v_FL =  Vy + Vx - ω
 *   v_FR =  Vy - Vx + ω
 *   v_RL =  Vy - Vx - ω
 *   v_RR =  Vy + Vx + ω
 * @endcode
 *
 * ### Maintenance
 * Call update() on every @c loop() iteration; it forwards the call to each
 * motor so that non-blocking rotation targets (if set) are tracked correctly.
 */
class MecanumDrivetrain {
public:
    /**
     * @brief Construct a drivetrain from four pre-configured motor instances.
     *
     * @param fl            Front-left motor (M1).
     * @param fr            Front-right motor (M2).
     * @param rl            Rear-left motor (M3).
     * @param rr            Rear-right motor (M4).
     * @param trackWidth_mm Lateral distance between left and right wheel
     *                      contact patches, in millimetres.
     * @param wheelBase_mm  Longitudinal distance between front and rear wheel
     *                      contact patches, in millimetres.
     */
    MecanumDrivetrain(DCMotorWithEncoder& fl, DCMotorWithEncoder& fr,
                      DCMotorWithEncoder& rl, DCMotorWithEncoder& rr,
                      float trackWidth_mm,
                      float wheelBase_mm);

    /**
     * @brief Translate the robot in 2-D without any rotation.
     *
     * Equivalent to strafe(x, y, 0.0f, speed).
     *
     * @param x     Lateral component, -1 (full left) to +1 (full right).
     * @param y     Forward component, -1 (full back) to +1 (full forward).
     * @param speed Magnitude of the resulting wheel speeds.
     */
    void strafe(float x, float y,
                Speed speed = Speed::fromPercentage(50));

    /**
     * @brief Translate and rotate the robot simultaneously.
     *
     * Computes raw wheel speeds from the mecanum kinematic equations, then
     * normalises them so no wheel exceeds the requested @p speed, and
     * applies the result.
     *
     * @param x        Lateral component, -1 to +1.
     * @param y        Forward component, -1 to +1.
     * @param rotation Yaw rate, -1 to +1.  Positive = CCW (left turn),
     *                 negative = CW (right turn).
     * @param speed    Magnitude applied after normalisation.
     */
    void strafe(float x, float y, float rotation,
                Speed speed = Speed::fromPercentage(50));

    /**
     * @brief Drive the robot in a circle around a fixed centre point.
     *
     * The robot maintains its orientation relative to the centre (i.e. it
     * always faces tangentially) while orbiting at the requested speed.
     *
     * An effective moment arm @c Leff = (trackWidth + wheelBase) / 4 is used
     * to derive the rotation component:
     * @code
     *   Direction::Left  → Vx = +1, ω = +Leff / radius_mm  (CCW orbit)
     *   Direction::Right → Vx = -1, ω = -Leff / radius_mm  (CW  orbit)
     * @endcode
     *
     * As @p radius_mm → ∞ the motion approaches a pure lateral strafe.
     * When @p radius_mm == @c Leff one pair of diagonally opposite wheels
     * stops and the other pair carries all the speed.
     *
     * @param radius_mm  Orbit radius in millimetres (same units as the
     *                   constructor parameters).  Must be > 0.
     * @param direction  ::Direction::Left for a CCW orbit (robot moves right),
     *                   ::Direction::Right for a CW orbit (robot moves left).
     * @param speed      Tangential speed of the robot along the arc.
     */
    void circle(float radius_mm, Direction direction,
                Speed speed = Speed::fromPercentage(50));

    /**
     * @brief Stop all four motors immediately.
     */
    void stop();

    /**
     * @brief Forward the periodic update to each motor and apply saturation
     *        compensation.
     *
     * Must be called every @c loop() iteration.  In addition to ticking each
     * motor's PID loop and RPM estimator, this method checks whether any
     * motor is power-saturated (running at 100 % duty cycle but below its
     * RPM target).  If so, all four motors' targets are scaled down
     * proportionally to the worst-case ratio so the mecanum kinematics remain
     * consistent even when one wheel is load-limited.
     *
     * The compensation factor only decreases; it resets to 1.0 the next time
     * strafe() or circle() is called.
     */
    void update();

private:
    DCMotorWithEncoder& _fl;
    DCMotorWithEncoder& _fr;
    DCMotorWithEncoder& _rl;
    DCMotorWithEncoder& _rr;

    float _trackWidth; ///< Lateral wheel-centre distance in mm.
    float _wheelBase;  ///< Longitudinal wheel-centre distance in mm.

    // ── Saturation compensation state ────────────────────────────────────────
    float _lastFL  = 0.0f; ///< Normalised Front-Left velocity fraction from last command.
    float _lastFR  = 0.0f; ///< Normalised Front-Right velocity fraction from last command.
    float _lastRL  = 0.0f; ///< Normalised Rear-Left velocity fraction from last command.
    float _lastRR  = 0.0f; ///< Normalised Rear-Right velocity fraction from last command.
    float _lastSpeedPct       = 0.0f;  ///< Speed percentage requested by the last command.
    float _compensationFactor = 1.0f;  ///< Current scale applied to all wheel targets (0–1].
    bool  _motionActive       = false; ///< True while the robot is commanded to move.

    /**
     * @brief Convert a Speed value to a percentage for use with setSpeed(float).
     *
     * - Percentage: returned as-is.
     * - MotorRPM / WheelRPM: the raw @c value is returned unchanged here;
     *   the individual motors resolve the unit when their setSpeed(Speed) is
     *   called instead.
     *
     * @param speed Speed to resolve.
     * @returns Percentage value in the range 0–100.
     */
    float _resolveSpeedPct(Speed speed) const;

    /**
     * @brief Normalise raw wheel velocities and drive all four motors.
     *
     * If the largest absolute value among the four inputs exceeds 1.0 every
     * value is divided by it so the fastest wheel is capped at @p speedPct.
     * Values that are already within [-1, 1] are used directly.
     *
     * @param fl       Raw Front-Left velocity, arbitrary scale.
     * @param fr       Raw Front-Right velocity.
     * @param rl       Raw Rear-Left velocity.
     * @param rr       Raw Rear-Right velocity.
     * @param speedPct Output magnitude applied after normalisation, 0–100 %.
     */
    void _applyWheelValues(float fl, float fr, float rl, float rr,
                           float speedPct);
};
