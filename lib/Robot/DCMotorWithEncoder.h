#pragma once

#include <Arduino.h>
#include "L293DShield.h"
#include "Speed.h"

/**
 * @brief Controls one DC motor with a quadrature encoder through an L293DShield.
 *
 * Each instance manages a single motor channel: it writes direction and PWM
 * commands to the shared L293DShield, and reads position feedback from a
 * two-channel (A+B) incremental encoder via a hardware external interrupt on
 * channel A.
 *
 * Up to four instances can coexist simultaneously.  The constructor
 * auto-registers the instance in the first free slot of the static
 * @c _instances table and attaches the corresponding ISR.
 *
 * ### Non-blocking rotation targets
 * rotateMotorRevolutions() and friends start a move and return immediately.
 * The application must call update() on every @c loop() iteration; that call
 * checks whether the encoder count has reached the target and calls stop()
 * automatically when it has.
 *
 * ### Thread / ISR safety
 * @c _encoderCount is @c volatile and is read atomically (with
 * noInterrupts()/interrupts()) in getEncoderCount() and resetEncoder().
 * All other state is only ever written from the main execution context.
 *
 * @note The maximum number of simultaneous instances is 4.
 */
class DCMotorWithEncoder {
public:
    /**
     * @brief Construct and register a motor+encoder instance.
     *
     * Assigns the instance to the next free ISR slot (0–3), configures
     * encoder pins as @c INPUT_PULLUP, and attaches the RISING-edge interrupt
     * on @p encoderPinA.
     *
     * @param shield             Reference to the shared L293DShield.
     * @param motorNum           Motor channel on the shield, 1–4.
     * @param encoderPinA        Arduino pin connected to encoder channel A.
     *                           Must be an external-interrupt-capable pin
     *                           (Mega 2560: 2, 18, 19, 20, 21).
     * @param encoderPinB        Arduino pin connected to encoder channel B
     *                           (any digital pin). Read in the ISR to
     *                           determine rotation direction.
     * @param countsPerRevolution Number of encoder pulses per full motor-shaft
     *                           revolution (A-channel RISING edges only).
     *                           For a 14-pole magnet ring use 14 × 4 = 56
     *                           if counting all edges, or 14 if counting
     *                           only RISING edges on A.
     * @param gearRatio          Motor shaft revolutions per wheel revolution
     *                           (e.g. 30 for a 30:1 gearbox).
     * @param maxMotorRPM        No-load motor-shaft speed in RPM, used to
     *                           convert RPM targets to PWM percentages.
     */
    DCMotorWithEncoder(L293DShield& shield,
                       uint8_t motorNum,
                       int encoderPinA,
                       int encoderPinB,
                       float countsPerRevolution,
                       float gearRatio,
                       float maxMotorRPM);

    // ── Speed control ────────────────────────────────────────────────────────

    /**
     * @brief Drive the motor at a given percentage of full power.
     *
     * Cancels any active rotation target before applying the new speed.
     *
     * @param percentage Output level, -100.0–+100.0.  Positive values drive
     *                   forward (A=HIGH); negative values drive in reverse
     *                   (B=HIGH); zero coasts.
     */
    void setSpeed(float percentage);

    /**
     * @brief Drive the motor at a given motor-shaft speed.
     *
     * Converts @p motorRPM to a percentage using @c maxMotorRPM supplied at
     * construction time, then calls setSpeed(float).
     *
     * @param motorRPM Desired motor-shaft speed in RPM.  Positive = forward.
     */
    void setSpeedRPM(float motorRPM);

    /**
     * @brief Drive the motor so the wheel turns at a given speed.
     *
     * Converts @p wheelRPM to a motor-shaft RPM by multiplying by @c gearRatio,
     * then calls setSpeedRPM().
     *
     * @param wheelRPM Desired wheel speed in RPM.  Positive = forward.
     */
    void setSpeedWheelRPM(float wheelRPM);

    /**
     * @brief Unified speed setter that dispatches on the unit carried by @p speed.
     *
     * Equivalent to calling setSpeed(), setSpeedRPM(), or setSpeedWheelRPM()
     * depending on @c speed.unit.
     *
     * @param speed Desired speed with embedded unit information.
     */
    void setSpeed(Speed speed);

    /**
     * @brief Stop the motor immediately and cancel any active rotation target.
     *
     * Sets direction to coast and PWM to 0.
     */
    void stop();

    // ── Non-blocking rotation control ────────────────────────────────────────

    /**
     * @brief Rotate the motor shaft a given number of revolutions, then stop.
     *
     * Calculates the required encoder-count delta and starts the motor.
     * The move is non-blocking: call update() every @c loop() iteration to
     * detect completion.
     *
     * @param revs      Number of motor-shaft revolutions.  Negative values
     *                  rotate in reverse.
     * @param speedPct  Magnitude of speed while moving, 0–100 %.
     */
    void rotateMotorRevolutions(float revs, float speedPct = 50.0f);

    /**
     * @brief Rotate the motor shaft a given number of revolutions at a set RPM.
     *
     * Like rotateMotorRevolutions() but expresses speed in motor-shaft RPM.
     *
     * @param revs     Number of motor-shaft revolutions.
     * @param motorRPM Motor-shaft speed during the move in RPM.
     */
    void rotateMotorRevolutionsRPM(float revs, float motorRPM);

    /**
     * @brief Rotate the wheel a given number of revolutions, then stop.
     *
     * Converts @p revs to motor-shaft revolutions using @c gearRatio, then
     * behaves identically to rotateMotorRevolutions().
     *
     * @param revs     Number of wheel revolutions.  Negative = reverse.
     * @param speedPct Magnitude of speed while moving, 0–100 %.
     */
    void rotateWheelRevolutions(float revs, float speedPct = 50.0f);

    /**
     * @brief Rotate the wheel a given number of revolutions at a set motor RPM.
     *
     * @param revs     Number of wheel revolutions.
     * @param motorRPM Motor-shaft speed during the move in RPM.
     */
    void rotateWheelRevolutionsRPM(float revs, float motorRPM);

    /**
     * @brief Unified rotation entry point used internally by all rotate*() helpers.
     *
     * @param revs        Revolution count (positive = forward).
     * @param isWheelRevs @c true → @p revs is in wheel revolutions;
     *                    @c false → @p revs is in motor-shaft revolutions.
     * @param speed       Speed during the move (any unit).
     */
    void rotate(float revs, bool isWheelRevs, Speed speed);

    // ── Encoder feedback ─────────────────────────────────────────────────────

    /**
     * @brief Read the raw encoder count since the last resetEncoder() call.
     *
     * Disables interrupts briefly to ensure a consistent 32-bit read.
     *
     * @returns Signed pulse count.  Positive = net forward rotation;
     *          negative = net reverse rotation.
     */
    long getEncoderCount();

    /**
     * @brief Reset the encoder count and the RPM tracking baseline to zero.
     *
     * Safe to call from the main context; uses noInterrupts()/interrupts()
     * to avoid a torn write to the volatile counter.
     */
    void resetEncoder();

    /**
     * @brief Return the most recently computed motor-shaft speed.
     *
     * The value is updated by update() at most once every 50 ms using a
     * simple delta-count / delta-time calculation.  It will be 0 until the
     * first update() call after at least 50 ms have elapsed.
     *
     * @returns Motor-shaft speed in RPM.  Positive = forward.
     */
    float getMotorRPM() const;

    /**
     * @brief Return the most recently computed wheel speed.
     *
     * Divides getMotorRPM() by the gearRatio supplied at construction.
     *
     * @returns Wheel speed in RPM.  Positive = forward.
     */
    float getWheelRPM();

    /**
     * @brief Returns whether a rotation target is currently active.
     *
     * @returns @c true from the moment a rotate*() call is made until
     *          update() detects that the encoder count has reached the
     *          target and calls stop().
     */
    bool isRunning();

    // ── Closed-loop speed control ─────────────────────────────────────────────

    /**
     * @brief Override the PID gains used for closed-loop speed control.
     *
     * The default values (Kp=0.5, Ki=0.1, Kd=0.05) are a reasonable starting
     * point for a geared DC motor at low-to-medium speeds.  Tune them on the
     * actual hardware with a connected encoder.
     *
     * @param kp Proportional gain — corrects steady-state tracking error.
     * @param ki Integral gain — eliminates persistent offset.
     * @param kd Derivative gain — damps oscillation.
     */
    void setPIDGains(float kp, float ki, float kd);

    /**
     * @brief Return the current motor-shaft RPM target set by the PID loop.
     *
     * This is the signed target derived from the last setSpeed() call.
     * Positive = forward, negative = reverse.  Zero when closed-loop is
     * inactive (e.g. during a rotation target or after stop()).
     *
     * @returns Target RPM, or 0 if closed-loop is not active.
     */
    float getTargetRPM() const;

    /**
     * @brief Return the PWM percentage that was last applied to the motor.
     *
     * Updated by every _applySpeed() call (including those made by the PID
     * loop).  Useful for diagnosing saturation: a value at ±100 while the
     * motor is below its target RPM indicates that the motor cannot reach
     * the requested speed.
     *
     * @returns Current duty-cycle percentage in the range -100–+100.
     */
    float getCurrentPWMPct() const;

    /**
     * @brief Returns @c true when the motor is working at full power but
     *        cannot reach its target RPM.
     *
     * Saturation is defined as:
     *  - Closed-loop control is active, AND
     *  - |PWM| ≥ 99 % (at or within 1 % of the hardware limit), AND
     *  - |actual RPM| < 90 % of |target RPM|, AND
     *  - |actual RPM| > 5 % of |target RPM| (not a true stall).
     *
     * The 5 % lower bound prevents a robot pressed hard against a wall
     * (all wheels at zero) from triggering compensation unnecessarily.
     *
     * @returns @c true if the motor is power-saturated but speed-limited.
     */
    bool isSaturated() const;

    /**
     * @brief Scale the current RPM target by @p factor without resetting the
     *        PID integrator.
     *
     * Called internally by MecanumDrivetrain::update() to apply saturation
     * compensation across all four wheels.  Has no effect if closed-loop
     * control is not active.
     *
     * @param factor Multiplicative scale applied to _targetRPM, e.g. 0.8 to
     *               reduce the target to 80 % of its current value.
     */
    void adjustTargetRPM(float factor);

    /**
     * @brief Periodic maintenance — must be called every @c loop() iteration.
     *
     * Performs three tasks:
     *  1. **RPM calculation**: samples the encoder count and elapsed time at
     *     most once every 50 ms and updates the value returned by getMotorRPM().
     *  2. **PID speed control**: if closed-loop is active, computes a PID
     *     correction from the RPM error and updates the motor PWM.  Runs at
     *     the same rate as the RPM measurement (every 50 ms).
     *  3. **Target tracking**: if a rotation target is active, checks whether
     *     the encoder count has reached (or passed) the target and calls stop()
     *     if so.
     */
    void update();

    // ── ISR plumbing ─────────────────────────────────────────────────────────

    /**
     * @brief Encoder ISR body — called by the static ISR stubs below.
     *
     * Reads channel B to determine direction: HIGH → forward (count++),
     * LOW → reverse (count--).  Must not be called from user code.
     */
    void _handleEncoder();

    /** Static table of registered instances, indexed by ISR slot (0–3). */
    static DCMotorWithEncoder* _instances[4];

    /** ISR stub for slot 0 — attached to encoder channel A of the first motor. */
    static void _isr0();
    /** ISR stub for slot 1 — attached to encoder channel A of the second motor. */
    static void _isr1();
    /** ISR stub for slot 2 — attached to encoder channel A of the third motor. */
    static void _isr2();
    /** ISR stub for slot 3 — attached to encoder channel A of the fourth motor. */
    static void _isr3();

private:
    L293DShield& _shield;
    uint8_t      _motorNum;
    int          _encoderPinA;
    int          _encoderPinB;
    float        _countsPerRev;    ///< Encoder pulses per motor-shaft revolution.
    float        _gearRatio;       ///< Motor revolutions per wheel revolution.
    float        _maxMotorRPM;     ///< No-load motor speed used for RPM→PWM conversion.

    volatile long _encoderCount = 0; ///< Running pulse count; written by ISR.
    long          _lastCount    = 0; ///< Snapshot used by the RPM calculator.
    unsigned long _lastRPMTime  = 0; ///< Timestamp of the last RPM update (ms).
    float         _currentRPM   = 0.0f;

    bool  _targetActive    = false; ///< A rotation target is in progress.
    long  _targetCount     = 0;    ///< Absolute encoder count to reach.
    float _runningSpeedPct = 0.0f; ///< Speed sign used to pick done condition.

    // ── Closed-loop PID state ─────────────────────────────────────────────────
    bool  _closedLoopActive = false; ///< True while setSpeed(non-zero) is controlling.
    float _targetRPM        = 0.0f;  ///< Signed motor-shaft RPM setpoint.
    float _currentPWMPct    = 0.0f;  ///< Last PWM percentage applied to the motor.
    float _kp               = 0.5f;  ///< Proportional gain.
    float _ki               = 0.1f;  ///< Integral gain.
    float _kd               = 0.05f; ///< Derivative gain.
    float _pidIntegral      = 0.0f;  ///< Accumulated integral term.
    float _pidLastError     = 0.0f;  ///< Error from the previous PID update.

    /**
     * @brief Write direction and PWM to the shield and record the applied
     *        duty-cycle in @c _currentPWMPct.
     * @param percentage -100–+100; sign sets direction, magnitude sets duty cycle.
     */
    void _applySpeed(float percentage);
};
