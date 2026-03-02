#pragma once

/**
 * @brief Strong-typed speed value that carries its own unit.
 *
 * Use the factory methods to construct a Speed rather than filling the
 * members directly.  The unit tag lets every function that accepts a Speed
 * choose the correct conversion without overload pollution.
 *
 * @example
 *   motor.setSpeed(Speed::fromPercentage(75));
 *   motor.setSpeed(Speed::fromWheelRPM(5.0f));
 */
struct Speed {
    /** Discriminator for the three supported speed representations. */
    enum class Unit { Percentage, WheelRPM, MotorRPM };

    float value; ///< Magnitude in the given unit.
    Unit  unit;  ///< Interpretation of @c value.

    /**
     * @brief Construct a Speed from a duty-cycle percentage.
     * @param pct Desired output level, 0–100.  Values outside this range are
     *            accepted but will saturate the PWM at the hardware layer.
     */
    static Speed fromPercentage(float pct) { return {pct, Unit::Percentage}; }

    /**
     * @brief Construct a Speed from a wheel-shaft angular velocity.
     * @param rpm Revolutions per minute of the wheel (after the gearbox).
     */
    static Speed fromWheelRPM(float rpm) { return {rpm, Unit::WheelRPM}; }

    /**
     * @brief Construct a Speed from a motor-shaft angular velocity.
     * @param rpm Revolutions per minute of the motor shaft (before the gearbox).
     */
    static Speed fromMotorRPM(float rpm) { return {rpm, Unit::MotorRPM}; }

    /** @returns @c true when the unit is Percentage. */
    bool isPercentage() const { return unit == Unit::Percentage; }

    /** @returns @c true when the unit is WheelRPM. */
    bool isWheelRPM()   const { return unit == Unit::WheelRPM; }

    /** @returns @c true when the unit is MotorRPM. */
    bool isMotorRPM()   const { return unit == Unit::MotorRPM; }
};
