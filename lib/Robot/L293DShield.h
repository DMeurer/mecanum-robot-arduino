#pragma once

#include <Arduino.h>

/** @name Shift-register control pins (fixed by shield PCB) @{ */
static constexpr uint8_t MOTORLATCH  = 12; ///< 74HC595 latch (RCLK).
static constexpr uint8_t MOTORCLK   = 4;  ///< 74HC595 clock (SRCLK).
static constexpr uint8_t MOTORENABLE = 7;  ///< 74HC595 output enable — active LOW.
static constexpr uint8_t MOTORDATA  = 8;  ///< 74HC595 serial data (SER).
/** @} */

/**
 * @brief Driver for the AZDelivery L293D Motor Shield (74HC595-based).
 *
 * The shield uses a single 74HC595 shift register to control the direction of
 * all four H-bridges from just four Arduino pins.  Speed is set independently
 * on dedicated PWM pins, one per motor.
 *
 * All four motors share a single internal shift-register state byte
 * (@c _shiftState).  Calling setMotorDirection() for any motor rewrites the
 * entire byte to the hardware.
 *
 * @note The shield is UNO-sized and is wired manually to the Mega 2560 — it
 *       is **not** stacked.  Pin assignments are hard-coded to match the
 *       shield's PCB traces.
 *
 * @note Call begin() once from setup() before using any other method.
 */
class L293DShield {
public:
    /**
     * @brief Initialise all shield pins and disable motor outputs.
     *
     * Sets the shift-register and PWM pins as outputs, clears the shift
     * register to coast all motors, and asserts MOTORENABLE (active LOW) to
     * enable the 74HC595 output drivers.
     *
     * Must be called once from @c setup() before any other method.
     */
    void begin();

    /**
     * @brief Set the direction of one H-bridge.
     *
     * Writes the appropriate A/B bits for the selected motor into the internal
     * shift-register state and immediately clocks the new state out to the
     * hardware.
     *
     * H-bridge truth table:
     *  - @c dir =  1 → A=HIGH, B=LOW  (forward)
     *  - @c dir = -1 → A=LOW,  B=HIGH (backward)
     *  - @c dir =  0 → A=LOW,  B=LOW  (coast / free-wheel)
     *
     * @param motor Motor index, 1–4 (M1=FL, M2=FR, M3=RL, M4=RR).
     *              Out-of-range values are silently ignored.
     * @param dir   Desired direction: +1, 0, or -1.
     */
    void setMotorDirection(uint8_t motor, int8_t dir);

    /**
     * @brief Set the PWM duty cycle (speed magnitude) for one motor.
     *
     * Writes directly to the motor's dedicated PWM pin via @c analogWrite().
     * Direction is controlled separately by setMotorDirection().
     *
     * @param motor Motor index, 1–4.  Out-of-range values are silently ignored.
     * @param pwm   Duty cycle, 0 (stopped) – 255 (full speed).
     */
    void setMotorSpeed(uint8_t motor, uint8_t pwm);

private:
    /** Current 8-bit shadow of the 74HC595 output register. */
    uint8_t _shiftState = 0;

    /**
     * @brief Clock the current @c _shiftState out to the 74HC595.
     *
     * Pulls MOTORLATCH LOW, shifts 8 bits MSB-first on MOTORDATA/MOTORCLK,
     * then pulses MOTORLATCH HIGH to latch the new state.
     */
    void _writeShiftReg();

    // Index 0 = Motor 1 (FL), 1 = Motor 2 (FR), 2 = Motor 3 (RL), 3 = Motor 4 (RR)
    static constexpr uint8_t _pwmPins[4] = {11, 3, 5, 6}; ///< Hardware PWM pins.
    static constexpr uint8_t _bitA[4]    = {2, 1, 5, 0};  ///< Shift-reg bit for H-bridge input A.
    static constexpr uint8_t _bitB[4]    = {3, 4, 7, 6};  ///< Shift-reg bit for H-bridge input B.
};
