#pragma once

/**
 * @brief Rotational direction as seen from directly above the robot.
 *
 * Used by MecanumDrivetrain::circle() to specify which way the robot orbits
 * around the centre point.
 */
enum class Direction {
    Left,  ///< Counter-clockwise (CCW) orbit when viewed from above.
    Right  ///< Clockwise (CW) orbit when viewed from above.
};
