
#pragma once
 
namespace robot_nav_controller {
 
constexpr double kPi = 3.14159265358979323846;
 
// Returns the angle (radians) equivalent to `angle` in the range (-pi, pi].
// Non-finite inputs (NaN, +/-infinity) are returned unchanged.
double wrap_angle(double angle);
 
// Yaw (rotation about z, radians, in (-pi, pi]) of the quaternion (x, y, z, w).
// Works for any quaternion, normalized or not.
double quaternion_to_yaw(double x, double y, double z, double w);
 
}  // namespace robot_nav_controller
 