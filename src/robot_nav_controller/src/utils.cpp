
#include "robot_nav_controller/utils.hpp"
 
#include <cmath>
 
namespace robot_nav_controller {
 
double wrap_angle(double angle) {
  if (!std::isfinite(angle)) {
    return angle;
  }
 
  // Remove whole turns. Result is in (-2*pi, 2*pi), same sign as the input.
  angle = std::fmod(angle, 2.0 * kPi);
 
  // Shift once into (-pi, pi].
  if (angle <= -kPi) {
    angle += 2.0 * kPi;
  } else if (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  return angle;
}
 
double quaternion_to_yaw(double x, double y, double z, double w) {
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = w * w + x * x - y * y - z * z;
  return wrap_angle(std::atan2(siny, cosy));
}
}  // namespace robot_nav_controller
 