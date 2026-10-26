#pragma once

#include <cstdint>
#include <vector>

namespace robot_nav_controller {

// How to read a forward-facing depth image (metres, row-major, width * height).
// Defaults match the parameters robot_driver.py used for this vehicle.
struct DepthParams {
    double horizontal_fov = 1.047;      // Radians across the image width.
    double inflated_half_width = 1.0;   // Half the protected corridor width (m).
    double min_valid_depth = 0.05;      // Readings outside (min, max) are ignored.
    double max_valid_depth = 5.0;
};

// Distance to the nearest obstacle in the robot's path, in metres.
//
// Uses the lower part of the image (rows 45% to 98%), where wheel-height
// obstacles appear, and keeps only pixels whose lateral offset from the camera
// axis (depth * tan(column angle)) falls inside the corridor. Returns the 5th
// percentile of those depths, so a single noisy pixel can't dominate.
//
// Fails closed: if no pixel is valid (all NaN, out of range, or the image is
// malformed), returns 0.0, meaning "something is right in front of us". An
// empty corridor usually means the obstacle is too close for the sensor to
// measure, not that the path is clear.
double front_depth(const std::vector<float>& depth, int width, int height,
                   const DepthParams& params);

// Converts a 16UC1 depth image (millimetres) to metres.
std::vector<float> decode_16uc1(const std::vector<std::uint16_t>& depth_mm);

}  // namespace robot_nav_controller
