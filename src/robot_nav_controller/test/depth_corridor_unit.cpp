#include "robot_nav_controller/depth_corridor.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "robot_nav_controller/utils.hpp"

using robot_nav_controller::DepthParams;
using robot_nav_controller::decode_16uc1;
using robot_nav_controller::front_depth;

namespace {

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b) { return std::abs(a - b) < 1e-6; }

// 9 x 20 image. Rows 9-18 are analysed (45%-98% of 20). With the default 60 deg
// field of view and a 1.0 m half-width, at 4 m depth only columns 3-5 are in
// the corridor (|4 * tan(angle)| <= 1); columns 0-2 and 6-8 are outside it.
constexpr int kW = 9, kH = 20;
std::vector<float> image(float fill) { return std::vector<float>(kW * kH, fill); }
bool throws(const DepthParams& p) {
    try { robot_nav_controller::validate_depth_params(p); } catch (const std::invalid_argument&) { return true; }
    return false;
}

void set_column(std::vector<float>& img, int col, int row_from, int row_to, float d) {
    for (int r = row_from; r < row_to; ++r) img[r * kW + col] = d;
}

}  // namespace

int main() {
    const DepthParams p;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    check(near(front_depth(image(4.0f), kW, kH, p), 4.0), "Uniform depth is reported");

    // Fail closed: no valid pixels means "blocked", never "clear". This was the
    // bug that let the old driver drive into a person (robot_driver.py).
    check(front_depth(image(nan), kW, kH, p) == 0.0, "All NaN fails closed");
    check(front_depth(image(0.01f), kW, kH, p) == 0.0, "Too close to measure fails closed");
    check(front_depth(image(9.0f), kW, kH, p) == 0.0, "Beyond max range fails closed");
    check(front_depth(image(4.0f), kW, kH - 1, p) == 0.0, "Size mismatch fails closed");
    check(front_depth(std::vector<float>(kH, 4.0f), 1, kH, p) == 0.0, "Degenerate width fails closed");

    // Corridor geometry: the same obstacle counts straight ahead but not off to the side.
    auto ahead = image(4.0f);
    set_column(ahead, 4, 9, 19, 2.5f);
    check(near(front_depth(ahead, kW, kH, p), 2.5), "Obstacle straight ahead is in the corridor");
    auto beside = image(4.0f);
    set_column(beside, 0, 9, 19, 2.5f);  // 2.5 * tan(30 deg) = 1.44 m to the side.
    check(near(front_depth(beside, kW, kH, p), 4.0), "Obstacle outside the corridor is ignored");

    // Only the lower part of the image is used.
    auto high = image(4.0f);
    set_column(high, 4, 0, 9, 1.0f);
    check(near(front_depth(high, kW, kH, p), 4.0), "Rows above 45% are ignored");

    // 5th percentile: one noisy pixel among ~30 in-corridor pixels can't dominate.
    auto noisy = image(4.0f);
    noisy[12 * kW + 4] = 1.0f;
    check(near(front_depth(noisy, kW, kH, p), 4.0), "Single noisy pixel is ignored");

    // Percentile interpolation matches numpy.percentile: with exactly four valid
    // pixels (1, 2, 3, 4 m; everything else NaN), the 5th percentile is at rank
    // 0.05 * (4 - 1) = 0.15, which interpolates to 1.15 m.
    DepthParams wide = p;
    wide.inflated_half_width = 100.0;  // Every column in the corridor.
    std::vector<float> ramp = image(nan);
    ramp[9 * kW + 0] = 1.0f; ramp[9 * kW + 1] = 2.0f; ramp[9 * kW + 2] = 3.0f; ramp[9 * kW + 3] = 4.0f;
    check(near(front_depth(ramp, kW, kH, wide), 1.15), "Percentile interpolates like numpy");

    const auto metres = decode_16uc1({0, 1500, 65535});
    // Compare as float: 65.535 has no exact float representation.
    check(metres.size() == 3 && metres[0] == 0.0f && metres[1] == 1.5f && metres[2] == 65.535f,
          "16UC1 millimetres decode to metres");

    // Parameter validation: each rule rejects a bad value; the defaults pass.
    check(!throws(p), "Default parameters are valid");
    DepthParams bad = p;
    bad.horizontal_fov = 0.0;
    check(throws(bad), "Zero field of view throws");
    bad = p;
    bad.horizontal_fov = robot_nav_controller::kPi;
    check(throws(bad), "Field of view of pi throws (tan of the edge angle blows up)");
    bad = p;
    bad.inflated_half_width = 0.0;
    check(throws(bad), "Zero corridor half-width throws");
    bad = p;
    bad.min_valid_depth = -0.1;
    check(throws(bad), "Negative min_valid_depth throws");
    bad = p;
    bad.min_valid_depth = 5.0;  // Equal to max: no depth could ever be valid.
    check(throws(bad), "min_valid_depth >= max_valid_depth throws");
    bad = p;
    bad.max_valid_depth = std::numeric_limits<double>::infinity();
    check(throws(bad), "Infinite max_valid_depth throws");

    std::cout << "Depth corridor unit tests passed\n";
}
