#include "robot_nav_controller/depth_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "robot_nav_controller/utils.hpp"

namespace robot_nav_controller {

namespace {

// Linear-interpolation percentile, matching numpy.percentile's default.
// Reorders `values`; requires it to be non-empty.
double percentile(std::vector<float>& values, double p) {
    const double rank = p * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(rank));
    // nth_element puts the lo-th smallest value at index lo, with everything
    // after it >= it, so the next order statistic is the minimum of the rest.
    // That is O(n), versus O(n log n) for a full sort, at camera frame rate.
    std::nth_element(values.begin(), values.begin() + lo, values.end());
    const double low = values[lo];
    if (lo + 1 >= values.size()) return low;
    const double high = *std::min_element(values.begin() + lo + 1, values.end());
    return low + (rank - static_cast<double>(lo)) * (high - low);
}

void require(bool ok, const std::string& message) {
    if (!ok) throw std::invalid_argument(message);
}

}  // namespace

void validate_depth_params(const DepthParams& p) {
    // tan(horizontal_fov / 2) gives the edge columns' angle, so the field of view
    // must stay below pi (180 degrees), where tan blows up.
    require(std::isfinite(p.horizontal_fov) && p.horizontal_fov > 0.0 && p.horizontal_fov < kPi,
            "horizontal_fov must be in (0, pi), got " + std::to_string(p.horizontal_fov));
    require(std::isfinite(p.inflated_half_width) && p.inflated_half_width > 0.0,
            "inflated_half_width must be finite and > 0, got " +
                std::to_string(p.inflated_half_width));
    require(std::isfinite(p.min_valid_depth) && p.min_valid_depth >= 0.0,
            "min_valid_depth must be finite and >= 0, got " + std::to_string(p.min_valid_depth));
    require(std::isfinite(p.max_valid_depth) && p.max_valid_depth > p.min_valid_depth,
            "max_valid_depth must be finite and > min_valid_depth, got " +
                std::to_string(p.max_valid_depth));
}

double front_depth(const std::vector<float>& depth, int width, int height,
                   const DepthParams& params) {
    if (width < 2 || height < 1 ||
        depth.size() != static_cast<std::size_t>(width) * height) {
        return 0.0;  // Malformed image: fail closed.
    }

    const int row_start = static_cast<int>(height * 0.45);
    const int row_end = static_cast<int>(height * 0.98);
    const double center = (width - 1) / 2.0;

    std::vector<double> tan_angle(width);
    for (int col = 0; col < width; ++col) {
        tan_angle[col] = std::tan((col - center) / center * (params.horizontal_fov / 2.0));
    }

    std::vector<float> in_corridor;
    for (int row = row_start; row < row_end; ++row) {
        for (int col = 0; col < width; ++col) {
            const float d = depth[static_cast<std::size_t>(row) * width + col];
            if (!std::isfinite(d) || d <= params.min_valid_depth ||
                d >= params.max_valid_depth) {
                continue;
            }
            if (std::abs(d * tan_angle[col]) <= params.inflated_half_width) {
                in_corridor.push_back(d);
            }
        }
    }

    if (in_corridor.empty()) return 0.0;  // Fail closed; see header.
    return percentile(in_corridor, 0.05);
}

std::vector<float> decode_16uc1(const std::vector<std::uint16_t>& depth_mm) {
    std::vector<float> metres(depth_mm.size());
    std::transform(depth_mm.begin(), depth_mm.end(), metres.begin(),
                   [](std::uint16_t mm) { return mm / 1000.0f; });
    return metres;
}

}  // namespace robot_nav_controller
