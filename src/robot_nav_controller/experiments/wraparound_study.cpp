// Issue #4: does heading noise make the robot dither when the goal is behind it?
//
// The heading error alpha is wrapped to (-pi, pi]. With the goal almost
// directly behind, a little heading noise can move the measured alpha between
// about +pi and -pi, flipping the commanded turn direction. This study measures
// how often that happens and how long the robot takes to commit to a turn.
//
// Noise is added to the heading the controller *sees* (measurement noise). The
// true pose moves with noise-free physics. Each trial uses a fixed seed, and the
// same seeds are reused for every configuration, so rows differ only by the
// configuration, and reruns give identical output on the same compiler and
// standard library.
//
// Prints a table; it is not a pass/fail test. See docs/controller-wraparound-study.md.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/utils.hpp"
#include "unicycle.hpp"

namespace {

constexpr double kPi = robot_nav_controller::kPi;
constexpr double kDt = 0.02;       // 50 Hz, same as the closed-loop test.
constexpr int kMaxSteps = 6000;    // 120 simulated seconds.
constexpr int kTrials = 200;
constexpr double kGoalDistance = 2.0;

struct Trial {
    int reversals = 0;          // Turn-direction flips before committing.
    double commit_time = -1;    // Seconds until true |alpha| < pi/2; -1 if never.
    double goal_time = -1;      // Seconds until GOAL_REACHED; -1 if never.
};

double true_alpha(const Pose& pose, const Pose& goal) {
    return robot_nav_controller::wrap_angle(
        std::atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta);
}

Trial run_trial(const Controller& controller, double goal_offset, double sigma,
                unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma);

    // Robot at the origin facing +x; goal behind it, rotated by goal_offset.
    Pose pose{0, 0, 0};
    const Pose goal{kGoalDistance * std::cos(kPi + goal_offset),
                    kGoalDistance * std::sin(kPi + goal_offset), 0};

    Trial t;
    int last_sign = 0;
    for (int step = 0; step < kMaxSteps; ++step) {
        Pose measured = pose;
        if (sigma > 0) measured.theta += noise(rng);
        const ControlResult res = controller.compute_command(measured, goal);
        if (res.status == ControllerStatus::GOAL_REACHED) {
            t.goal_time = step * kDt;
            break;
        }
        if (t.commit_time < 0) {
            if (std::abs(true_alpha(pose, goal)) < kPi / 2) {
                t.commit_time = step * kDt;
            } else {
                const double w = res.command.angular_velocity;
                const int sign = (w > 0) - (w < 0);
                if (sign != 0 && last_sign != 0 && sign != last_sign) ++t.reversals;
                if (sign != 0) last_sign = sign;
            }
        }
        step_unicycle(pose, res.command, kDt);
    }
    return t;
}

// Nearest-rank percentile, p in (0, 1]. Values of -1 (never happened) sort first,
// so callers check success separately.
double percentile(std::vector<double> v, double p) {
    std::sort(v.begin(), v.end());
    const auto rank = static_cast<std::size_t>(std::ceil(p * v.size()));
    return v[std::max<std::size_t>(rank, 1) - 1];
}

}  // namespace

int main() {
    const Controller controller;  // Default gains, as used everywhere else.
    const double offsets[] = {0.0, 0.01, -0.01, 0.05, -0.05};
    const double sigmas[] = {0.0, 0.01, 0.02, 0.05, 0.1};

    std::printf("Goal %.0f m behind the robot, %d seeded trials per row, %.0f Hz.\n",
                kGoalDistance, kTrials, 1 / kDt);
    std::printf("offset = goal angle from directly behind (rad); sigma = heading noise std dev (rad)\n");
    std::printf("reversals = turn-direction flips before committing (true |alpha| < pi/2)\n\n");
    std::printf("%7s %6s | %-15s | %-21s | %8s | %7s\n", "offset", "sigma",
                "reversals", "commit time (s)", "goal (s)", "reached");
    std::printf("%7s %6s | %4s %4s %5s | %6s %6s %7s | %8s | %7s\n", "", "",
                "med", "p95", "max", "med", "p95", "max", "med", "");

    for (double offset : offsets) {
        for (double sigma : sigmas) {
            std::vector<double> rev, commit, goal;
            int reached = 0, committed = 0;
            for (int i = 0; i < kTrials; ++i) {
                const Trial t = run_trial(controller, offset, sigma, 1000u + i);
                rev.push_back(t.reversals);
                commit.push_back(t.commit_time);
                goal.push_back(t.goal_time);
                reached += t.goal_time >= 0;
                committed += t.commit_time >= 0;
            }
            if (committed < kTrials) {
                std::printf("%7.2f %6.2f | %d of %d trials never committed to a turn\n",
                            offset, sigma, kTrials - committed, kTrials);
                continue;
            }
            std::printf("%7.2f %6.2f | %4.0f %4.0f %5.0f | %6.2f %6.2f %7.2f | %8.2f | %3d/%d\n",
                        offset, sigma, percentile(rev, 0.5), percentile(rev, 0.95),
                        percentile(rev, 1.0), percentile(commit, 0.5),
                        percentile(commit, 0.95), percentile(commit, 1.0),
                        percentile(goal, 0.5), reached, kTrials);
        }
        std::printf("\n");
    }
}
