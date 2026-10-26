#include "robot_nav_controller/driver_logic.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "robot_nav_controller/utils.hpp"

using robot_nav_controller::DriverInputs;
using robot_nav_controller::DriverLogic;
using robot_nav_controller::DriverParams;
using robot_nav_controller::DriverStatus;
using robot_nav_controller::Goal;

namespace {

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

// Robot at the origin facing +x, goal 5 m ahead, clear depth measured now.
DriverInputs driving() {
    DriverInputs in;
    in.pose = Pose{0, 0, 0};
    in.goal = Goal{5, 0};
    in.front_depth = 4.0;
    in.depth_stamp = 10.0;
    return in;
}
constexpr double kNow = 10.0;

bool throws(const DriverParams& p) {
    try { DriverLogic d(p); } catch (const std::invalid_argument&) { return true; }
    return false;
}

}  // namespace

int main() {
    const double pi = robot_nav_controller::kPi;
    const DriverLogic driver;

    // Missing inputs: never move.
    DriverInputs in = driving();
    in.goal.reset();
    auto out = driver.update(in, kNow);
    check(out.status == DriverStatus::NO_GOAL && out.command.linear_velocity == 0
          && out.command.angular_velocity == 0 && std::isnan(out.distance_to_goal),
          "No goal stays still");
    in = driving();
    in.pose.reset();
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::INVALID_INPUT && out.command.linear_velocity == 0
          && out.command.angular_velocity == 0, "No odometry stays still");
    in = driving();
    in.goal = Goal{std::numeric_limits<double>::quiet_NaN(), 0};
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::INVALID_INPUT && out.command.linear_velocity == 0,
          "Non-finite goal stays still");

    // Normal driving uses the vehicle's tuned limits.
    out = driver.update(driving(), kNow);
    check(out.status == DriverStatus::RUNNING && near(out.command.linear_velocity, 0.35)
          && near(out.command.angular_velocity, 0) && near(out.distance_to_goal, 5),
          "Drives at the tuned speed limit toward a far goal");
    in = driving();
    in.goal = Goal{-5, 0};
    out = driver.update(in, kNow);
    check(out.command.linear_velocity == 0 && near(out.command.angular_velocity, 0.32),
          "Goal behind turns in place at the tuned turn limit");
    in = driving();
    in.pose = Pose{4.8, 0, 0};
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::GOAL_REACHED && out.command.linear_velocity == 0
          && out.command.angular_velocity == 0, "Within goal_tolerance reports GOAL_REACHED");

    // Depth safety stop.
    in = driving();
    in.front_depth = 0.5;
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::SAFETY_STOP && out.command.linear_velocity == 0,
          "Obstacle inside hard_stop_distance stops forward motion");
    in.goal = Goal{5, 1};  // Goal off to the left: the robot should still turn.
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::SAFETY_STOP && out.command.linear_velocity == 0
          && out.command.angular_velocity > 0, "Safety stop still allows turning");
    in = driving();
    in.front_depth = 0.0;  // front_depth()'s fail-closed value for an empty corridor.
    check(driver.update(in, kNow).status == DriverStatus::SAFETY_STOP,
          "Fail-closed depth reading stops the robot");
    in.front_depth = 0.85;
    check(driver.update(in, kNow).status == DriverStatus::RUNNING,
          "Exactly hard_stop_distance does not stop (strict <, as in robot_driver.py)");

    // Stale or missing depth fails closed: no driving blind (unlike robot_driver.py).
    in = driving();
    in.depth_stamp = kNow - 0.8;  // Clear reading, but too old to trust.
    in.goal = Goal{5, 1};
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::DEPTH_STALE && out.command.linear_velocity == 0
          && out.command.angular_velocity > 0, "Stale depth stops forward motion, turning allowed");
    in.depth_stamp = kNow - 0.7;
    check(driver.update(in, kNow).status == DriverStatus::RUNNING,
          "Depth just under the timeout still counts as fresh");
    in = driving();
    in.front_depth.reset();
    out = driver.update(in, kNow);
    check(out.status == DriverStatus::DEPTH_STALE && out.command.linear_velocity == 0,
          "No depth reading yet stops forward motion");

    // Safety stop never overrides the terminal statuses.
    in = driving();
    in.pose = Pose{4.8, 0, 0};
    in.front_depth = 0.1;
    check(driver.update(in, kNow).status == DriverStatus::GOAL_REACHED,
          "GOAL_REACHED wins over a close obstacle");

    DriverParams no_safety;
    no_safety.depth_safety_enabled = false;
    in = driving();
    in.front_depth = 0.1;
    check(DriverLogic(no_safety).update(in, kNow).status == DriverStatus::RUNNING,
          "depth_safety_enabled=false disables the stop");
    in.front_depth.reset();
    check(DriverLogic(no_safety).update(in, kNow).status == DriverStatus::RUNNING,
          "depth_safety_enabled=false does not require depth");
    in = driving();
    in.pose = Pose{4.8, 0, 0};
    in.front_depth.reset();
    check(driver.update(in, kNow).status == DriverStatus::GOAL_REACHED,
          "GOAL_REACHED wins over missing depth");

    // Parameter validation, including the Controller's own checks.
    DriverParams bad;
    bad.hard_stop_distance = -1;
    check(throws(bad), "Negative hard_stop_distance throws");
    bad = DriverParams{};
    bad.depth_timeout = 0;
    check(throws(bad), "Zero depth_timeout throws");
    bad = DriverParams{};
    bad.max_linear_velocity = 0;
    check(throws(bad), "Controller parameter errors propagate");
    bad = DriverParams{};
    bad.spin_threshold = pi;
    check(throws(bad), "Controller spin_threshold limit propagates");

    std::cout << "Driver logic unit tests passed\n";
}
