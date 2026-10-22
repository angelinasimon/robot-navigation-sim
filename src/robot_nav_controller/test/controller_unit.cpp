#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b) { return std::abs(a - b) < 1e-10; }
bool is_stop(const Command& c) { return c.linear_velocity == 0 && c.angular_velocity == 0; }
int main() {
    const double pi = robot_nav_controller::kPi;
    using S = ControllerStatus;
    const Controller controller;
    auto res = controller.compute_command({0, 0, 0}, {0, 0, pi});
    check(is_stop(res.command) && res.status == S::GOAL_REACHED, "At goal must stop");
    res = controller.compute_command({0, 0, 0}, {0.05, 0, 0});
    check(is_stop(res.command) && res.status == S::GOAL_REACHED, "Tolerance boundary must stop");
    res = controller.compute_command({0, 0, 0}, {0.2, 0, 0});
    check(near(res.command.linear_velocity, 0.2) && near(res.command.angular_velocity, 0)
          && res.status == S::RUNNING, "Proportional forward motion");
    res = controller.compute_command({0, 0, 0}, {10, 0, 0});
    check(near(res.command.linear_velocity, 0.5) && res.status == S::RUNNING, "Linear speed cap");
    for (double sign : {-1.0, 1.0}) {
        res = controller.compute_command({0, 0, 0}, {0, sign, 0});
        check(res.command.linear_velocity == 0 && near(res.command.angular_velocity, sign * 1.5)
              && res.status == S::RUNNING, "Spin direction and angular cap");
    }
    res = controller.compute_command({0, 0, pi - 0.1}, {-std::cos(0.1), -std::sin(0.1), 0});
    check(near(res.command.angular_velocity, 0.4), "Shortest turn across angle wrap");
    res = controller.compute_command({0, 0, 8 * pi}, {1, 0, 0});
    check(near(res.command.angular_velocity, 0), "Unwrapped heading");
    res = controller.compute_command({0, 0, 0}, {std::cos(0.25), std::sin(0.25), 0});
    check(res.command.linear_velocity > 0 && near(res.command.angular_velocity, 0.5)
          && res.status == S::RUNNING, "Steer while driving");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {nan, inf, -inf}) {
        for (int i = 0; i < 6; ++i) {
            Pose current{0, 0, 0}, goal{1, 0, 0};
            double* fields[] = {&current.x, &current.y, &current.theta, &goal.x, &goal.y, &goal.theta};
            *fields[i] = bad;
            res = controller.compute_command(current, goal);
            check(is_stop(res.command) && res.status == S::INVALID_INPUT, "Invalid pose must stop");
        }
    }
    res = controller.compute_command({0, 0, nan}, {0, 0, 0});
    check(is_stop(res.command) && res.status == S::INVALID_INPUT, "Invalid input outranks goal reached");
    for (int i = 0; i < 6; ++i) {
        const bool is_stop_threshold = (i == 2);
        for (double bad : {0.0, -1e-9, -1.0, nan, inf, -inf}) {
            if (bad == 0.0 && is_stop_threshold) continue;
            double args[] = {1, 2, 0.05, 0.5, 0.5, 1.5};
            args[i] = bad;
            bool threw = false;
            try { Controller invalid(args[0], args[1], args[2], args[3], args[4], args[5]); }
            catch (const std::invalid_argument&) { threw = true; }
            check(threw, "Invalid parameters must throw");
        }
    }
    for (double bad : {pi / 2 + 1e-9, pi, 4.0}) {
        bool threw = false;
        try { Controller invalid(1, 2, 0.05, bad); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "Reject spin threshold above pi/2");
    }
    try {
        Controller(1, 2, 0.0);
        Controller(1, 2, 0.05, pi / 2);
        Controller(1e-9, 1e-9, 0.05, 0.5, 1e-9, 1e-9);
    } catch (const std::invalid_argument&) {
        check(false, "Valid boundary parameters must not throw");
    }
    res = Controller(1, 2, 0.05, pi / 2).compute_command({0, 0, 0}, {-1, 0, 0});
    check(res.command.linear_velocity == 0 && res.status == S::RUNNING,
          "Goal behind must turn in place at the widest spin threshold");
    std::cout << "Controller unit tests passed\n";
}
