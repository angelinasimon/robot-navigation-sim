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
int main() {
    const double pi = robot_nav_controller::kPi;
    const Controller controller;
    auto cmd = controller.compute_command({0, 0, 0}, {0, 0, pi});
    check(cmd.linear_velocity == 0 && cmd.angular_velocity == 0, "At goal must stop");
    cmd = controller.compute_command({0, 0, 0}, {0.05, 0, 0});
    check(cmd.linear_velocity == 0 && cmd.angular_velocity == 0, "Tolerance boundary must stop");
    cmd = controller.compute_command({0, 0, 0}, {0.2, 0, 0});
    check(near(cmd.linear_velocity, 0.2) && near(cmd.angular_velocity, 0), "Proportional forward motion");
    cmd = controller.compute_command({0, 0, 0}, {10, 0, 0});
    check(near(cmd.linear_velocity, 0.5), "Linear speed cap");
    for (double sign : {-1.0, 1.0}) {
        cmd = controller.compute_command({0, 0, 0}, {0, sign, 0});
        check(cmd.linear_velocity == 0 && near(cmd.angular_velocity, sign * 1.5), "Spin direction and angular cap");
    }
    cmd = controller.compute_command({0, 0, pi - 0.1}, {-std::cos(0.1), -std::sin(0.1), 0});
    check(near(cmd.angular_velocity, 0.4), "Shortest turn across angle wrap");
    cmd = controller.compute_command({0, 0, 8 * pi}, {1, 0, 0});
    check(near(cmd.angular_velocity, 0), "Unwrapped heading");
    cmd = controller.compute_command({0, 0, 0}, {std::cos(0.25), std::sin(0.25), 0});
    check(cmd.linear_velocity > 0 && near(cmd.angular_velocity, 0.5), "Steer while driving");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {nan, inf, -inf}) {
        for (int i = 0; i < 6; ++i) {
            Pose current{0, 0, 0}, goal{1, 0, 0};
            double* fields[] = {&current.x, &current.y, &current.theta, &goal.x, &goal.y, &goal.theta};
            *fields[i] = bad;
            cmd = controller.compute_command(current, goal);
            check(cmd.linear_velocity == 0 && cmd.angular_velocity == 0, "Invalid pose must stop");
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (double bad : {0.0, -1.0, nan, inf}) {
            double args[] = {1, 2, 0.05, 0.5, 0.5, 1.5};
            args[i] = bad;
            bool threw = false;
            try { Controller invalid(args[0], args[1], args[2], args[3], args[4], args[5]); }
            catch (const std::invalid_argument&) { threw = true; }
            check(threw, "Invalid parameters must throw");
        }
    }
    bool threw = false;
    try { Controller invalid(1, 2, 0.05, pi); }
    catch (const std::invalid_argument&) { threw = true; }
    check(threw, "Reject backward-facing drive threshold");
    std::cout << "Controller unit tests passed\n";
}
