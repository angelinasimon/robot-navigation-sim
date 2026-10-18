#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/utils.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    const Controller controller;
    const double pi = robot_nav_controller::kPi;
    // Unicycle model with exact constant-command integration at 50 Hz.
    const Pose starts[] = {{0, 0, 0}, {0, 0, pi}, {2, -3, -pi / 2}, {-2, 1, 9 * pi}};
    const Pose goals[] = {{2, 1, 0}, {-1, -2, 0}, {0, 0, pi}, {3, -1, -pi}};
    for (const Pose& start : starts) {
        for (const Pose& goal : goals) {
            Pose pose = start;
            bool stopped = false;
            for (int step = 0; step < 6000; ++step) {
                const auto cmd = controller.compute_command(pose, goal);
                if (!std::isfinite(cmd.linear_velocity) || !std::isfinite(cmd.angular_velocity)
                    || cmd.linear_velocity < 0 || cmd.linear_velocity > 0.5
                    || std::abs(cmd.angular_velocity) > 1.5) return EXIT_FAILURE;
                if (cmd.linear_velocity == 0 && cmd.angular_velocity == 0) {
                    stopped = true;
                    break;
                }
                const double dt = 0.02;
                const double next = pose.theta + cmd.angular_velocity * dt;
                if (std::abs(cmd.angular_velocity) < 1e-12) {
                    pose.x += cmd.linear_velocity * std::cos(pose.theta) * dt;
                    pose.y += cmd.linear_velocity * std::sin(pose.theta) * dt;
                } else {
                    const double radius = cmd.linear_velocity / cmd.angular_velocity;
                    pose.x += radius * (std::sin(next) - std::sin(pose.theta));
                    pose.y -= radius * (std::cos(next) - std::cos(pose.theta));
                }
                pose.theta = robot_nav_controller::wrap_angle(next);
            }
            const double error = std::hypot(goal.x - pose.x, goal.y - pose.y);
            if (!stopped || error > 0.05) {
                std::cerr << "Drive failed, final distance: " << error << '\n';
                return EXIT_FAILURE;
            }
            const auto settled = controller.compute_command(pose, goal);
            if (settled.linear_velocity != 0 || settled.angular_velocity != 0) return EXIT_FAILURE;
        }
    }
    std::cout << "All 16 closed-loop drives reached the goal and stopped\n";
}
