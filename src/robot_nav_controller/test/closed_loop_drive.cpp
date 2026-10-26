#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
    const Controller controller;
    const double pi = robot_nav_controller::kPi;
    // Unicycle model with exact constant-command integration at 50 Hz.
    const Pose starts[] = {{0, 0, 0}, {0, 0, pi}, {2, -3, -pi / 2}, {-2, 1, 9 * pi}};
    // {-2, 0} and {2, 0} are directly behind the starts at (0, 0) facing 0 and pi,
    // so those runs begin with a heading error of exactly pi (#4).
    const Pose goals[] = {{2, 1, 0}, {-1, -2, 0}, {0, 0, pi}, {3, -1, -pi}, {-2, 0, 0}, {2, 0, 0}};
    int runs = 0;
    const double dt = 0.02;
    // Smoothness metric: largest change in linear velocity between consecutive
    // commands. The first command of each run is skipped: starting from rest is
    // a separate concern from the spin/drive transition this measures.
    double worst_dv = 0, total_time = 0, worst_time = 0;
    for (const Pose& start : starts) {
        for (const Pose& goal : goals) {
            Pose pose = start;
            bool stopped = false;
            double prev_v = 0;
            int step = 0;
            for (; step < 6000; ++step) {
                const auto res = controller.compute_command(pose, goal);
                if (res.status == ControllerStatus::INVALID_INPUT) return EXIT_FAILURE;
                if (step > 0)
                    worst_dv = std::max(worst_dv, std::abs(res.command.linear_velocity - prev_v));
                prev_v = res.command.linear_velocity;
                if (res.status == ControllerStatus::GOAL_REACHED) {
                    if (res.command.linear_velocity != 0 || res.command.angular_velocity != 0)
                        return EXIT_FAILURE;
                    stopped = true;
                    break;
                }
                const Command& cmd = res.command;
                if (!std::isfinite(cmd.linear_velocity) || !std::isfinite(cmd.angular_velocity)
                    || cmd.linear_velocity < 0 || cmd.linear_velocity > 0.5
                    || std::abs(cmd.angular_velocity) > 1.5) return EXIT_FAILURE;
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
            if (settled.status != ControllerStatus::GOAL_REACHED
                || settled.command.linear_velocity != 0
                || settled.command.angular_velocity != 0) return EXIT_FAILURE;
            ++runs;
            total_time += step * dt;
            worst_time = std::max(worst_time, step * dt);
        }
    }
    // Baseline before the taper (#1) was 0.5 m/s. About 0.05 m/s remains, from
    // the final step down from kp * stop_threshold to zero on arrival.
    if (worst_dv > 0.1) {
        std::cerr << "Linear velocity jumped by " << worst_dv << " m/s in one step\n";
        return EXIT_FAILURE;
    }
    std::cout << "All " << runs << " closed-loop drives reached the goal and stopped\n"
              << "  max |dv| per step: " << worst_dv << " m/s\n"
              << "  time to goal: mean " << total_time / runs << " s, max " << worst_time << " s\n";
}
