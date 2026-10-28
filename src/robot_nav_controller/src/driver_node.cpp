// ROS 2 wrapper around DriverLogic: drives the vehicle toward the goal the
// planner publishes on /goal, with a depth-camera safety stop.
//
// Every decision lives in DriverLogic and front_depth(), which are unit-tested
// without ROS. This node only converts messages, keeps the latest inputs and
// runs DriverLogic::update() on a 10 Hz timer. Callbacks run one at a time on
// the default single-threaded executor, so the inputs need no locking.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_nav_controller/depth_corridor.hpp"
#include "robot_nav_controller/driver_logic.hpp"
#include "robot_nav_controller/utils.hpp"
#include "robot_nav_interfaces/msg/driver_status.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rnc = robot_nav_controller;
using StatusMsg = robot_nav_interfaces::msg::DriverStatus;
using sensor_msgs::msg::Image;

// The status is sent as its number, so the enum and the message constants must
// agree. Checking at compile time means they can't drift apart unnoticed.
#define SAME_STATUS(name) \
    static_assert(StatusMsg::name == static_cast<std::uint8_t>(rnc::DriverStatus::name), \
                  "DriverStatus.msg and driver_logic.hpp disagree on " #name)
SAME_STATUS(RUNNING);
SAME_STATUS(GOAL_REACHED);
SAME_STATUS(INVALID_INPUT);
SAME_STATUS(SAFETY_STOP);
SAME_STATUS(NO_GOAL);
SAME_STATUS(DEPTH_STALE);
#undef SAME_STATUS

namespace {

const char* status_name(rnc::DriverStatus s) {
    switch (s) {
        case rnc::DriverStatus::RUNNING: return "RUNNING";
        case rnc::DriverStatus::GOAL_REACHED: return "GOAL_REACHED";
        case rnc::DriverStatus::INVALID_INPUT: return "INVALID_INPUT";
        case rnc::DriverStatus::SAFETY_STOP: return "SAFETY_STOP";
        case rnc::DriverStatus::NO_GOAL: return "NO_GOAL";
        case rnc::DriverStatus::DEPTH_STALE: return "DEPTH_STALE";
    }
    return "UNKNOWN";
}

// Copies each row's pixels out of the message, skipping any row padding
// (msg.step can be larger than width * pixel size).
template <typename T>
std::vector<T> copy_pixels(const Image& msg) {
    std::vector<T> pixels(static_cast<std::size_t>(msg.width) * msg.height);
    for (std::size_t row = 0; row < msg.height; ++row) {
        std::memcpy(pixels.data() + row * msg.width, msg.data.data() + row * msg.step,
                    msg.width * sizeof(T));
    }
    return pixels;
}

// Depth in metres, row-major, or nothing if the image can't be read: an
// unsupported encoding, big-endian data (this code assumes a little-endian
// host, like every x86 and ARM machine ROS runs on), or a buffer too small for
// its stated size.
std::optional<std::vector<float>> to_metres(const Image& msg) {
    std::size_t pixel_bytes = 0;
    if (msg.encoding == "32FC1") pixel_bytes = sizeof(float);
    if (msg.encoding == "16UC1") pixel_bytes = sizeof(std::uint16_t);
    if (pixel_bytes == 0 || msg.is_bigendian || msg.step < msg.width * pixel_bytes ||
        msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height) {
        return std::nullopt;
    }
    if (msg.encoding == "32FC1") return copy_pixels<float>(msg);
    return rnc::decode_16uc1(copy_pixels<std::uint16_t>(msg));
}

}  // namespace

class DriverNode : public rclcpp::Node {
 public:
    // Throws if a parameter is invalid (or has the wrong type, e.g. kp:=1
    // instead of kp:=1.0), so a bad configuration stops the node at startup.
    DriverNode()
        : Node("driver_node"), logic_(read_driver_params()), depth_params_(read_depth_params()) {
        const auto depth_topic =
            declare_parameter<std::string>("depth_topic", "/rgbd_camera/depth_image");

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, [this](const geometry_msgs::msg::PoseStamped& msg) { on_goal(msg); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/vehicle/odometry", 10,
            [this](const nav_msgs::msg::Odometry& msg) { on_odometry(msg); });
        depth_sub_ = create_subscription<Image>(depth_topic, 10,
                                                [this](const Image& msg) { on_depth(msg); });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/vehicle/cmd_vel", 10);
        enable_pub_ = create_publisher<std_msgs::msg::Bool>("/model/vehicle/enable", 10);
        status_pub_ = create_publisher<StatusMsg>("/driver/status", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { on_timer(); });

        RCLCPP_INFO(get_logger(), "driver_node started: depth_topic=%s, waiting for /goal",
                    depth_topic.c_str());
    }

 private:
    // Defaults are DriverParams' tuned values; ROS parameters override them.
    // Called from the constructor's initializer list: safe, because the Node
    // base class (which owns the parameters) is already constructed by then.
    rnc::DriverParams read_driver_params() {
        rnc::DriverParams p;
        p.kp = declare_parameter("kp", p.kp);
        p.ka = declare_parameter("ka", p.ka);
        p.goal_tolerance = declare_parameter("goal_tolerance", p.goal_tolerance);
        p.spin_threshold = declare_parameter("spin_threshold", p.spin_threshold);
        p.max_linear_velocity = declare_parameter("max_linear_velocity", p.max_linear_velocity);
        p.max_angular_velocity = declare_parameter("max_angular_velocity", p.max_angular_velocity);
        p.depth_safety_enabled = declare_parameter("depth_safety_enabled", p.depth_safety_enabled);
        p.hard_stop_distance = declare_parameter("hard_stop_distance", p.hard_stop_distance);
        p.depth_timeout = declare_parameter("depth_timeout", p.depth_timeout);
        return p;  // DriverLogic's constructor validates these.
    }

    rnc::DepthParams read_depth_params() {
        rnc::DepthParams p;
        p.horizontal_fov = declare_parameter("horizontal_fov", p.horizontal_fov);
        p.min_valid_depth = declare_parameter("min_valid_depth", p.min_valid_depth);
        p.max_valid_depth = declare_parameter("max_valid_depth", p.max_valid_depth);
        // The protected corridor is the vehicle's width plus a margin on each
        // side, as in robot_driver.py: 0.5 * (1.0 + 0.1) + 0.45 = 1.0 m.
        const double wheel_separation = declare_parameter("wheel_separation", 1.0);
        const double wheel_width = declare_parameter("wheel_width", 0.1);
        const double side_safety_margin = declare_parameter("side_safety_margin", 0.45);
        p.inflated_half_width = 0.5 * (wheel_separation + wheel_width) + side_safety_margin;
        rnc::validate_depth_params(p);
        return p;
    }

    void on_goal(const geometry_msgs::msg::PoseStamped& msg) {
        // The goal is taken to be in the odometry frame; its heading is ignored.
        const rnc::Goal goal{msg.pose.position.x, msg.pose.position.y};
        // The planner republishes its goal at 2 Hz, so only log changes.
        if (!inputs_.goal || inputs_.goal->x != goal.x || inputs_.goal->y != goal.y) {
            RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f)", goal.x, goal.y);
        }
        inputs_.goal = goal;
    }

    void on_odometry(const nav_msgs::msg::Odometry& msg) {
        const auto& p = msg.pose.pose;
        inputs_.pose = Pose{p.position.x, p.position.y,
                            rnc::quaternion_to_yaw(p.orientation.x, p.orientation.y,
                                                   p.orientation.z, p.orientation.w)};
    }

    void on_depth(const Image& msg) {
        const auto metres = to_metres(msg);
        if (!metres) {
            // Skipping the image lets the last reading age out, so the driver
            // reports DEPTH_STALE: honest, and safe (no forward motion).
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Ignoring depth image: encoding=%s bigendian=%d %ux%u step=%u "
                                 "bytes=%zu (need 32FC1 or 16UC1, little-endian)",
                                 msg.encoding.c_str(), msg.is_bigendian, msg.width, msg.height,
                                 msg.step, msg.data.size());
            return;
        }
        inputs_.front_depth = rnc::front_depth(*metres, static_cast<int>(msg.width),
                                               static_cast<int>(msg.height), depth_params_);
        // Stamp with this node's clock, not msg.header.stamp: on_timer() compares
        // against the same clock, so a reading's age is always measured
        // correctly. Gazebo stamps images with sim time, which this node
        // doesn't use, so comparing the two would be meaningless.
        inputs_.depth_stamp = now().seconds();
    }

    void on_timer() {
        const rnc::DriverOutput out = logic_.update(inputs_, now().seconds());

        std_msgs::msg::Bool enable;
        enable.data = true;
        enable_pub_->publish(enable);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = out.command.linear_velocity;
        cmd.angular.z = out.command.angular_velocity;
        cmd_pub_->publish(cmd);

        const double nan = std::numeric_limits<double>::quiet_NaN();
        StatusMsg status;
        status.status = static_cast<std::uint8_t>(out.status);
        status.goal_x = inputs_.goal ? inputs_.goal->x : nan;
        status.goal_y = inputs_.goal ? inputs_.goal->y : nan;
        status.distance_to_goal_m = out.distance_to_goal;
        status_pub_->publish(status);

        // Log transitions, not every tick, so the log shows what happened.
        if (out.status != last_status_) {
            RCLCPP_INFO(get_logger(), "Status %s (distance %.2f m, front depth %.2f m)",
                        status_name(out.status), out.distance_to_goal,
                        inputs_.front_depth.value_or(nan));
            last_status_ = out.status;
        }
    }

    const rnc::DriverLogic logic_;
    const rnc::DepthParams depth_params_;
    rnc::DriverInputs inputs_;
    std::optional<rnc::DriverStatus> last_status_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<StatusMsg>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<DriverNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("driver_node"), "driver_node failed to start: %s",
                     e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
