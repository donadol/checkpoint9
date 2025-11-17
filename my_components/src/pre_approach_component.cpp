#include "my_components/pre_approach_component.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions& options)
    : Node("pre_approach", options),
      state_(State::MOVING_FORWARD),
      front_distance_(std::numeric_limits<double>::max()),
      current_yaw_(0.0),
      initial_yaw_(0.0),
      target_yaw_(0.0) {

    // Declare parameter for shutdown control
    this->declare_parameter<bool>("shutdown_on_complete", true);
    shutdown_on_complete_ = this->get_parameter("shutdown_on_complete").as_bool();

    RCLCPP_INFO(this->get_logger(), "PreApproach Component Started");
    RCLCPP_INFO(this->get_logger(), "Obstacle distance: %.2f m", obstacle_distance_);
    RCLCPP_INFO(this->get_logger(), "Rotation degrees: %d", rotation_degrees_);
    RCLCPP_INFO(this->get_logger(), "Shutdown on complete: %s",
                shutdown_on_complete_ ? "true" : "false");

    // Create subscription to laser scan
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PreApproach::laser_callback, this, _1));

    // Create subscription to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&PreApproach::odom_callback, this, _1));

    // Create publisher for velocity commands
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Create timer for control loop
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::control_loop, this));
}

void PreApproach::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Get the front distance (center of laser scan)
    int center_idx = msg->ranges.size() / 2;

    // Average a few readings around the center for robustness
    double min_distance = std::numeric_limits<double>::max();
    int range = 10;  // Check +/- 10 readings around center

    for (int i = center_idx - range; i <= center_idx + range; i++) {
        if (i >= 0 && i < static_cast<int>(msg->ranges.size())) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.0) {
                min_distance = std::min(min_distance,
                                       static_cast<double>(msg->ranges[i]));
            }
        }
    }

    front_distance_ = min_distance;
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract yaw from quaternion
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
}

void PreApproach::control_loop() {
    auto twist_msg = geometry_msgs::msg::Twist();

    switch (state_) {
        case State::MOVING_FORWARD: {
            // Check if we've reached the obstacle
            if (front_distance_ <= obstacle_distance_) {
                RCLCPP_INFO(this->get_logger(),
                           "Obstacle detected at %.2f m. Stopping.",
                           front_distance_);

                // Stop the robot
                vel_pub_->publish(twist_msg);

                // Transition to rotating state
                state_ = State::ROTATING;

                // Store initial yaw for rotation tracking
                initial_yaw_ = current_yaw_;
                target_yaw_ = initial_yaw_ + (rotation_degrees_ * M_PI / 180.0);

                // Normalize target yaw to [-pi, pi]
                while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
                while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;

                RCLCPP_INFO(this->get_logger(),
                           "Starting rotation of %d degrees (from %.2f to %.2f rad)",
                           rotation_degrees_, initial_yaw_, target_yaw_);
                break;
            }

            // Move forward
            twist_msg.linear.x = linear_velocity_;
            twist_msg.angular.z = 0.0;
            vel_pub_->publish(twist_msg);
            break;
        }

        case State::ROTATING: {
            // Calculate angle difference (handling wrap-around)
            double angle_diff = target_yaw_ - current_yaw_;

            // Normalize angle difference to [-pi, pi]
            while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

            // Check if rotation is complete (within tolerance)
            if (std::abs(angle_diff) < angle_tolerance_) {
                RCLCPP_INFO(this->get_logger(),
                           "Rotation completed. Final yaw: %.2f rad",
                           current_yaw_);

                // Stop the robot
                vel_pub_->publish(twist_msg);

                // Mark as completed
                state_ = State::COMPLETED;
                break;
            }

            // Continue rotating (proportional control for smoother stopping)
            twist_msg.linear.x = 0.0;

            // Use proportional control near target, full speed otherwise
            if (std::abs(angle_diff) < 0.3) {  // Within ~17 degrees
                twist_msg.angular.z = angle_diff * 2.0;  // Proportional gain
            } else {
                twist_msg.angular.z = (angle_diff > 0) ? angular_velocity_
                                                        : -angular_velocity_;
            }

            vel_pub_->publish(twist_msg);
            break;
        }

        case State::COMPLETED: {
            // Stop the timer to prevent further callbacks
            if (timer_) {
                timer_->cancel();
                timer_.reset();
            }

            if (shutdown_on_complete_) {
                RCLCPP_INFO(this->get_logger(),
                           "Pre-approach maneuver completed. Shutting down.");
                rclcpp::shutdown();
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "Pre-approach maneuver completed.");
            }

            break;
        }
    }
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
