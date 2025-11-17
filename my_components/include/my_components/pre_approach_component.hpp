#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {
   public:
    COMPOSITION_PUBLIC
    explicit PreApproach(const rclcpp::NodeOptions& options);

   private:
    enum class State {
        MOVING_FORWARD,
        ROTATING,
        COMPLETED
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Hardcoded parameters (from original obstacle and degrees parameters)
    const double obstacle_distance_ = 0.3;  // meters
    const int rotation_degrees_ = -90;      // degrees (turn right)

    // State variables
    State state_;
    double front_distance_;
    double current_yaw_;
    double initial_yaw_;
    double target_yaw_;

    // Control parameters
    const double linear_velocity_ = 0.5;     // m/s
    const double angular_velocity_ = 0.8;    // rad/s
    const double angle_tolerance_ = 0.02;    // rad (~1.1 degrees)
};

}  // namespace my_components

#endif  // COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
