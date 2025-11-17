#ifndef COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_

#include <memory>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace my_components {

class AttachServer : public rclcpp::Node {
   public:
    COMPOSITION_PUBLIC
    explicit AttachServer(const rclcpp::NodeOptions& options);

   private:
    enum class State {
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED
    };

    struct LegDetection {
        int index;
        double angle;
        double range;
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void approach_callback(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);
    std::vector<LegDetection> detect_shelf_legs();
    void publish_cart_frame(const std::vector<LegDetection>& legs);
    void control_loop();

    // ROS2 components
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    State state_;
    bool service_active_;
    bool attach_to_shelf_;
    bool approach_successful_;
    bool final_approach_started_;

    // Data
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_;
    double current_yaw_;
    double current_x_;
    double current_y_;
    double final_approach_start_x_;
    double final_approach_start_y_;
};

}  // namespace my_components

#endif  // COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_
