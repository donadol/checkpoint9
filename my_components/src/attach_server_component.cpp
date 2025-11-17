#include "my_components/attach_server_component.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions& options)
    : Node("attach_server", options),
      state_(State::COMPLETED),
      service_active_(false),
      attach_to_shelf_(false),
      approach_successful_(false),
      final_approach_started_(false),
      current_yaw_(0.0),
      current_x_(0.0),
      current_y_(0.0),
      final_approach_start_x_(0.0),
      final_approach_start_y_(0.0) {

    RCLCPP_INFO(this->get_logger(), "AttachServer Component Started");

    // Create callback groups for concurrent execution
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create service with its own callback group
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&AttachServer::approach_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        service_callback_group_);

    // Create subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AttachServer::laser_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&AttachServer::odom_callback, this, _1));

    // Create publishers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    elevator_up_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/elevator_up", 10);

    // Create static TF broadcaster
    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create timer for control loop with its own callback group
    timer_ = this->create_wall_timer(
        100ms, std::bind(&AttachServer::control_loop, this),
        timer_callback_group_);
}

void AttachServer::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_scan_ = msg;
}

void AttachServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract yaw from quaternion
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
}

void AttachServer::approach_callback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Approach service called with attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    // Detect shelf legs
    if (!last_laser_scan_) {
        RCLCPP_ERROR(this->get_logger(), "No laser scan data available");
        response->complete = false;
        return;
    }

    auto legs = detect_shelf_legs();

    if (legs.size() < 2) {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not detect both shelf legs (found %zu)", legs.size());
        response->complete = false;
        return;
    }

    // Publish cart_frame TF
    publish_cart_frame(legs);

    // Give TF time to propagate through the system
    RCLCPP_INFO(this->get_logger(), "Waiting for cart_frame TF to propagate...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (!request->attach_to_shelf) {
        // Only publish TF, don't approach
        response->complete = true;
        RCLCPP_INFO(this->get_logger(),
                   "Published cart_frame TF without approaching");
        return;
    }

    // Verify cart_frame is available in TF tree
    if (!tf_buffer_->canTransform("robot_base_link", "cart_frame",
                                  tf2::TimePointZero, std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(),
                    "cart_frame TF not available after waiting");
        response->complete = false;
        return;
    }

    // Perform final approach
    service_active_ = true;
    attach_to_shelf_ = true;
    state_ = State::MOVING_TO_CART;
    final_approach_started_ = false;

    RCLCPP_INFO(this->get_logger(), "Starting approach to shelf");

    // Wait for approach to complete
    while (service_active_ && rclcpp::ok()) {
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Approach to shelf %s",
                approach_successful_ ? "successful" : "failed");

    response->complete = approach_successful_;
}

std::vector<AttachServer::LegDetection> AttachServer::detect_shelf_legs() {
    std::vector<LegDetection> legs;

    if (!last_laser_scan_) {
        return legs;
    }

    const double INTENSITY_THRESHOLD = 8000.0;
    const int MIN_CONSECUTIVE = 3;

    std::vector<int> high_intensity_indices;

    // Find all indices with high intensity
    for (size_t i = 0; i < last_laser_scan_->intensities.size(); i++) {
        if (last_laser_scan_->intensities[i] >= INTENSITY_THRESHOLD) {
            high_intensity_indices.push_back(i);
        }
    }

    if (high_intensity_indices.empty()) {
        RCLCPP_INFO(this->get_logger(), "Detected %zu shelf legs", legs.size());
        return legs;
    }

    // Group consecutive indices into legs
    std::vector<std::vector<int>> leg_groups;
    std::vector<int> current_group = {high_intensity_indices[0]};

    for (size_t i = 1; i < high_intensity_indices.size(); i++) {
        if (high_intensity_indices[i] - high_intensity_indices[i - 1] <= 2) {
            current_group.push_back(high_intensity_indices[i]);
            continue;
        }

        if (current_group.size() >= static_cast<size_t>(MIN_CONSECUTIVE)) {
            leg_groups.push_back(current_group);
        }
        current_group = {high_intensity_indices[i]};
    }

    if (current_group.size() >= static_cast<size_t>(MIN_CONSECUTIVE)) {
        leg_groups.push_back(current_group);
    }

    // Convert groups to leg detections (use center of each group)
    for (const auto& group : leg_groups) {
        int center_idx = group[group.size() / 2];
        LegDetection leg;
        leg.index = center_idx;
        leg.angle = last_laser_scan_->angle_min +
                   center_idx * last_laser_scan_->angle_increment;
        leg.range = last_laser_scan_->ranges[center_idx];
        legs.push_back(leg);
    }

    RCLCPP_INFO(this->get_logger(), "Detected %zu shelf legs", legs.size());
    return legs;
}

void AttachServer::publish_cart_frame(const std::vector<LegDetection>& legs) {
    if (legs.size() < 2) {
        return;
    }

    // Calculate midpoint between the two detected legs in laser frame
    double angle1 = legs[0].angle;
    double angle2 = legs[1].angle;
    double range1 = legs[0].range;
    double range2 = legs[1].range;

    // Convert to Cartesian coordinates (laser frame)
    double x1 = range1 * cos(angle1);
    double y1 = range1 * sin(angle1);
    double x2 = range2 * cos(angle2);
    double y2 = range2 * sin(angle2);

    // Calculate midpoint in laser frame
    double cart_x_laser = (x1 + x2) / 2.0;
    double cart_y_laser = (y1 + y2) / 2.0;

    // Create point in laser frame
    geometry_msgs::msg::PointStamped point_in_laser;
    point_in_laser.header.frame_id = last_laser_scan_->header.frame_id;
    point_in_laser.header.stamp = last_laser_scan_->header.stamp;
    point_in_laser.point.x = cart_x_laser;
    point_in_laser.point.y = cart_y_laser;
    point_in_laser.point.z = 0.0;

    // Transform to odom frame using TF2
    geometry_msgs::msg::PointStamped point_in_odom;
    try {
        point_in_odom = tf_buffer_->transform(point_in_laser, "odom",
                                              tf2::durationFromSec(1.0));
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "TF2 transform failed: %s", ex.what());
        return;
    }

    // Broadcast static TF in odom frame
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "cart_frame";

    transform.transform.translation.x = point_in_odom.point.x;
    transform.transform.translation.y = point_in_odom.point.y;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(transform);
}

void AttachServer::control_loop() {
    if (!service_active_) {
        return;
    }

    auto twist_msg = geometry_msgs::msg::Twist();

    switch (state_) {
        case State::MOVING_TO_CART: {
            // Check if cart_frame is available
            if (!tf_buffer_->canTransform("robot_base_link", "cart_frame",
                                         tf2::TimePointZero)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Waiting for cart_frame TF to become available");
                break;
            }

            // Use TF to get transform from robot_base_link to cart_frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer_->lookupTransform(
                    "robot_base_link", "cart_frame", tf2::TimePointZero);
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "TF lookup failed: %s", ex.what());
                break;
            }

            // Get target position in robot frame
            double target_x = transform_stamped.transform.translation.x;
            double target_y = transform_stamped.transform.translation.y;

            // Calculate distance and angle to target
            double distance = sqrt(target_x * target_x + target_y * target_y);
            double angle_to_target = atan2(target_y, target_x);

            // If very close, move under shelf
            if (distance < 0.10) {  // 10cm
                vel_pub_->publish(twist_msg);  // Stop
                state_ = State::MOVING_UNDER_SHELF;
                RCLCPP_INFO(this->get_logger(),
                           "Reached cart position (%.2fm away), moving under shelf",
                           distance);
                break;
            }

            // If far from target, prioritize alignment first
            if (distance > 0.3 && std::abs(angle_to_target) > 0.1) {
                twist_msg.angular.z = angle_to_target * 1.5;
            } else {
                // Close enough - move forward with gentle correction
                twist_msg.linear.x = std::min(0.3, distance * 0.5);
                twist_msg.angular.z = angle_to_target * 0.5;
            }

            vel_pub_->publish(twist_msg);
            break;
        }

        case State::MOVING_UNDER_SHELF: {
            // Move forward 30cm
            if (!final_approach_started_) {
                final_approach_start_x_ = current_x_;
                final_approach_start_y_ = current_y_;
                final_approach_started_ = true;
            }

            double distance_traveled =
                sqrt(pow(current_x_ - final_approach_start_x_, 2) +
                     pow(current_y_ - final_approach_start_y_, 2));

            if (distance_traveled < 0.30) {  // 30cm
                twist_msg.linear.x = 0.2;
                vel_pub_->publish(twist_msg);
                break;
            }

            vel_pub_->publish(twist_msg);  // Stop
            state_ = State::LIFTING_SHELF;
            RCLCPP_INFO(this->get_logger(), "Under shelf, lifting");
            break;
        }

        case State::LIFTING_SHELF: {
            // Lift the shelf
            auto msg = std_msgs::msg::String();
            elevator_up_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Shelf lifted successfully");
            state_ = State::COMPLETED;
            approach_successful_ = true;
            service_active_ = false;
            break;
        }

        case State::COMPLETED: {
            // Stop the timer to prevent further callbacks
            if (timer_) {
                timer_->cancel();
                timer_.reset();
            }

            RCLCPP_INFO(this->get_logger(),
                       "Shelf attachment complete. Shutting down.");

            // Shutdown ROS2 to terminate the entire container
            rclcpp::shutdown();
            break;
        }
    }
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
