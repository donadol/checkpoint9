#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ApproachServiceServer : public rclcpp::Node {
   public:
    ApproachServiceServer() : Node("approach_service_server_node") {
        // Create service
        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::approach_callback, this, _1, _2));

        // Create subscriptions
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::laser_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diffbot_base_controller/odom", 10, std::bind(&ApproachServiceServer::odom_callback, this, _1));

        // Create publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);

        elevator_up_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

        // Create static TF broadcaster
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ApproachServiceServer::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Approach Service Server Started");
    }

   private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_laser_scan_ = msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
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

    void approach_callback(
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
            RCLCPP_ERROR(this->get_logger(), "Could not detect both shelf legs (found %zu)", legs.size());
            response->complete = false;
            return;
        }

        // Publish cart_frame TF
        publish_cart_frame(legs);

        if (!request->attach_to_shelf) {
            // Only publish TF, don't approach
            response->complete = true;
            return;
        }

        // Perform final approach
        service_active_ = true;
        attach_to_shelf_ = true;
        state_ = State::MOVING_TO_CART;

        // Wait for approach to complete
        while (service_active_ && rclcpp::ok()) {
            std::this_thread::sleep_for(100ms);
        }

        response->complete = approach_successful_;
        return;
    }

    struct LegDetection {
        int index;
        double angle;
        double range;
    };

    std::vector<LegDetection> detect_shelf_legs() {
        std::vector<LegDetection> legs;

        if (!last_laser_scan_) {
            return legs;
        }

        const double INTENSITY_THRESHOLD = 8000.0;
        const int MIN_CONSECUTIVE = 3;  // Minimum consecutive readings to consider a leg

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
            leg.angle = last_laser_scan_->angle_min + center_idx * last_laser_scan_->angle_increment;
            leg.range = last_laser_scan_->ranges[center_idx];
            legs.push_back(leg);
        }

        RCLCPP_INFO(this->get_logger(), "Detected %zu shelf legs", legs.size());
        return legs;
    }

    void publish_cart_frame(const std::vector<LegDetection>& legs) {
        if (legs.size() < 2) {
            return;
        }

        // Calculate midpoint between two legs in robot frame
        double angle1 = legs[0].angle;
        double angle2 = legs[1].angle;
        double range1 = legs[0].range;
        double range2 = legs[1].range;

        // Convert to Cartesian coordinates (robot frame)
        double x1 = range1 * cos(angle1);
        double y1 = range1 * sin(angle1);
        double x2 = range2 * cos(angle2);
        double y2 = range2 * sin(angle2);

        // Calculate midpoint in robot frame
        cart_x_ = (x1 + x2) / 2.0;
        cart_y_ = (y1 + y2) / 2.0;

        RCLCPP_INFO(this->get_logger(), "Cart frame at x=%.2f, y=%.2f (robot frame)", cart_x_, cart_y_);

        // Transform cart position to odom frame
        // Robot position in odom frame
        double robot_x_odom = current_x_;
        double robot_y_odom = current_y_;
        double robot_yaw_odom = current_yaw_;

        // Cart position in odom frame
        double cart_x_odom = robot_x_odom + cart_x_ * cos(robot_yaw_odom) - cart_y_ * sin(robot_yaw_odom);
        double cart_y_odom = robot_y_odom + cart_x_ * sin(robot_yaw_odom) + cart_y_ * cos(robot_yaw_odom);

        RCLCPP_INFO(this->get_logger(), "Cart frame in odom at x=%.2f, y=%.2f", cart_x_odom, cart_y_odom);

        // Broadcast static TF in odom frame
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "cart_frame";

        transform.transform.translation.x = cart_x_odom;
        transform.transform.translation.y = cart_y_odom;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(transform);
    }

    void control_loop() {
        if (!service_active_) {
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();

        switch (state_) {
            case State::MOVING_TO_CART: {
                // Calculate distance to cart frame
                double distance = sqrt(cart_x_ * cart_x_ + cart_y_ * cart_y_);
                double angle_to_cart = atan2(cart_y_, cart_x_);

                // First align with cart
                if (std::abs(angle_to_cart) > 0.05) {           // ~3 degrees
                    twist_msg.angular.z = angle_to_cart * 1.5;  // Proportional control
                    vel_pub_->publish(twist_msg);
                    break;
                }

                // Then move forward
                if (distance > 0.05) {                                   // 5cm threshold
                    twist_msg.linear.x = std::min(0.3, distance * 0.5);  // Proportional control
                    vel_pub_->publish(twist_msg);
                    break;
                }

                // Reached cart position
                vel_pub_->publish(twist_msg);  // Stop
                state_ = State::MOVING_UNDER_SHELF;
                RCLCPP_INFO(this->get_logger(), "Reached cart position, moving under shelf");

                break;
            }

            case State::MOVING_UNDER_SHELF: {
                // Move forward 30cm
                if (!final_approach_started_) {
                    final_approach_start_x_ = current_x_;
                    final_approach_start_y_ = current_y_;
                    final_approach_started_ = true;
                }

                double distance_traveled = sqrt(
                    pow(current_x_ - final_approach_start_x_, 2) +
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
                // Do nothing
                break;
            }
        }
    }

    enum class State {
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED
    };

    // ROS2 components
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    State state_ = State::COMPLETED;
    bool service_active_ = false;
    bool attach_to_shelf_ = false;
    bool approach_successful_ = false;
    bool final_approach_started_ = false;

    // Data
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_;
    double current_yaw_ = 0.0;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double cart_x_ = 0.0;
    double cart_y_ = 0.0;
    double final_approach_start_x_ = 0.0;
    double final_approach_start_y_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
