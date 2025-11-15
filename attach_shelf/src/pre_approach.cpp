#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PreApproachNode : public rclcpp::Node {
   public:
    PreApproachNode() : Node("pre_approach_node") {
        // Declare parameters
        this->declare_parameter<double>("obstacle", 0.3);  // Default: 30 cm
        this->declare_parameter<int>("degrees", -90);      // Default: -90 degrees (turn right)

        // Get parameters
        obstacle_distance_ = this->get_parameter("obstacle").as_double();
        rotation_degrees_ = this->get_parameter("degrees").as_int();

        // Create subscription to laser scan
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproachNode::laser_callback, this, _1));

        // Create publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);

        // Initialize state
        state_ = State::MOVING_FORWARD;
        rotation_start_time_ = this->now();

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PreApproachNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Pre-Approach Node Started");
        RCLCPP_INFO(this->get_logger(), "Obstacle distance: %.2f m", obstacle_distance_);
        RCLCPP_INFO(this->get_logger(), "Rotation degrees: %d", rotation_degrees_);
    }

   private:
    enum class State {
        MOVING_FORWARD,
        ROTATING,
        COMPLETED
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Get the front distance (center of laser scan)
        int center_idx = msg->ranges.size() / 2;

        // Average a few readings around the center for robustness
        double min_distance = std::numeric_limits<double>::max();
        int range = 10;  // Check +/- 10 readings around center

        for (int i = center_idx - range; i <= center_idx + range; i++) {
            if (i >= 0 && i < static_cast<int>(msg->ranges.size())) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.0) {
                    min_distance = std::min(min_distance, static_cast<double>(msg->ranges[i]));
                }
            }
        }

        front_distance_ = min_distance;
    }

    void control_loop() {
        auto twist_msg = geometry_msgs::msg::Twist();

        switch (state_) {
            case State::MOVING_FORWARD: {
                // Check if we've reached the obstacle
                if (front_distance_ <= obstacle_distance_) {
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f m. Stopping.", front_distance_);

                    // Stop the robot
                    vel_pub_->publish(twist_msg);

                    // Transition to rotating state
                    state_ = State::ROTATING;
                    rotation_start_time_ = this->now();

                    // Calculate rotation duration based on degrees
                    double rotation_radians = rotation_degrees_ * M_PI / 180.0;
                    rotation_duration_ = std::abs(rotation_radians) / angular_velocity_;

                    RCLCPP_INFO(this->get_logger(), "Starting rotation of %d degrees (%.2f seconds)",
                                rotation_degrees_, rotation_duration_);
                    break;
                }

                // Move forward
                twist_msg.linear.x = linear_velocity_;
                twist_msg.angular.z = 0.0;
                vel_pub_->publish(twist_msg);
                break;
            }

            case State::ROTATING: {
                double elapsed_time = (this->now() - rotation_start_time_).seconds();

                if (elapsed_time < rotation_duration_) {
                    // Continue rotating
                    twist_msg.linear.x = 0.0;
                    // Negative degrees = turn right (negative angular velocity)
                    twist_msg.angular.z = (rotation_degrees_ > 0) ? angular_velocity_ : -angular_velocity_;
                    vel_pub_->publish(twist_msg);
                    break;
                }

                RCLCPP_INFO(this->get_logger(), "Rotation completed. Final position reached.");

                // Stop the robot
                vel_pub_->publish(twist_msg);

                // Mark as completed
                state_ = State::COMPLETED;
                break;
            }

            case State::COMPLETED: {
                // Do nothing - maneuver is complete
                break;
            }
        }
    }

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double obstacle_distance_;
    int rotation_degrees_;

    // State variables
    State state_;
    double front_distance_ = std::numeric_limits<double>::max();
    rclcpp::Time rotation_start_time_;
    double rotation_duration_;

    // Control parameters
    const double linear_velocity_ = 0.2;   // m/s
    const double angular_velocity_ = 0.5;  // rad/s
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproachNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
