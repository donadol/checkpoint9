#include "my_components/attach_client_component.hpp"

#include <chrono>
#include <memory>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GoToLoading = attach_shelf::srv::GoToLoading;
using ServiceResponseFuture = rclcpp::Client<GoToLoading>::SharedFuture;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions& options)
    : Node("attach_client", options), service_called_(false) {
    RCLCPP_INFO(this->get_logger(), "AttachClient Component Started");

    // Create service client
    client_ = create_client<GoToLoading>("/approach_shelf");

    // Create timer to call service once after a delay
    timer_ = create_wall_timer(5s, std::bind(&AttachClient::call_service, this));
}

void AttachClient::call_service() {
    // Only call service once
    if (service_called_) {
        return;
    }

    if (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_WARN(this->get_logger(),
                    "/approach_shelf service not available, waiting...");
        return;
    }

    // Mark as called
    service_called_ = true;

    // Stop the timer
    timer_->cancel();

    // Create request with attach_to_shelf = true
    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = true;

    RCLCPP_INFO(this->get_logger(),
                "Calling /approach_shelf service with attach_to_shelf=true");

    // Async send request
    auto response_received_callback = [this](ServiceResponseFuture future) {
        try {
            auto response = future.get();
            if (response->complete) {
                RCLCPP_INFO(this->get_logger(),
                            "Service call successful! Robot attached to shelf. Shutting down.");
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Service call failed. Robot could not attach to shelf. Shutting down.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Service call exception: %s. Shutting down.", e.what());
        }

        // Shutdown ROS2 to terminate the program
        rclcpp::shutdown();
    };

    client_->async_send_request(request, response_received_callback);
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
