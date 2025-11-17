#ifndef COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
   public:
    COMPOSITION_PUBLIC
    explicit AttachClient(const rclcpp::NodeOptions& options);

   protected:
    void call_service();

   private:
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool service_called_;
};

}  // namespace my_components

#endif  // COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_
