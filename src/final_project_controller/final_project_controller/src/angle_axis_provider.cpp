#include <iostream>
#include "final_project_controller/angle_axis_provider.h"

template <typename T>
static void waitForClientToBecomeReachable(const std::shared_ptr<T>& service) {
    while (!service->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}


AngleAxisProvider::AngleAxisProvider(std::shared_ptr<rclcpp::Node> node): _node{node} {
    _angleAxisClient = node->create_client<urscript_interfaces::srv::GetEefAngleAxis>("get_eef_angle_axis");
    waitForClientToBecomeReachable(_angleAxisClient);
}

geometry_msgs::msg::Vector3 AngleAxisProvider::getAngleAxis() {
    std::cout << "preparing angle axis request" << std::endl;
    urscript_interfaces::srv::GetEefAngleAxis::Request::SharedPtr request = std::make_shared<urscript_interfaces::srv::GetEefAngleAxis::Request>();
    std::cout << "sending angle axis request" << std::endl;
    auto future = _angleAxisClient->async_send_request(request);
    std::cout << "spinning on angle axis request" << std::endl;
    if (rclcpp::spin_until_future_complete(_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move service with command forward");
    } 
    std::cout << "done spinning" << std::endl;
    auto response = future.get();
    std::cout << "returning" << std::endl;
    return response->angle_axis;
}

