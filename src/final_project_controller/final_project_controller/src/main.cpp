#include <iostream>
#include <rclcpp/rclcpp.hpp>
    // TODO topic
#include <std_msgs/msg/string.hpp>
#include "urscript_common/defines/UrScriptTopics.h"
#include "urscript_interfaces/srv/ur_script.hpp"
#include "final_project_controller/misc.h"
#include "final_project_controller/box_position_loader.h"
#include "final_project_controller/joint_state_subscriber.h"


int main(int32_t argc, char *argv[]) {
    rclcpp::InitOptions initOptions;
    initOptions.shutdown_on_sigint = true;
    rclcpp::init(argc, argv, initOptions);

    std::cout << "loading box positions" << std::endl;
    auto boxPositions = bpl::loadBoxPositions();
    for (const auto& bp: boxPositions) {
      std::cout << bp.str() << std::endl;
    }
    std::cout << "done loading box positions" << std::endl;
    std::cout << "---" << std::endl;



    using UrScriptSrv = urscript_interfaces::srv::UrScript;
    // TODO topic
    using String = std_msgs::msg::String;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("final_project_controller_node");
    auto urScriptClient = node->create_client<UrScriptSrv>("urscript_service", rmw_qos_profile_services_default);

    auto jointStateSubscriber = JointStateSubscriber(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "getting current joint state" << std::endl;
    std::cout << jointStateSubscriber.getCurrentJointState().str() << std::endl;
    std::cout << "done getting current joint state" << std::endl;
    std::cout << "---" << std::endl;

    // TODO topic
    constexpr auto queueSize = 10;
    const rclcpp::QoS qos(queueSize);
    auto urScriptPublisher = node->create_publisher<String>("urscript", qos);

    const auto leanForwardScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/ur_dev/ur_control_gui/resources/scripts/command03.script");

    while (!urScriptClient->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return EXIT_FAILURE;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // 10, 1, 2, 11, 3, 4, 12, 5, 6, 13

    // TODO topic
    std::cout << "sending message" << std::endl;
    String message{};
    message.data = leanForwardScript;
    urScriptPublisher->publish(message);
    std::cout << "sent" << std::endl;

    // std::cout << "service is up" << std::endl;

    // std::cout << "building request" << std::endl;
    // auto request = std::make_shared<UrScriptSrv::Request>();
    // request->data = leanForwardScript;
    // std::cout << "sending request" << std::endl;
    // std::cout << request->data << std::endl;
    // std::cout <<  "---" << std::endl;
    // auto result = urScriptClient->async_send_request(request);
    // std::cout << "request sent, waiting" << std::endl;
    // if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move service with command forward");
    // }
    // auto response = result.get();
    // std::cout << "success: " << response->success << ", error: " << response->error_reason << std::endl;

    rclcpp::shutdown();
    return 0;
}