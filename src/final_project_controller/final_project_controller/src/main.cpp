#include <iostream>
#include <functional>
// #include <fmt/core.h>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
    // TODO topic
#include <std_msgs/msg/string.hpp>
#include "urscript_common/defines/UrScriptTopics.h"
#include "urscript_interfaces/srv/ur_script.hpp"
#include "final_project_controller/misc.h"
#include "final_project_controller/box_position_loader.h"
#include "final_project_controller/joint_state_subscriber.h"
#include "final_project_controller/angle_axis_provider.h"

constexpr auto SLEEP_MILLIS = 3000;

std::string goLinearlyTo(BoxPosition boxPosition) {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", " << boxPosition.z
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end";
  return acc.str();
}

std::string hoverAbove(BoxPosition boxPosition) {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", 0.440"
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end";
  return acc.str();
}

void executeActionsInSuccession(const std::vector<std::function<void()>>& functs) {
  for (const auto& func: functs) {
    func();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
  }
}

void visitBox(const BoxPosition& position, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urScriptPublisher) {
    std::cout << "going to box " << position.str() << std::endl;
    std_msgs::msg::String message{};
    message.data = hoverAbove(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    message.data = goLinearlyTo(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    message.data = hoverAbove(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
}

void visitBoxPositionsInorder(
  const std::vector<BoxPosition>& positions,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& urScriptPublisher
) {
  for (const auto& position: positions) {
    std::cout << "going to box " << position.str() << std::endl;
    std_msgs::msg::String message{};
    message.data = hoverAbove(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    message.data = goLinearlyTo(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    message.data = hoverAbove(position);
    std::cout << message.data << std::endl;
    urScriptPublisher->publish(message); 
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
  }
}


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
    std::cout << "---" << std::endl;
    std::cout << "---" << std::endl;



    using UrScriptSrv = urscript_interfaces::srv::UrScript;
    // TODO topic
    using String = std_msgs::msg::String;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("final_project_controller_node");
    auto urScriptClient = node->create_client<UrScriptSrv>("urscript_service", rmw_qos_profile_services_default);

    auto jointStateSubscriber = JointStateSubscriber(node);
    auto angleAxisProvider = AngleAxisProvider(node);
    // TODO this angle axis thing needs to be sorted out

    // std::cout << "getting angle axis" << std::endl;
    // auto aa = angleAxisProvider.getAngleAxis();
    // std::cout << "x: " << aa.x << ", y: " << aa.y << ", z: " << aa.z << std::endl;
    // std::cout << "angle axis received" << std::endl;


    // TODO topic
    constexpr auto queueSize = 10;
    const rclcpp::QoS qos(queueSize);
    auto urScriptPublisher = node->create_publisher<String>("urscript", qos);

    const auto leanForwardScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/ur_dev/ur_control_gui/resources/scripts/command03.script");
    const auto goToHome = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/go_to_home.urscript");
    // const auto genericMovelScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/generic_movel.urscript");
    // TODO

    // 10, 1, 2, 11, 3, 4, 12, 5, 6, 13

    std::cout << "leaning forward" << std::endl;
    String lfm{};
    lfm.data = leanForwardScript;
    urScriptPublisher->publish(lfm);
    std::cout << "sleeping" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    // TODO topic
    std::cout << "going home" << std::endl;
    String message{};
    message.data = goToHome;
    urScriptPublisher->publish(message);
    std::cout << "sleeping" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    // (0, -0.87, -0.05)
    // (0, -0.755, -0.05)
    // (0, -0.755, 0.065)
    // (0, -0.64, -0.05)
    // (0, -0.64, 0.065)
    // (0, -0.64, 0.18)
    // visitBoxPositionsInorder(boxPositions, urScriptPublisher);
    std::vector<std::function<void()>> functions;
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[10], urScriptPublisher);  });
    // GRIP ^ before the ascent
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67, -0.05, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[1], urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67 + 0.115, -0.05, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[2], urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67 + 0.115, -0.05 + 0.115, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[11], urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67 + 0.115* 2, -0.05, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[3], urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67 + 0.115* 2, -0.05 + 0.115 * 2, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(boxPositions[4], urScriptPublisher);  });
    functions.emplace_back([urScriptPublisher, &boxPositions] { visitBox(BoxPosition(0, -0.67 + 0.115* 2, -0.05 + .115 * 3, boxPositions[10].rx, boxPositions[10].ry, boxPositions[10].rz), urScriptPublisher);  });
    executeActionsInSuccession(functions);

    rclcpp::shutdown();
    return 0;
}

    // while (!urScriptClient->wait_for_service(std::chrono::seconds(1))) {
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //     return EXIT_FAILURE;
    //   }
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    // }
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