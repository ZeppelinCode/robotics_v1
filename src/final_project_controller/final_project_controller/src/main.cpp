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

int isGripperEnabled{};
double OFFSET_SIZE_M{};
int64_t SLEEP_MILLIS{};
double HOVER_ABOVE_BOX{};
std::string HOVER_ABOVE_PICKING_CLEARANCE{};
double TARGET_HEIGHT{};
// TODO gripper should be a global here

void loadConfig() {
  auto sConfig = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/config/config.txt");
  auto lines = misc::splitStringBy(sConfig, '\n');
  isGripperEnabled = std::stoi(lines[0]);
  OFFSET_SIZE_M = std::stod(lines[1]);
  SLEEP_MILLIS = std::stoi(lines[2]);
  HOVER_ABOVE_BOX = std::stod(lines[3]);
  HOVER_ABOVE_PICKING_CLEARANCE = lines[4];
  TARGET_HEIGHT = std::stod(lines[5]);

  std::cout << "config read:" << std::endl;
  std::cout << "isGripperEnabled: " << isGripperEnabled << std::endl;
  std::cout << "offset in meters: " << OFFSET_SIZE_M << std::endl;
  std::cout << "sleep millis: " << SLEEP_MILLIS << std::endl;
  std::cout << "hover above box meters: " << HOVER_ABOVE_BOX << std::endl;
  std::cout << "hover above picking clearance: " << HOVER_ABOVE_PICKING_CLEARANCE << std::endl;
  std::cout << "target height: " << TARGET_HEIGHT << std::endl;
}

void openGripper(const std::string& openGripperScript, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urScriptPublisher) {
  std::cout << "openning gripper log: " << isGripperEnabled << std::endl;
  if (!isGripperEnabled) {
    std::cout << "openning gripper for real: " << isGripperEnabled << std::endl;
    return;
  }
  std_msgs::msg::String message{};
  message.data = openGripperScript;
  std::cout << message.data << std::endl;
  urScriptPublisher->publish(message); 
}

void closeGripper(const std::string& closeGripperScript, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urScriptPublisher) {
  std::cout << "closing gripper log: " << isGripperEnabled << std::endl;
  if (!isGripperEnabled) {
    std::cout << "closing gripper for real: " << isGripperEnabled << std::endl;
    return;
  }
  std_msgs::msg::String message{};
  message.data = closeGripperScript;
  std::cout << message.data << std::endl;
  urScriptPublisher->publish(message); 
}

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
  acc << "end\n\n\n";
  return acc.str();
}

std::string hoverAbove(BoxPosition boxPosition) {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", 0.500"//boxPosition.z + HOVER_ABOVE_BOX // 0.15
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end\n\n\n";
  return acc.str();
}

std::string hoverAboveForPicking(BoxPosition boxPosition) {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", " << HOVER_ABOVE_PICKING_CLEARANCE // 0.25
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end\n\n\n";
  return acc.str();
}

void executeActionsInSuccession(const std::vector<std::function<void()>>& functs) {
  int i = 1;
  for (const auto& func: functs) {
    func();
    std::cout << "action #" << i++ << std::endl;
  }
}

void executeServiceRequest(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client,
  const std::string& scriptToExecute
) {
  using UrScriptSrv = urscript_interfaces::srv::UrScript;
  auto request = std::make_shared<UrScriptSrv::Request>();
  request->data = scriptToExecute;
  auto result = client->async_send_request(request);
  std::cout << "request sent, waiting" << std::endl;
  std::cout << scriptToExecute << std::endl;
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move service with command forward");
  }
  auto response = result.get();
  std::cout << "success: " << response->success << ", error: " << response->error_reason << std::endl;
  std::cout << "----" << std::endl;
}

void grabBox(
  const BoxPosition& position, 
  [[maybe_unused]]const std::string& closeGripperScript, 
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client 
) {
    executeServiceRequest(node, client, hoverAboveForPicking(position));
    executeServiceRequest(node, client, goLinearlyTo(position));

    // TODO close gripper
    // closeGripper(closeGripperScript, urScriptPublisher);

    executeServiceRequest(node, client, hoverAboveForPicking(position));
}

void placeBox(
  const BoxPosition& position, 
  [[maybe_unused]]const std::string& openGripperScript, 
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client 
) {
  executeServiceRequest(node, client, hoverAbove(position));
  executeServiceRequest(node, client, goLinearlyTo(position));
  // TODO open gripper
  // openGripper(openGripperScript, urScriptPublisher);
  executeServiceRequest(node, client, hoverAbove(position));
}

double offsetBy(int n) {
  return OFFSET_SIZE_M * n; // 0.115
}

int main(int32_t argc, char *argv[]) {
    rclcpp::InitOptions initOptions;
    initOptions.shutdown_on_sigint = true;
    rclcpp::init(argc, argv, initOptions);

    loadConfig();

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
    // // TODO topic
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("final_project_controller_node");
    auto urScriptClient = node->create_client<UrScriptSrv>("urscript_service", rmw_qos_profile_services_default);



    // auto jointStateSubscriber = JointStateSubscriber(node);
    // auto angleAxisProvider = AngleAxisProvider(node);
    // // TODO this angle axis thing needs to be sorted out

    // // std::cout << "getting angle axis" << std::endl;
    // // auto aa = angleAxisProvider.getAngleAxis();
    // // std::cout << "x: " << aa.x << ", y: " << aa.y << ", z: " << aa.z << std::endl;
    // // std::cout << "angle axis received" << std::endl;


    // // TODO topic
    // constexpr auto queueSize = 10;
    // const rclcpp::QoS qos(queueSize);
    // auto urScriptPublisher = node->create_publisher<String>("urscript", qos);

    const auto leanForwardScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/ur_dev/ur_control_gui/resources/scripts/command03.script");
    const auto goToHome = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/go_to_home.urscript");
    const auto openGripperScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/open_gripper.urscript");
    const auto closeGripperScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/close_gripper.urscript");
    // // const auto genericMovelScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/generic_movel.urscript");
    // // TODO

    // executeServiceRequest(node, urScriptClient, leanForwardScript);
    executeServiceRequest(node, urScriptClient, goToHome);
    //         while (!urScriptClient->wait_for_service(std::chrono::seconds(1))) {
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //     return EXIT_FAILURE;
    //   }
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    // }
    // std::cout << "service is up" << std::endl;
    // std::cout << "building request" << std::endl;

    // for (const auto& boxPosition : boxPositions) {
    //   auto request = std::make_shared<UrScriptSrv::Request>();
    //   request->data = goLinearlyTo(boxPosition);
    //   auto result = urScriptClient->async_send_request(request);
    //   std::cout << "request sent, waiting " << boxPosition.str() << std::endl;
    //   std::cout << request->data << std::endl;
    //   if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    //       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move service with command forward");
    //   }
    //   auto response = result.get();
    //   std::cout << "success: " << response->success << ", error: " << response->error_reason << std::endl;
    // }
   


    // // TODO topic
    // std::cout << "going home" << std::endl;
    // String message{};
    // message.data = goToHome;
    // urScriptPublisher->publish(message);
    // std::cout << "sleeping" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
    // std::cout << "going home 2" << std::endl;
    // String message2{};
    // message2.data = goToHome;
    // urScriptPublisher->publish(message2);
    // std::cout << "sleeping" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    // openGripper(openGripperScript, urScriptPublisher);
    // std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));

    // // (0, -0.87, -0.05)
    // // (0, -0.755, -0.05)
    // // (0, -0.755, 0.065)
    // // (0, -0.64, -0.05)
    // // (0, -0.64, 0.065)
    // // (0, -0.64, 0.18)
    // // 10, 1, 2, 11, 3, 4, 12, 5, 13, 7
    std::vector<std::function<void()>> functions;
    // First stair
    functions.emplace_back([&] { grabBox(boxPositions[9], closeGripperScript, node, urScriptClient);  }); // Grip before ascent
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67, TARGET_HEIGHT, 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    // Second stair
    functions.emplace_back([&] { grabBox(boxPositions[0], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(1), TARGET_HEIGHT, 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[1], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(1), TARGET_HEIGHT + 0.115, 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    // Third stair
    functions.emplace_back([&] { grabBox(boxPositions[10], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(2), TARGET_HEIGHT, 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[2], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(2), TARGET_HEIGHT + offsetBy(1), 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[3], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(2), TARGET_HEIGHT + offsetBy(2), 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    // Fourth stair
    functions.emplace_back([&] { grabBox(boxPositions[11], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(3), TARGET_HEIGHT, 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[4], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(3), TARGET_HEIGHT + offsetBy(1), 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[12], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(3), TARGET_HEIGHT + offsetBy(2), 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { grabBox(boxPositions[6], closeGripperScript, node, urScriptClient);  });
    functions.emplace_back([&] { placeBox(BoxPosition(0, -0.67 + offsetBy(3), TARGET_HEIGHT + offsetBy(3), 0, 3.14, 0), openGripperScript, node, urScriptClient);  });
    executeActionsInSuccession(functions);

    rclcpp::shutdown();
    return 0;
}

