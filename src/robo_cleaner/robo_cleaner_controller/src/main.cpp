#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "robo_cleaner_controller/robo_cleaner_external_bridge.h"

int main(int argc, char ** argv)
{
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = true;
  rclcpp::init(argc, argv, initOptions);

  auto roboCleanerExternalBridge = std::make_shared<RoboCleanerExternalBridge>();
  // roboCleanerExternalBridge->issueMoveOrder(2);
  // roboCleanerExternalBridge->queryInitialState();
  // roboCleanerExternalBridge->issueMoveOrder(1);

  // std::thread spinThread([&roboCleanerExternalBridge]() {
  //   // roboCleanerExternalBridge->shutdown();
  //   // roboCleanerExternalBridge->shutdown();
  // });

  // runApp(node);
  roboCleanerExternalBridge->init();
  std::cout << "issuing 1" << std::endl;
  roboCleanerExternalBridge->issueMoveOrder(0);
  // std::cout << "issuing 2" << std::endl;
  // roboCleanerExternalBridge->issueMoveOrder(2);
  // std::cout << "issuing 3" << std::endl;
  // roboCleanerExternalBridge->issueMoveOrder(0);

  rclcpp::spin(roboCleanerExternalBridge);
  std::cout << "shutting down for some reason" << std::endl;
  rclcpp::shutdown();
  // spinThread.join();
  return 0;
}

// ros2 action send_goal /move_robot robo_cleaner_interfaces/action/RobotMove "{ robot_move_type: { move_type: 0 } }" --feedback
