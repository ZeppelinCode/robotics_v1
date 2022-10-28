#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include "robo_cleaner_controller/robo_cleaner_external_bridge.h"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
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
  // std::thread t{[&]() {
  //   roboCleanerExternalBridge->init();
  // }};

  roboCleanerExternalBridge->init();
  roboCleanerExternalBridge->clean();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(roboCleanerExternalBridge);
  executor.spin();
  rclcpp::spin(roboCleanerExternalBridge);
  std::cout << "shutting down for some reason" << std::endl;
  rclcpp::shutdown();
  // spinThread.join();
  // t1.join();
  // t.join();
  return 0;
}

// ros2 action send_goal /move_robot robo_cleaner_interfaces/action/RobotMove "{ robot_move_type: { move_type: 0 } }" --feedback
