#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "robo_miner_controller/robo_miner_experiment.h"
// #include "robo_miner_controller/robo_miner_controller_external_bridge.h"


int32_t main(int32_t argc, char *argv[])
{

  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = true;
  rclcpp::init(argc, argv, initOptions);

  std::cout << "hello from miner controller" << std::endl;

  run_experiment();

  // auto node = std::make_shared<RoboMinerControllerExternalBridge>();
  // if (node->init() != EXIT_SUCCESS) {
  //   std::cout << "failed to init miner controller external bridge" << std::endl;
  //   return EXIT_FAILURE;
  // }

  // std::cout << "spinning node" << std::endl;
  // rclcpp::spin(node);



  // std::cout << "running node" << std::endl;
  // node->run();

  std::cout << "shutdown" << std::endl;
  rclcpp::shutdown();
  return 0;
}
