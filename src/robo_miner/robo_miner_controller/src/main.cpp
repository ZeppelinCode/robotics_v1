#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "robo_miner_controller/robo_miner_experiment.h"


int32_t main(int32_t argc, char *argv[])
{

  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = true;
  rclcpp::init(argc, argv, initOptions);
  std::cout << "[CONTROLLER NODE] started" << std::endl;

  run_experiment();

  rclcpp::shutdown();
  return 0;
}
