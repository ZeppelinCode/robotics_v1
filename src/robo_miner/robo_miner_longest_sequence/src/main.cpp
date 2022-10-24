#include <rclcpp/rclcpp.hpp>
#include "robo_miner_interfaces/srv/longest_sequence_calculate.hpp"

void add(const std::shared_ptr<robo_miner_interfaces::srv::LongestSequenceCalculate::Request> request,
         std::shared_ptr<robo_miner_interfaces::srv::LongestSequenceCalculate::Response>      response
) {
  auto fieldMap = request->field_map;
}

int main(int argc, char ** argv)
{
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = true;
  rclcpp::init(argc, argv, initOptions);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("calculate_longest_sequence_server");

  rclcpp::Service<robo_miner_interfaces::srv::LongestSequenceCalculate>::SharedPtr service =
    node->create_service<robo_miner_interfaces::srv::LongestSequenceCalculate>("add_two_ints", &add);



  return 0;
}
