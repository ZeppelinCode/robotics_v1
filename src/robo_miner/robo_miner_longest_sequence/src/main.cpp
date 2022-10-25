#include <rclcpp/rclcpp.hpp>
#include <iostream>
// #include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_longest_sequence/longest_sequence.h"
#include "robo_miner_interfaces/srv/longest_sequence_calculate.hpp"
#include "robo_miner_interfaces/msg/u_int8_multi_array.hpp"
#include "robo_miner_interfaces/msg/coordinate.hpp"

MapStructure fieldMapToMapStructure(const robo_miner_interfaces::msg::UInt8MultiArray& in) {
  MapStructure mapStructure;
  mapStructure.cols = in.cols;
  mapStructure.rows = in.rows;
  mapStructure.data = in.data;
  return mapStructure;
}

std::vector<robo_miner_interfaces::msg::Coordinate> domainLongestSequenceToTransportLongestSequence(const std::vector<Coordinate>& domainLongestSequence) {
 std::vector<robo_miner_interfaces::msg::Coordinate> retval{};
 for (const auto& domainCoord: domainLongestSequence) {
  robo_miner_interfaces::msg::Coordinate transportCoord;
  transportCoord.x = domainCoord.x;
  transportCoord.y = domainCoord.y;
  retval.emplace_back(transportCoord);
 }

 return retval;
}

void calculateLongestSequence(const std::shared_ptr<robo_miner_interfaces::srv::LongestSequenceCalculate::Request> request,
         std::shared_ptr<robo_miner_interfaces::srv::LongestSequenceCalculate::Response>      response
) {
  std::cout << "[LONGEST SEQUENCE] received calculate longest sequence request" << std::endl;
  MapStructure mapStructure = fieldMapToMapStructure(request->field_map);
  std::vector<Coordinate> longestSequence = getLongestSequence(mapStructure);
  response->longest_sequence = domainLongestSequenceToTransportLongestSequence(longestSequence);
}

int main(int argc, char ** argv)
{
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = true;
  rclcpp::init(argc, argv, initOptions);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("longest_sequence_service_node");

  rclcpp::Service<robo_miner_interfaces::srv::LongestSequenceCalculate>::SharedPtr service =
    node->create_service<robo_miner_interfaces::srv::LongestSequenceCalculate>(LONGEST_SEQUENCE_CALCULATE_SERVICE, &calculateLongestSequence);

  std::cout << "[LONGEST SEQUENCE] started" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
