#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

#include "robo_miner_controller/robo_miner_experiment.h"
#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_controller/mover_communicator.h"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_calculate.hpp"
#include "robo_miner_interfaces/msg/field_point.hpp"
#include "robo_miner_interfaces/msg/coordinate.hpp"
#include "robo_miner_interfaces/msg/u_int8_multi_array.hpp"
#include "robo_miner_controller/coordinate_remapper.h"
#include "robo_miner_common/defines/RoboMinerTopics.h"

robo_miner_interfaces::msg::FieldPoint coordinateToFieldPoint(const Coordinate& c) {
  robo_miner_interfaces::msg::FieldPoint fieldPoint;
  fieldPoint.row = c.y;
  fieldPoint.col = c.x;
  return fieldPoint;
}

std::vector<robo_miner_interfaces::msg::FieldPoint> convertCoordinatesToFieldPoints(const std::vector<Coordinate>& coordinates) {
  std::vector<robo_miner_interfaces::msg::FieldPoint> retval{};
  for (const auto& coord : coordinates) {
    retval.emplace_back(coordinateToFieldPoint(coord));
  }
  return retval;
}

std::vector<Coordinate> getLongestSequence(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<rclcpp::Client<robo_miner_interfaces::srv::LongestSequenceCalculate>> client,
  robo_miner_interfaces::msg::UInt8MultiArray fieldMap) {
  
  auto request = std::make_shared<robo_miner_interfaces::srv::LongestSequenceCalculate::Request>();
  request->field_map = fieldMap;
  
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
  }
  auto response = result.get();

  std::vector<Coordinate> retval;
  for (const auto& transportCoord : response->longest_sequence) {
    retval.emplace_back(Coordinate(transportCoord.x, transportCoord.y));
  }
  return retval;
}

int32_t run_experiment() {
  using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
  using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
  using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;
  using LongestSequenceCalculate = robo_miner_interfaces::srv::LongestSequenceCalculate;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robo_miner_controller_node");
  auto initialPositionClient = node->create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE);
  auto moveClient = node->create_client<RobotMove>(ROBOT_MOVE_SERVICE);
  auto validateMapClient = node->create_client<FieldMapValidate>(FIELD_MAP_VALIDATE_SERVICE);
  auto miningValidateClient = node->create_client<ActivateMiningValidate>(ACTIVATE_MINING_VALIDATE_SERVICE);
  auto validateLongestSequenceClient = node->create_client<LongestSequenceValidate>(LONGEST_SEQUENCE_VALIDATE_SERVICE);
  auto getLongestSequenceClient = node->create_client<LongestSequenceCalculate>(LONGEST_SEQUENCE_CALCULATE_SERVICE);

  auto activateMiningValidateFn = [miningValidateClient, node]() {
    auto request = std::make_shared<ActivateMiningValidate::Request>();
    auto result = miningValidateClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
    }
    const auto response = result.get();
    std::cout << "got response for activate mining validate: " << response->success << " with failure reason" << response->error_reason << std::endl;

  };

  auto validateMapFn = [validateMapClient, validateLongestSequenceClient, getLongestSequenceClient, node](const MapStructure& mapStructure) {
    auto validateMapRequest = std::make_shared<FieldMapValidate::Request>();
    validateMapRequest->field_map.rows = mapStructure.rows;
    validateMapRequest->field_map.cols = mapStructure.cols;
    validateMapRequest->field_map.data = mapStructure.data;
    std::cout << "sending validate map request" << std::endl;
    auto result = validateMapClient->async_send_request(validateMapRequest);
    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
    }
    std::cout << "" << std::endl;
    const auto response = result.get();
    std::cout << "got response for map submission: " << response->success << " with failure reason" << response->error_reason << std::endl;

    // Longest trail request
    std::cout << "sending get longest sequence request" << std::endl;
    auto longestConnectedCoordinates = getLongestSequence(node, getLongestSequenceClient, validateMapRequest->field_map);
    std::cout << "got response for longest sequence" << std::endl;
    auto sequenceRequest = std::make_shared<LongestSequenceValidate::Request>();
    sequenceRequest->sequence_points = convertCoordinatesToFieldPoints(longestConnectedCoordinates);;
    std::cout << "sending validate longest sequence request" << std::endl;
    auto seuqenceResult = validateLongestSequenceClient->async_send_request(sequenceRequest);
        if (rclcpp::spin_until_future_complete(node, seuqenceResult) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
    }
    const auto sequenceResponse = seuqenceResult.get();
    std::cout << "got validate longest sequence response: " << sequenceResponse->success << " with failure reason" << response->error_reason << std::endl;
    return longestConnectedCoordinates;
  };

  while (!initialPositionClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  MinerNavigator navigator(MoverCommunicator(node, moveClient), node, initialPositionClient, validateMapFn, activateMiningValidateFn);
  navigator.init();
  navigator.exploreMap();

  return EXIT_SUCCESS;
}