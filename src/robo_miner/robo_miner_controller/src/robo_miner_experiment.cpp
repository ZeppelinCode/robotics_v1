#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

#include "robo_miner_controller/robo_miner_experiment.h"
#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_controller/mover_communicator.h"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/msg/field_point.hpp"
#include "robo_miner_controller/coordinate_remapper.h"
#include "robo_miner_controller/longest_trail.h"

robo_miner_interfaces::msg::FieldPoint coordinateToFieldPoint(Coordinate c) {
  robo_miner_interfaces::msg::FieldPoint fieldPoint;
  fieldPoint.row = c.y;
  fieldPoint.col = c.x;
  return fieldPoint;
}
// TODO error handling
int32_t run_experiment() {
  using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
  using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("try_to_get_initial_conditions");
  // TODO the service names should be pulled as constants
  auto initialPositionClient = node->create_client<QueryInitialRobotPosition>("query_initial_robot_position");
  auto moveClient = node->create_client<RobotMove>("move_robot");
  auto validateMapClient = node->create_client<FieldMapValidate>("field_map_validate");
  auto validateLongestSequenceClient = node->create_client<LongestSequenceValidate>("longest_sequence_validate");
  auto validateMapFn = [validateMapClient, validateLongestSequenceClient, node](const MapStructure& mapStructure) {
    auto request = std::make_shared<FieldMapValidate::Request>();
    request->field_map.rows = mapStructure.rows;
    request->field_map.cols = mapStructure.cols;
    request->field_map.data = mapStructure.data;
    std::cout << "sending validate request" << std::endl;
    auto result = validateMapClient->async_send_request(request);
    std::cout << "waiting for response" << std::endl;
    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
    }
    std::cout << "" << std::endl;
    const auto response = result.get();
    std::cout << "got response for map submission: " << response->success << " with failure reason" << response->error_reason << std::endl;

    // Longest trail request
    std::cout << "running longest trail" << std::endl;
    auto longestConnectedCoordinates = longest_trail::getLongestTrail(mapStructure);
    auto sequenceRequest = std::make_shared<LongestSequenceValidate::Request>();
    std::transform(longestConnectedCoordinates.begin(), longestConnectedCoordinates.end(), sequenceRequest->sequence_points.begin(),
      &coordinateToFieldPoint);
    auto seuqenceResult = validateLongestSequenceClient->async_send_request(sequenceRequest);
        if (rclcpp::spin_until_future_complete(node, seuqenceResult) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call field map valdiate service");
    }
    std::cout << "" << std::endl;
    const auto sequenceResponse = seuqenceResult.get();
    std::cout << "got sequenceResponse for map submission: " << sequenceResponse->success << " with failure reason" << response->error_reason << std::endl;
  };

  while (!initialPositionClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  MinerNavigator navigator(MoverCommunicator(node, moveClient), node, initialPositionClient, validateMapFn);
  navigator.init();
  navigator.exploreMap();

  return EXIT_SUCCESS;
}