#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"
#include <functional>

namespace {
    using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
}

static RobotDirection toDirection(int8_t d) {
  switch (d)
  {
  case 0:
    return RobotDirection::UP;
  case 1:
    return RobotDirection::RIGHT;
  case 2:
    return RobotDirection::DOWN;
  case 3:
    return RobotDirection::LEFT;
  default:
    return RobotDirection::UNKNOWN;
  }
}

MinerNavigator::MinerNavigator(
    MoverCommunicator&& moverCommunicator,
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient
    ) : moverCommunicator{moverCommunicator}, node{node}, initialRobotPositionClient{initialRobotPositionClient} {}

void MinerNavigator::init() {
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();
  auto result = initialRobotPositionClient->async_send_request(request);

  // Wait for the result. TODO error handling
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 

  const std::shared_ptr<QueryInitialRobotPosition::Response> responseData = result.get();
  // uint8_t initialTile = responseData->robot_initial_tile;
  const RobotPositionResponse rpr = responseData->robot_position_response;

  robotState.direction = toDirection(rpr.robot_dir);
  robotState.surroundingTiles = rpr.surrounding_tiles;

  std::cout << "initial state " << robotState.toString() << std::endl;
}

int8_t pickNonCollisionTile(const std::array<uint8_t, 3> &surroundingTiles) {
  for (int i = 0; i < 3; i++) {
    if (surroundingTiles[i] != 'X' && surroundingTiles[i] != '#') {
      return i;
    }
  }
  return -1;
}

void MinerNavigator::exploreMap() {
  for (int i = 0; i < 100; i++) {
    auto idxOfNonCollisionTile = pickNonCollisionTile(robotState.surroundingTiles);
    if (idxOfNonCollisionTile == 0) {
      goLeft();
      continue;
    }
    if (idxOfNonCollisionTile == 1) {
      goForward();
      continue;;
    }
    if (idxOfNonCollisionTile == 2) {
      goRight();
      continue;
    }

    // We are in some corner and need to turn to find a way out
    std::cout << "stuck in a corner, trying to turn" << std::endl;
    auto result = moverCommunicator.sendTurnRightCommand();
    robotState.direction = toDirection(result->robot_position_response.robot_dir);
    robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  }
}

void MinerNavigator::goForward() {
  auto result = moverCommunicator.sendMoveForwardCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "moved forward, state updated to" << robotState.toString() << std::endl;
}

void MinerNavigator::goLeft() {
  moverCommunicator.sendTurnLeftCommand();
  auto result = moverCommunicator.sendMoveForwardCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "moved left, state updated to" << robotState.toString() << std::endl;
}

void MinerNavigator::goRight() {
  moverCommunicator.sendTurnRightCommand();
  auto result = moverCommunicator.sendMoveForwardCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "moved right, state updated to" << robotState.toString() << std::endl;
}

void MinerNavigator::goBack() {
  moverCommunicator.sendTurnRightCommand();
  moverCommunicator.sendTurnRightCommand();
  auto result = moverCommunicator.sendMoveForwardCommand();

  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "moved back, state updated to" << robotState.toString() << std::endl;
}