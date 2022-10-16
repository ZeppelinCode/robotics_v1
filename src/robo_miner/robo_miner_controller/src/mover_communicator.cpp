#include "robo_miner_controller/mover_communicator.h"


MoverCommunicator::MoverCommunicator(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<RobotMove>> moveClient
) : node{node}, moveClient{moveClient} {
}

// Compilation issues of sendCommand(RobotMoveType robotMoveType) -> repetition to get something going
std::shared_ptr<RobotMove::Response> MoverCommunicator::sendMoveForwardCommand() {
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = RobotMoveType::FORWARD;
  auto result = moveClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 
  return result.get();
}

std::shared_ptr<RobotMove::Response> MoverCommunicator::sendTurnLeftCommand() {
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = RobotMoveType::ROTATE_LEFT;
  auto result = moveClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 
  return result.get();
}

std::shared_ptr<RobotMove::Response> MoverCommunicator::sendTurnRightCommand() {
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = RobotMoveType::ROTATE_RIGHT;
  auto result = moveClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 
  return result.get();
}
//   // TODO error handling (do we care about non successful response? given a correct algorithm we should never bump into things)
//   GraphNode newNode = GraphNode(
//     calculateNewCoordianteBasedOnMovement(robotState->direction, robotState->currentNode->getCoordinate()),
//     robotState->surroundingTiles[1]
//   );
//   robotState->direction = toDirection(responseData->robot_position_response.robot_dir);
//   robotState->surroundingTiles = responseData->robot_position_response.surrounding_tiles;
//   robotState->currentNode = std::make_shared<GraphNode>(newNode);
// }

// std::shared_ptr<RobotMove::Response> MoverCommunicator::sendCommand(RobotMoveType robotMoveType) {
//   auto request = std::make_shared<RobotMove::Request>();
//   request->robot_move_type.move_type = robotMoveType; // ?? :(
//   auto result = moveClient->async_send_request(request);
//   if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//   } 
//   return result.get();
// }