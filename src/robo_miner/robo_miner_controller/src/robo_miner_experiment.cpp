#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "robo_miner_controller/robo_miner_experiment.h"
#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_controller/mover_communicator.h"


// Coordinate calculateNewCoordianteBasedOnMovement(RobotDirection currentDirection, Coordinate oldCoordinate) {
//   switch (currentDirection) {
//   case RobotDirection::LEFT:
//     return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
//   case RobotDirection::UP:
//     return Coordinate(oldCoordinate.x, oldCoordinate.y -1);
//   case RobotDirection::RIGHT:
//     return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
//   case RobotDirection::DOWN:
//     return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
//   default:
//     return Coordinate(oldCoordinate.x, oldCoordinate.y);
//   }
// }

// TODO error handling
int32_t run_experiment() {
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("try_to_get_initial_conditions");
  auto initialPositionClient = node->create_client<QueryInitialRobotPosition>("query_initial_robot_position");
  auto moveClient = node->create_client<RobotMove>("move_robot");

  while (!initialPositionClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  MinerNavigator navigator(MoverCommunicator(node, moveClient), node, initialPositionClient);
  navigator.init();
  navigator.exploreMap();

  // int8_t initialRobotDirection;
  // std::array<uint8_t, 3> initialSurroundingTiles;
  // MapGraph mapGraph{};
  // auto initialGraphNode = getInitialPosition(node, initialPositionClient, &initialRobotDirection, &initialSurroundingTiles);
  // mapGraph.addNode(initialGraphNode);
  // RobotState robotState(toDirection(initialRobotDirection), initialGraphNode, initialSurroundingTiles);
  // std::cout << "initial robot state" << robotState.toString() << std::endl;
  // moveForward(node, moveClient, &robotState);
  // std::cout << "robot state after move" << robotState.toString() << std::endl;
  // mapGraph.addNode(robotState.currentNode);
  
  // std::cout << "seen" << std::endl;
  // std::cout << mapGraph.toString() << std::endl;

  return EXIT_SUCCESS;
}