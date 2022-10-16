#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "robo_miner_controller/robo_miner_experiment.h"
#include "robo_miner_controller/map_graph.h"
#include "robo_miner_interfaces/srv/query_initial_robot_position.h"
#include "robo_miner_controller/robo_miner_controller_external_bridge.h"


namespace {
    using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
    using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
    using RobotMove = robo_miner_interfaces::srv::RobotMove;
    using RobotMoveType = robo_miner_interfaces::msg::RobotMoveType;
}

enum class RobotDirection {
  UP, RIGHT, DOWN, LEFT
};

RobotDirection toDirection(int8_t d) {
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
    return RobotDirection::UP;
  }
}

struct RobotState {
  RobotDirection direction;
  std::shared_ptr<GraphNode> currentNode;
  std::array<uint8_t, 3> surroundingTiles;

  RobotState(
    RobotDirection direction,
    std::shared_ptr<GraphNode> currentNode,
    std::array<uint8_t, 3> surroundingTiles) : direction{direction}, currentNode{currentNode}, surroundingTiles{surroundingTiles} {}

  std::string toString() {
    std::stringstream r;
    r << "direction: " << static_cast<int8_t>(direction) << ", surroundingTiles: ";
    for (int i = 0; i < 3; i++) {
      r << surroundingTiles[i] << ", ";
    }
    return r.str();
  }
};

std::shared_ptr<GraphNode> getInitialPosition(
  std::shared_ptr<rclcpp::Node> node, 
  std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> client,
  int8_t* oOrientation,
  std::array<uint8_t, 3>* oSurroundingTiles 
  ) {
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();
  auto result = client->async_send_request(request);

  // Wait for the result. TODO error handling
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 

  const std::shared_ptr<QueryInitialRobotPosition::Response> responseData = result.get();
  uint8_t initialTile = responseData->robot_initial_tile;
  const RobotPositionResponse rpr = responseData->robot_position_response;

  std::cout << "initial tile " << initialTile << std::endl;
  std::cout << "orientation: " << rpr.robot_dir << std::endl;
  *oOrientation = static_cast<int8_t>(rpr.robot_dir);
  *oSurroundingTiles = rpr.surrounding_tiles;

  std::cout << "surrounding tiles" << std::endl;
  for (int i = 0; i < 3; i++) {
      std::cout << rpr.surrounding_tiles[i] << " ";
  }
  std::cout << std::endl;


  GraphNode initialNode(Coordinate(0, 0), initialTile);
  return std::make_shared<GraphNode>(initialNode);
}

Coordinate calculateNewCoordianteBasedOnMovement(RobotDirection currentDirection, Coordinate oldCoordinate) {
  switch (currentDirection) {
  case RobotDirection::LEFT:
    return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
  case RobotDirection::UP:
    return Coordinate(oldCoordinate.x, oldCoordinate.y -1);
  case RobotDirection::RIGHT:
    return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
  case RobotDirection::DOWN:
    return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
  default:
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  }
}

void moveForward(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<rclcpp::Client<RobotMove>> moveClient,
  RobotState *robotState
) {
  auto request = std::make_shared<RobotMove::Request>();
  request->robot_move_type.move_type = RobotMoveType::FORWARD;
  auto result = moveClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 
  const std::shared_ptr<RobotMove::Response> responseData = result.get();
  // TODO error handling (do we care about non successful response? given a correct algorithm we should never bump into things)
  GraphNode newNode = GraphNode(
    calculateNewCoordianteBasedOnMovement(robotState->direction, robotState->currentNode->getCoordinate()),
    robotState->surroundingTiles[1]
  );
  robotState->direction = toDirection(responseData->robot_position_response.robot_dir);
  robotState->surroundingTiles = responseData->robot_position_response.surrounding_tiles;
  robotState->currentNode = std::make_shared<GraphNode>(newNode);
}

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

  int8_t initialRobotDirection;
  std::array<uint8_t, 3> initialSurroundingTiles;
  MapGraph mapGraph{};
  auto initialGraphNode = getInitialPosition(node, initialPositionClient, &initialRobotDirection, &initialSurroundingTiles);
  mapGraph.addNode(initialGraphNode);
  RobotState robotState(toDirection(initialRobotDirection), initialGraphNode, initialSurroundingTiles);
  std::cout << "initial robot state" << robotState.toString() << std::endl;
  moveForward(node, moveClient, &robotState);
  std::cout << "robot state after move" << robotState.toString() << std::endl;
  mapGraph.addNode(robotState.currentNode);
  
  std::cout << "seen" << std::endl;
  std::cout << mapGraph.toString() << std::endl;

  return EXIT_SUCCESS;
}