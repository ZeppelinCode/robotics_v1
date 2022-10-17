#ifndef _H_MINER_NAVIGATOR
#define _H_MINER_NAVIGATOR

#include <rclcpp/rclcpp.hpp>
#include <stack>

#include "robo_miner_controller/mover_communicator.h"
#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_controller/map_graph.h"
#include "robo_miner_controller/coordinate_remapper.h"

namespace {
  using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
}

enum class RobotDirection : uint8_t {
  UP, RIGHT, DOWN, LEFT, UNKNOWN
};

static std::string dirToStr(RobotDirection direction) {
  switch (direction)
  {
  case RobotDirection::UP:
    return "UP";
  case RobotDirection::RIGHT:
    return "RIGHT";
  case RobotDirection::DOWN:
    return "DOWN";
  case RobotDirection::LEFT:
    return "LEFT";
  default:
    return "UNKNOWN";
  }
}

struct RobotState {
  RobotDirection direction;
  std::shared_ptr<GraphNode> currentNode;
  std::array<uint8_t, 3> surroundingTiles;

  RobotState() {
    direction = RobotDirection::UNKNOWN;
    surroundingTiles = {0, 0, 0};
  }

  RobotState(
    RobotDirection direction,
    std::shared_ptr<GraphNode> currentNode,
    std::array<uint8_t, 3> surroundingTiles) : direction{direction}, currentNode{currentNode}, surroundingTiles{surroundingTiles} {}

  std::string toString() {
    std::stringstream r;
    r << "direction: " << dirToStr(direction) << ", surroundingTiles: ";
    for (int i = 0; i < 3; i++) {
      r << surroundingTiles[i] << ", ";
    }
    return r.str();
  }
};

class MinerNavigator {
public:
  MinerNavigator(
    MoverCommunicator&& moverCommunicator,
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient,
    std::function<void(MapStructure&)> submitMapStructureFn
    );
  void init();
  void exploreMap();
private:
  MoverCommunicator moverCommunicator;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient;
  std::function<void(MapStructure&)> submitMapStructureFn;

  RobotState robotState;
  MapGraph mapGraph;

  std::stack<Coordinate> coordinatesTrail;

  std::vector<uint8_t> pickNonCollisionTileIndex();
  std::vector<std::function<void()>> getActionVector();
  std::vector<std::pair<uint8_t, Coordinate>> getValidMovementCoordinates();
  void goForward();
  void goLeft();
  void goRight();
  // void goBack();
  void backtrackUntilUnstuck();
};
#endif