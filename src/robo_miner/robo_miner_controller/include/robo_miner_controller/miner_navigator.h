#ifndef _H_MINER_NAVIGATOR
#define _H_MINER_NAVIGATOR

#include<rclcpp/rclcpp.hpp>
#include "robo_miner_controller/mover_communicator.h"
#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"

namespace {
  using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
}

enum class RobotDirection {
  UP, RIGHT, DOWN, LEFT, UNKNOWN
};



struct RobotState {
  RobotDirection direction;
  // std::shared_ptr<GraphNode> currentNode;
  std::array<uint8_t, 3> surroundingTiles;

  RobotState() {
    direction = RobotDirection::UNKNOWN;
    surroundingTiles = {0, 0, 0};
  }

  RobotState(
    RobotDirection direction,
    // std::shared_ptr<GraphNode> currentNode,
    std::array<uint8_t, 3> surroundingTiles) : direction{direction}, /*currentNode{currentNode},*/ surroundingTiles{surroundingTiles} {}

  std::string toString() {
    std::stringstream r;
    r << "direction: " << static_cast<int8_t>(direction) << ", surroundingTiles: ";
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
    std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient 
    );
  void init();
  void exploreMap();
private:
  MoverCommunicator moverCommunicator;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient;

  RobotState robotState;

  void goForward();
  void goLeft();
  void goRight();
  void goBack();
};
#endif