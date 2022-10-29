#ifndef _H_ROBOT_STATE
#define _H_ROBOT_STATE

#include <cstdint>
#include <stdexcept>
#include <memory>
#include <string>
#include "robo_cleaner_controller/map_graph.h"

enum class RobotDirection : uint8_t {
  UP, RIGHT, DOWN, LEFT, UNKNOWN
};


enum class StateMachine : uint8_t {
  IDLE, GOING_FORWARD, TURNING_LEFT, TURNED_LEFT_GOING_FORWARD, TURNING_RIGHT, TURNED_RIGHT_GOING_FORWARD, FIRST_HALF_OF_TURN, SECOND_HALF_OF_TURN
};

namespace robot_state {
  RobotDirection numberToDirection(int8_t d); 
  int8_t directionToNumber(RobotDirection direction);
  std::string robotDirectionToString(RobotDirection direction);
}

class RobotState {
public:
  RobotState();
  int32_t movesLeft = 0;
  int32_t maxMovesOnFullEnergy = 0;
  RobotDirection direction = RobotDirection::UNKNOWN;
  std::shared_ptr<GraphNode> currentNode = nullptr;
  StateMachine action = StateMachine::IDLE;
  std::string toString();
};

#endif