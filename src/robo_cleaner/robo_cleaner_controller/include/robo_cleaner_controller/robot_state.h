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

namespace robot_state {
  RobotDirection numberToDirection(int8_t d); 
  int8_t directionToNumber(RobotDirection direction);
  std::string robotDirectionToString(RobotDirection direction);
}

class RobotState {
public:
  RobotState();
  int32_t movesLeft = 0;
  RobotDirection direction = RobotDirection::UNKNOWN;
  std::shared_ptr<GraphNode> currentNode = nullptr;
};

#endif