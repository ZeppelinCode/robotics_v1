#include "robo_cleaner_controller/direction_coordinate_calculator.h"

RobotDirection calculateDirectionBasedOnCurrentDirectionAndTurnCommand(RobotDirection currentDirection, StateMachine turnCommand) {
  if (turnCommand == StateMachine::TURNING_LEFT) {
    switch (currentDirection)
    {
    case RobotDirection::LEFT:
      return RobotDirection::DOWN;
    case RobotDirection::UP:
      return RobotDirection::LEFT;
    case RobotDirection::DOWN:
      return RobotDirection::RIGHT;
    case RobotDirection::RIGHT:
      return RobotDirection::UP;
    case RobotDirection::UNKNOWN:
    default:
      throw std::runtime_error("unknown direction so no idea where we're turning to (TURNING_LEFT)");
      break;
    }
  }
  if (turnCommand == StateMachine::TURNING_RIGHT) {
    switch (currentDirection)
    {
    case RobotDirection::LEFT:
      return RobotDirection::UP;
    case RobotDirection::UP:
      return RobotDirection::RIGHT;
    case RobotDirection::DOWN:
      return RobotDirection::LEFT;
    case RobotDirection::RIGHT:
      return RobotDirection::DOWN;
    case RobotDirection::UNKNOWN:
    default:
      throw std::runtime_error("unknown direction so no idea where we're turning to (TURNING_RIGHT)");
      break;
    }
  }
  return RobotDirection::UNKNOWN;
}

Coordinate calculateNewCoordianteBasedOnDirection(RobotDirection currentDirection, Coordinate oldCoordinate) {
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

Coordinate calculateNewCoordianteBasedOnDirectionAndTurnIndex(RobotDirection currentDirection, Coordinate oldCoordinate, uint8_t clockwiseTurnIndex) {
  switch (currentDirection) {
  case RobotDirection::LEFT:
    if (clockwiseTurnIndex == CLOCKWISE_LEFT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_BEHIND_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::UP:
    if (clockwiseTurnIndex == CLOCKWISE_LEFT_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_BEHIND_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::RIGHT:
    if (clockwiseTurnIndex == CLOCKWISE_LEFT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_BEHIND_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::DOWN:
    if (clockwiseTurnIndex == CLOCKWISE_LEFT_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    if (clockwiseTurnIndex == CLOCKWISE_RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    if (clockwiseTurnIndex == CLOCKWISE_BEHIND_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  default:
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  }
}