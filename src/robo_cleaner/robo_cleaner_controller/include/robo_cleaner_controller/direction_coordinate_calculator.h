#ifndef _H_DIRECTION_COORDINATE_CALCULATOR
#define _H_DIRECTION_COORDINATE_CALCULATOR

#include "robo_cleaner_controller/robot_state.h"


constexpr auto CLOCKWISE_FORWARD_INDEX = 0;
constexpr auto CLOCKWISE_RIGHT_INDEX = 1;
constexpr auto CLOCKWISE_BEHIND_INDEX = 2;
constexpr auto CLOCKWISE_LEFT_INDEX = 3;

RobotDirection calculateDirectionBasedOnCurrentDirectionAndTurnCommand(RobotDirection currentDirection, StateMachine turnCommand);
Coordinate calculateNewCoordianteBasedOnDirection(RobotDirection currentDirection, Coordinate oldCoordinate);
Coordinate calculateNewCoordianteBasedOnDirectionAndTurnIndex(RobotDirection currentDirection, Coordinate oldCoordinate, uint8_t clockwiseTurnIndex);

#endif