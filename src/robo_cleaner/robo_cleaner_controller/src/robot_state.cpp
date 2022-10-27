#include "robo_cleaner_controller/robot_state.h"


RobotState::RobotState() {}

namespace robot_state
{
    RobotDirection numberToDirection(int8_t d) {
        switch (d) {
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

    int8_t directionToNumber(RobotDirection direction) {
        switch (direction) {
        case RobotDirection::UP:
        return 0;
        case RobotDirection::RIGHT:
        return 1;
        case RobotDirection::DOWN:
        return 2;
        case RobotDirection::LEFT:
        return 3;
        default:
        throw std::invalid_argument("unkown direction");
        }
    }

    std::string robotDirectionToString(RobotDirection direction) {
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
}
