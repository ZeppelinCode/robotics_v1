#include <sstream>
#include "robo_cleaner_controller/robot_state.h"

static std::string actionToString(StateMachine action) {
    switch (action) {
    case StateMachine::IDLE:
        return "IDLE";
    case StateMachine::GOING_FORWARD:
        return "GOING_FORWARD";
    case StateMachine::TURNING_LEFT:
        return "TURNING_LEFT";
    case StateMachine::TURNED_LEFT_GOING_FORWARD:
        return "TURNED_LEFT_GOING_FORWARD";
    case StateMachine::TURNING_RIGHT:
        return "TURNING_RIGHT";
    case StateMachine::TURNED_RIGHT_GOING_FORWARD:
        return "TURNED_RIGHT_GOING_FORWARD";
    default:
        return "action ???";
    }
}

RobotState::RobotState() {}

std::string RobotState::toString() {
    std::stringstream representation;
    representation << "action: " << actionToString(this->action) << std::endl
                    << "node: " << this->currentNode->toString() << std::endl
                    << "direction: " << robot_state::robotDirectionToString(this->direction) << std::endl
                    << "moves left: " << this->movesLeft << std::endl
                    << "----";
    return representation.str();
}

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
