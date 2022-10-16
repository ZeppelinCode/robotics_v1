#ifndef _H_ROBO_MINER_CONTROLLER_EXTERNAL_BRIDGE
#define _H_ROBO_MINER_CONTROLLER_EXTERNAL_BRIDGE

#include<rclcpp/node.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"

class RoboMinerControllerExternalBridge : public rclcpp::Node {
public:
    RoboMinerControllerExternalBridge();
    int32_t init();
    void run();
private:
    using RobotMove = robo_miner_interfaces::srv::RobotMove;
    using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;

    rclcpp::Client<QueryInitialRobotPosition>::SharedPtr initialRobotPositionClient;
};

#endif