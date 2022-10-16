#ifndef _H_MOVER_COMMUNICATOR
#define _H_MOVER_COMMUNICATOR

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/msg/robot_move_type.hpp"

namespace {
  using RobotMove = robo_miner_interfaces::srv::RobotMove;
  using RobotMoveType = robo_miner_interfaces::msg::RobotMoveType;
}

class MoverCommunicator {
public:
  MoverCommunicator(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<RobotMove>> moveClient
    );
  std::shared_ptr<RobotMove::Response> sendMoveForwardCommand();
  std::shared_ptr<RobotMove::Response> sendTurnLeftCommand();
  std::shared_ptr<RobotMove::Response> sendTurnRightCommand();
  
private:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Client<robo_miner_interfaces::srv::RobotMove>> moveClient;
  // std::shared_ptr<RobotMove::Response> sendCommand(RobotMoveType robotMoveType); ?? :(
};
#endif