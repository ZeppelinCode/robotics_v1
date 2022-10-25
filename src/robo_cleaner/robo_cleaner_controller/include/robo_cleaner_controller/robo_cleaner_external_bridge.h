#ifndef _H_ROBO_CLEANER_EXTERNAL_BRIDGE
#define _H_ROBO_CLEANER_EXTERNAL_BRIDGE

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robo_cleaner_interfaces/srv/query_initial_robot_state.hpp"
#include "robo_cleaner_interfaces/action/robot_move.hpp"

class RoboCleanerExternalBridge : public rclcpp::Node {
public:
  using QueryInitialRobotState = robo_cleaner_interfaces::srv::QueryInitialRobotState;
  using RobotMove = robo_cleaner_interfaces::action::RobotMove;
  using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;
  RoboCleanerExternalBridge();
  void init();
  void queryInitialState();
  void issueMoveOrder(int8_t moveType);
private:
  std::shared_ptr<rclcpp::Client<QueryInitialRobotState>> initialRobotStateClient;
  std::shared_ptr<rclcpp_action::Client<RobotMove>> moveActionClient;

  void moveGoalResponseCallback(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
  void moveGoalFeedbackCallback(GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback);
  void moveGoalResultCallback(const GoalHandleRobotMove::WrappedResult & result);
};
#endif