#ifndef _H_ROBO_CLEANER_EXTERNAL_BRIDGE
#define _H_ROBO_CLEANER_EXTERNAL_BRIDGE

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robo_cleaner_interfaces/srv/query_initial_robot_state.hpp"
#include "robo_cleaner_interfaces/srv/query_battery_status.hpp"
#include "robo_cleaner_interfaces/srv/charge_battery.hpp"
#include "robo_cleaner_interfaces/action/robot_move.hpp"
#include "robo_cleaner_controller/map_graph.h"
#include "robo_cleaner_controller/robot_state.h"
#include "robo_cleaner_controller/shortest_path_walker.h"
#include "robo_cleaner_interfaces/msg/user_authenticate.hpp"

class RoboCleanerExternalBridge : public rclcpp::Node {
public:
  using QueryInitialRobotState = robo_cleaner_interfaces::srv::QueryInitialRobotState;
  using RobotMove = robo_cleaner_interfaces::action::RobotMove;
  using QueryBatteryStatus = robo_cleaner_interfaces::srv::QueryBatteryStatus;
  using ChargeBattery = robo_cleaner_interfaces::srv::ChargeBattery;
  using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;
  using UserAuthenticate = robo_cleaner_interfaces::msg::UserAuthenticate;


  RoboCleanerExternalBridge();
  void init();
  void clean();
  void issueMoveOrder(int8_t moveType);
private:
  std::shared_ptr<rclcpp::Client<QueryInitialRobotState>> initialRobotStateClient;
  std::shared_ptr<rclcpp::Client<QueryBatteryStatus>> queryBatteryStatusClient;
  std::shared_ptr<rclcpp::Client<ChargeBattery>> chargeBatteryClient;
  std::shared_ptr<rclcpp::Publisher<UserAuthenticate>> userAuthenticatePublisher;
  std::shared_ptr<rclcpp_action::Client<RobotMove>> moveActionClient;
  MapGraph map{};
  RobotState robotState;
  rclcpp::TimerBase::SharedPtr timer;
  std::atomic<bool> isActionRunning = false;
  std::atomic<bool> isWalkingTowardsChargingStation = false;
  std::recursive_mutex actionLock{};
  std::vector<Coordinate> unvisitedCoordinates; // Visit candidates separate class
  ShortestPathWalker shortestPathWalker{};
  std::mutex batteryLock{};
  rclcpp::Node::SharedPtr _sharedReferenceToSelf;
  rclcpp::CallbackGroup::SharedPtr batteryStatusCallbackGroup;
  int tickCounter{0};

  void moveGoalResponseCallback(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
  void moveGoalFeedbackCallback(GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback);
  void moveGoalResultCallback(const GoalHandleRobotMove::WrappedResult & result);
  void timerCallback();
  bool onlyOneSpotLeftAndImOnIt();
  void queryInitialState();
  void updateBatteryStatus();
  void chargeBatteryToFull();
  bool shouldRecharge();
  bool isChargeFull();
  std::vector<Coordinate> getClockwiseCoordinatesAroundMe();
  void goLeft();
  void goRight();
  void goForward();
  void turnAround();
  bool goToCoordinate(const Coordinate& coord);
  void scrubLastSpot();
};
#endif