#include <chrono>
#include <iostream>
#include "robo_cleaner_controller/robo_cleaner_external_bridge.h"
#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"

using namespace std::literals;
using namespace std::placeholders;

template <typename T, typename ActionName>
static void waitForAction(const T &action, const ActionName &actionName) {
  while (!action->wait_for_action_server(1s)) {
    std::cout << "Action: [" << actionName
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}

template <typename T>
static void waitForClientToBecomeReachable(const std::shared_ptr<T>& service) {
    while (!service->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

RoboCleanerExternalBridge::RoboCleanerExternalBridge() : Node("CleanerExternalBridge") {}

void RoboCleanerExternalBridge::init() {
    initialRobotStateClient = create_client<QueryInitialRobotState>(QUERY_INITIAL_ROBOT_STATE_SERVICE);
    waitForClientToBecomeReachable(initialRobotStateClient);

    moveActionClient = rclcpp_action::create_client<RobotMove>(this, ROBOT_MOVE_ACTION);
    waitForAction(moveActionClient, ROBOT_MOVE_ACTION);
    std::cout << "move action client initialized" << std::endl;
}

void RoboCleanerExternalBridge::queryInitialState() {
    std::cout << "calling get initial state" << std::endl;
    auto request = std::make_shared<QueryInitialRobotState::Request>();
    auto result = initialRobotStateClient->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get initial robot state");
        throw std::runtime_error("Failed to get initial robot state");
    } 

    const std::shared_ptr<QueryInitialRobotState::Response> responseData = result.get();
    const auto batteryStatus = responseData->initial_robot_state.battery_status;
    const auto direction = responseData->initial_robot_state.robot_dir;
    const auto tileDirt = responseData->initial_robot_state.robot_tile;

    std::cout << "initial state battery status " << batteryStatus.moves_left
                << " direction " << direction
                << " tile " << static_cast<int>(tileDirt) << std::endl;;
}

void RoboCleanerExternalBridge::issueMoveOrder(int8_t moveType) {
    auto sendGoalOptions = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    sendGoalOptions.goal_response_callback =
      std::bind(&RoboCleanerExternalBridge::moveGoalResponseCallback, this, _1);
    sendGoalOptions.feedback_callback =
      std::bind(&RoboCleanerExternalBridge::moveGoalFeedbackCallback, this, _1, _2);
    sendGoalOptions.result_callback =
      std::bind(&RoboCleanerExternalBridge::moveGoalResultCallback, this, _1);

    auto goal_msg = robo_cleaner_interfaces::action::RobotMove::Goal();
    robo_cleaner_interfaces::msg::RobotMoveType robotMoveType;
    robotMoveType.move_type = moveType;
    goal_msg.robot_move_type = robotMoveType;
    std::cout << "issuing move order" << std::endl;
    moveActionClient->async_send_goal(goal_msg, sendGoalOptions);
    std::cout << "/async move order issued" << std::endl;
}

void RoboCleanerExternalBridge::moveGoalResponseCallback(std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
    std::cout << "response callback" << std::endl;
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Move goal was rejected by server");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Move goal accepted by server, waiting for result");
}

/**
 * 120 -> small x field marker
 * 88 -> large X field marker
 * 35 -> # field marker
 * 64 -> @ field marker (charging station)
 * 48 -> 0 field marker
 * 49 -> 1 field marker
 * 50 -> 2 field marker
 * 51 -> 3 field marker
 * moveActionClient->async_cancel_all_goals
*/

void RoboCleanerExternalBridge::moveGoalFeedbackCallback(
    [[maybe_unused]] const GoalHandleRobotMove::SharedPtr, 
    const std::shared_ptr<const RobotMove::Feedback> feedback
) {
    std::cout << "feedback tick" << std::endl;
    std::cout << "feedback approaching field marker " << feedback->approaching_field_marker 
            << " percent " << feedback->progress_percent << std::endl;
}

void RoboCleanerExternalBridge::moveGoalResultCallback(const GoalHandleRobotMove::WrappedResult & result) {
    std::cout << "result" << std::endl;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        std::cerr << "Goal was aborted" << std::endl;
        return;
    case rclcpp_action::ResultCode::CANCELED:
        std::cerr << "Goal was aborted" << std::endl;
        return;
    default:
        std::cerr << "Unknown result code: "
                << static_cast<int32_t>(result.code) << std::endl;
        return;
    }

    const uint8_t tileValueAfterMoveCompletion  = result.result->processed_field_marker;
    std::cout << "result result.processedFieldMarker " << result.result->processed_field_marker
        << "result result.success " << result.result->success
        << std::endl;
}