#include <chrono>
#include <iostream>
#include <random>
#include <thread>
#include "robo_cleaner_controller/robo_cleaner_external_bridge.h"
#include "robo_cleaner_controller/coordinate_remapper.h"
#include "robo_cleaner_controller/route_planner.h"
#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"

using namespace std::literals;
using namespace std::placeholders;

static void printVector(const std::vector<Coordinate>& path) {
  for (const auto& c : path) {
    std::cout << c.toString() << " ";
  }
  std::cout << std::endl;
}

namespace {
    constexpr auto GO_FORWARD = 0;
    constexpr auto TURN_LEFT = 1;
    constexpr auto TURN_RIGHT = 2;

    constexpr auto CLOCKWISE_FORWARD_INDEX = 0;
    constexpr auto CLOCKWISE_RIGHT_INDEX = 1;
    constexpr auto CLOCKWISE_BEHIND_INDEX = 2;
    constexpr auto CLOCKWISE_LEFT_INDEX = 3;
    constexpr std::array<uint8_t, 4> CLOCKWISE_DIRECTION_INDEXES = {CLOCKWISE_FORWARD_INDEX, CLOCKWISE_RIGHT_INDEX, CLOCKWISE_BEHIND_INDEX, CLOCKWISE_LEFT_INDEX};
}

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

static RobotDirection calculateDirectionBasedOnCurrentDirectionAndTurnCommand(RobotDirection currentDirection, StateMachine turnCommand) {
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

static Coordinate calculateNewCoordianteBasedOnDirection(RobotDirection currentDirection, Coordinate oldCoordinate) {
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

static Coordinate calculateNewCoordianteBasedOnDirectionAndTurnIndex(RobotDirection currentDirection, Coordinate oldCoordinate, uint8_t clockwiseTurnIndex) {
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

std::vector<Coordinate> RoboCleanerExternalBridge::getClockwiseCoordinatesAroundMe() {
    // 0 -> forward, 1 -> right, 2 -> back, 3 -> left
    std::vector<Coordinate> retval{};
    for (auto rotationIndex : CLOCKWISE_DIRECTION_INDEXES) {
        Coordinate coordinate = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
            robotState.direction,
            robotState.currentNode->getCoordinate(),
            rotationIndex 
        );
        retval.emplace_back(coordinate);
    }
    return retval;
}

RoboCleanerExternalBridge::RoboCleanerExternalBridge() : Node("CleanerExternalBridge") {}

void RoboCleanerExternalBridge::init() {
    using namespace std::chrono_literals;
    _sharedReferenceToSelf = shared_from_this();

    batteryStatusCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    initialRobotStateClient = create_client<QueryInitialRobotState>(QUERY_INITIAL_ROBOT_STATE_SERVICE);
    waitForClientToBecomeReachable(initialRobotStateClient);

    queryBatteryStatusClient = create_client<QueryBatteryStatus>(QUERY_BATTERY_STATUS_SERVICE);
    queryBatteryStatusClient = create_client<QueryBatteryStatus>(QUERY_BATTERY_STATUS_SERVICE, 
      rmw_qos_profile_services_default,
      batteryStatusCallbackGroup);
    waitForClientToBecomeReachable(queryBatteryStatusClient);

    chargeBatteryClient = create_client<ChargeBattery>(CHARGE_BATTERY_SERVICE,
      rmw_qos_profile_services_default,
      batteryStatusCallbackGroup);
    waitForClientToBecomeReachable(chargeBatteryClient);

    moveActionClient = rclcpp_action::create_client<RobotMove>(this, ROBOT_MOVE_ACTION);
    waitForAction(moveActionClient, ROBOT_MOVE_ACTION);

}

void RoboCleanerExternalBridge::clean() {
    queryInitialState();
    timer = create_wall_timer(1000ms, std::bind(&RoboCleanerExternalBridge::timerCallback, this));
}

bool isTileDirty(unsigned char tile)  {
  return tile == '1' || tile == '2' || tile == '3';
}

// TODO READ ALL OF THESE TO GET A CLUE OF WHAT TO DO
// 1. calculate potential coordinates around you and check which ones haven't been visited (keep these in a list, you'll need them later on in 3.)
// 2. once your battery is at a certain threshold [can be a function for now (70% remaining), will implement later],
// do a shortest path and retutrn to the charging station and chanrge to full
// 3. then do a shortest path to closest undiscovered coordinate and carry on 1. fro mthere
// TODO try not to issue orders to go to coordinates you know are collisions
// TODO maybe remove the mutex?
void RoboCleanerExternalBridge::timerCallback() {
    std::lock_guard<std::recursive_mutex> l{actionLock};
    waitForAction(moveActionClient, ROBOT_MOVE_ACTION);
    std::cout << "tick map" << std::endl;
    std::cout << coordinate_remapper::graphToMatrix(map).toString() << std::endl;
    std::cout << "tick state " << std::endl;
    std::cout << robotState.toString() << std::endl;
    std::cout << "should recharge " << shouldRecharge() << std::endl;

    if (isActionRunning) {
      // std::cout << "----- [NO] timer action running" << std::endl;
      return;
    }



    switch (robotState.action) {
    case StateMachine::IDLE:
      {
        if (shouldRecharge() && robotState.currentNode->getBlockType() == map_graph::CHARGING_STATION_COORDINATE) {
          chargeBatteryToFull();
          break;
        }

        // If we should recharge, issue a routing plan and leave the shortestPathWalker to navigate to the charging station
        if (shouldRecharge() && !isWalkingTowardsChargingStation) {
          shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
          isWalkingTowardsChargingStation = true;
          break;
        }

        // Most places around us have been visited; need to go somewhere unknown
        bool successfulNavigationToNearbyCoordinate = false;
        while (shortestPathWalker.hasUntracedCoordinates() && !successfulNavigationToNearbyCoordinate) {
          std::cout << "has unvisited coordinates" << std::endl;
          robotState.action = StateMachine::IDLE;
          successfulNavigationToNearbyCoordinate = goToCoordinate(shortestPathWalker.getNextCoordinate());
        }
        if (successfulNavigationToNearbyCoordinate) {
          break;
        }

        const auto coordinatesAroundMe = getClockwiseCoordinatesAroundMe();
        // FORWARD
        const auto potentialForwardNodeAtCoordinate = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_FORWARD_INDEX]);
        if (!potentialForwardNodeAtCoordinate) {
          goForward();
          break;
        }
        if (isTileDirty((*potentialForwardNodeAtCoordinate)->getBlockType())) {
          goForward();
          break;
        }
        
        // RIGHT
        const auto potentialRightNodeAtCoordinate = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_RIGHT_INDEX]);
        if (!potentialRightNodeAtCoordinate) {
          goRight();
          break;
        }
        if (isTileDirty((*potentialRightNodeAtCoordinate)->getBlockType())) {
          goRight();
          break;
        }
        
        // LEFT
        const auto potentialLeftNodeAtCoordinate = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_LEFT_INDEX]);
        if (!potentialLeftNodeAtCoordinate) {
          goLeft();
          break;
        }
        if (isTileDirty((*potentialLeftNodeAtCoordinate)->getBlockType())) {
          goLeft();
          break;
        }

        // Most places around us have been visited; need to go somewhere unknown
        shortestPathWalker = route_planner::calculateRouteToClosestUnexploredCoordiante(map, robotState.currentNode->getCoordinate());
        while (!shortestPathWalker.isValid()) {
          std::cout << "trying a different shortest path" << std::endl;
          std::cout << robotState.toString() << std::endl;
          std::cout << "[[[[]]]]]" << std::endl;
          shortestPathWalker = route_planner::calculateRouteToClosestUnexploredCoordiante(map, robotState.currentNode->getCoordinate());
        }
        std::cout << "shortest path set" << std::endl;
        // Navigate to closest unvisited coordinate
        // Remap discovered coordinates using top left corner into a matrix so that the shortest path algorithm can be used
        // then do a reverse mapping into the original coordinate system and naviate to the closest uncleaned node
      }
      break;
    case StateMachine::TURNING_RIGHT:
      goRight();
      break;
    case StateMachine::TURNED_RIGHT_GOING_FORWARD:
      std::cout << "turned right, so we need to go forward" << std::endl;
      issueMoveOrder(GO_FORWARD);
      break;
    case StateMachine::TURNED_LEFT_GOING_FORWARD:
      std::cout << "turned left, so we need to go forward" << std::endl;
      issueMoveOrder(GO_FORWARD);
      break;
    case StateMachine::SECOND_HALF_OF_TURN:
      issueMoveOrder(TURN_RIGHT);
    default:
      break;
    }
}

bool RoboCleanerExternalBridge::goToCoordinate(const Coordinate& targetCoord) {
  std::cout << "go to " << targetCoord.toString() << std::endl;
  std::cout << "current state" << std::endl;
  std::cout << robotState.toString() << std::endl;
  std::cout << "+++" << std::endl;
  if (robotState.currentNode->getCoordinate() == targetCoord) {
    std::cout << "same" << std::endl;
    return false;
  }

  // find direction of coordinate
  const auto coordinatesAroundMe = getClockwiseCoordinatesAroundMe();

  if (coordinatesAroundMe[CLOCKWISE_FORWARD_INDEX] == targetCoord) {
    std::cout << "goToCoordinate forward" << std::endl;
    goForward();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_LEFT_INDEX] == targetCoord) {
    std::cout << "goToCoordinate left" << std::endl;
    goLeft();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_RIGHT_INDEX] == targetCoord) {
    std::cout << "goToCoordinate right" << std::endl;
    goRight();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_BEHIND_INDEX] == targetCoord) {
    std::cout << "goToCoordinate behind" << std::endl;
    turnAround();
    return true;
  }

  std::cout << "goToCoordinate neither? but the current is " << robotState.currentNode->getCoordinate().toString() << std::endl;
  printVector(coordinatesAroundMe);
  return true;
}

void RoboCleanerExternalBridge::turnAround() {
  std::cout << "issue turn around command" << std::endl;
  robotState.action = StateMachine::FIRST_HALF_OF_TURN;
  issueMoveOrder(TURN_RIGHT);
}

void RoboCleanerExternalBridge::queryInitialState() {
    std::cout << "calling get initial state" << std::endl;
    auto request = std::make_shared<QueryInitialRobotState::Request>();
    auto result = initialRobotStateClient->async_send_request(request);
    std::cout << "initial state waiting" << std::endl;

    if (rclcpp::spin_until_future_complete(_sharedReferenceToSelf, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get initial robot state");
        throw std::runtime_error("Failed to get initial robot state");
    } 
    std::cout << "initial state complete" << std::endl;

    const std::shared_ptr<QueryInitialRobotState::Response> responseData = result.get();
    const auto batteryStatus = responseData->initial_robot_state.battery_status;
    const auto direction = responseData->initial_robot_state.robot_dir;
    const auto tileDirtiness = responseData->initial_robot_state.robot_tile;
    robotState.direction = robot_state::numberToDirection(direction);
    robotState.movesLeft = batteryStatus.moves_left;
    robotState.maxMovesOnFullEnergy = batteryStatus.max_moves_on_full_energy;

    auto newNode = std::make_shared<GraphNode>(GraphNode(Coordinate(0, 0), tileDirtiness));
    robotState.currentNode = newNode;
    map.addNode(newNode);
}

void RoboCleanerExternalBridge::issueMoveOrder(int8_t moveType) {
    std::lock_guard<std::recursive_mutex> l{actionLock};
    if (isActionRunning) {
      std::cout << "can't issue move order because action is running" << std::endl;
      return;
    }
    isActionRunning = true;
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
    std::cout << "issuing move order " << static_cast<unsigned char>(moveType) << std::endl;
    moveActionClient->async_send_goal(goal_msg, sendGoalOptions);
    std::cout << "/async move order issued " << static_cast<unsigned char>(moveType) << std::endl;
}

void RoboCleanerExternalBridge::moveGoalResponseCallback(std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
    std::lock_guard<std::recursive_mutex> l{actionLock};
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


bool isApproachingCollision(uint8_t fieldMarker) {
    return fieldMarker == '#' || fieldMarker == 'x' || fieldMarker == 'X';
} 

void RoboCleanerExternalBridge::moveGoalFeedbackCallback(
    [[maybe_unused]] const GoalHandleRobotMove::SharedPtr, 
    const std::shared_ptr<const RobotMove::Feedback> feedback
) {
  std::lock_guard<std::recursive_mutex> l{actionLock};
  if (isApproachingCollision(feedback->approaching_field_marker)) {
    moveActionClient->async_cancel_all_goals();
    std::cout << "cancelling goal due to approaching collision " << feedback->approaching_field_marker << std::endl;
    const auto newCoordinate = calculateNewCoordianteBasedOnDirection(robotState.direction, robotState.currentNode->getCoordinate());
    auto newNode = std::make_shared<GraphNode>(newCoordinate, map_graph::COLLISION_COORDINATE);
    map.addNode(newNode);
    this->isActionRunning = false;
    this->robotState.action = StateMachine::IDLE;

    // If we're trying to navigate using the route planner but hit an obstacle, we need to recalculate the route based on the new information
    if (shortestPathWalker.hasUntracedCoordinates()) {
      if (shouldRecharge()) {
        shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
      } else {
        shortestPathWalker = route_planner::calculateRouteToClosestUnexploredCoordiante(map, robotState.currentNode->getCoordinate());
      }
    }
    return;
  }
}

void RoboCleanerExternalBridge::moveGoalResultCallback(const GoalHandleRobotMove::WrappedResult & result) {
    std::lock_guard<std::recursive_mutex> l{actionLock};
    std::cout << "result" << std::endl;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        isActionRunning = false;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        isActionRunning = false;
        std::cerr << "Goal was aborted" << std::endl;
        return;
    case rclcpp_action::ResultCode::CANCELED:
        isActionRunning = false;
        std::cerr << "Goal was aborted" << std::endl;
        return;
    default:
        isActionRunning = false;
        std::cerr << "Unknown result code: "
                << static_cast<int32_t>(result.code) << std::endl;
        return;
    }

    updateBatteryStatus();
    switch (robotState.action)
    {
    case StateMachine::TURNING_RIGHT:
      robotState.direction = calculateDirectionBasedOnCurrentDirectionAndTurnCommand(robotState.direction, StateMachine::TURNING_RIGHT);
      robotState.action = StateMachine::TURNED_RIGHT_GOING_FORWARD;
      break;
    case StateMachine::TURNING_LEFT:
      robotState.direction = calculateDirectionBasedOnCurrentDirectionAndTurnCommand(robotState.direction, StateMachine::TURNING_LEFT);
      robotState.action = StateMachine::TURNED_LEFT_GOING_FORWARD;
      break;
    case StateMachine::TURNED_RIGHT_GOING_FORWARD:
    case StateMachine::TURNED_LEFT_GOING_FORWARD:
    case StateMachine::GOING_FORWARD:
      robotState.action = StateMachine::IDLE; // means we need to pick a new location to go to

      if (result.result->success) {
        auto newNode = std::make_shared<GraphNode>(calculateNewCoordianteBasedOnDirection(robotState.direction, robotState.currentNode->getCoordinate()), result.result->processed_field_marker);
        auto addedNode = map.addNode(newNode); 
        robotState.currentNode = addedNode;
      } else {
        std::cout << "result is not success" << std::endl;
        auto newNode = std::make_shared<GraphNode>(calculateNewCoordianteBasedOnDirection(robotState.direction, robotState.currentNode->getCoordinate()), 'X');
        map.addNode(newNode); 
      }
      break;
    case StateMachine::IDLE:
      std::cout << "wat idle" << std::endl;
      throw std::runtime_error("compile plz");
      break;
    case StateMachine::FIRST_HALF_OF_TURN:
      std::cout << "first half of turn around done; state before it" << std::endl;
      std::cout << robotState.toString() << std::endl;
      std::cout << ">>>" << std::endl;
      robotState.direction = calculateDirectionBasedOnCurrentDirectionAndTurnCommand(robotState.direction, StateMachine::TURNING_RIGHT);
      robotState.action = StateMachine::TURNING_RIGHT;
      std::cout << "first half of turn around done; state after it" << std::endl;
      std::cout << robotState.toString() << std::endl;
      std::cout << ">>>" << std::endl;
      break;
    default:
      throw std::runtime_error("compile plz");
      break;
  }
  isActionRunning = false;
}

void RoboCleanerExternalBridge::goRight() {
  robotState.action = StateMachine::TURNING_RIGHT;
  issueMoveOrder(TURN_RIGHT);
}

void RoboCleanerExternalBridge::goLeft() {
  robotState.action = StateMachine::TURNING_LEFT;
  issueMoveOrder(TURN_LEFT);
}

void RoboCleanerExternalBridge::goForward() {
  robotState.action = StateMachine::GOING_FORWARD;
  issueMoveOrder(GO_FORWARD);
}

void RoboCleanerExternalBridge::updateBatteryStatus() {
  std::lock_guard<std::mutex>(this->batteryLock);
  auto request = std::make_shared<QueryBatteryStatus::Request>();
  auto response = this->queryBatteryStatusClient->async_send_request(request);

  const auto result = response.get();
  this->robotState.movesLeft = result->battery_status.moves_left;
  this->robotState.maxMovesOnFullEnergy = result->battery_status.max_moves_on_full_energy;
}

void RoboCleanerExternalBridge::chargeBatteryToFull() {
  std::cout << "charging battery to full" << std::endl;
  std::lock_guard<std::mutex>(this->batteryLock);
  auto request = std::make_shared<ChargeBattery::Request>();
  request->charge_turns = ChargeBattery::Request::CHARGE_UNTIL_FULL;
  auto response = this->chargeBatteryClient->async_send_request(request);

  const auto result = response.get();
  this->robotState.movesLeft = result->battery_status.moves_left;
  this->robotState.maxMovesOnFullEnergy = result->battery_status.max_moves_on_full_energy; 
  isWalkingTowardsChargingStation = false;
  std::cout << "received recharge resoponse " << std::endl;
  std::cout << robotState.toString() << std::endl;
  std::cout << "/received recharge resoponse " << std::endl;
  robotState.action = StateMachine::IDLE;
}

bool RoboCleanerExternalBridge::shouldRecharge() {
  std::lock_guard<std::mutex>(this->batteryLock);
  return static_cast<float>(robotState.movesLeft) / static_cast<float>(robotState.maxMovesOnFullEnergy) < 0.55;
}

bool RoboCleanerExternalBridge::isChargeFull() {
  std::lock_guard<std::mutex>(this->batteryLock);
  return static_cast<float>(robotState.movesLeft) / static_cast<float>(robotState.maxMovesOnFullEnergy) == 1.0;
}