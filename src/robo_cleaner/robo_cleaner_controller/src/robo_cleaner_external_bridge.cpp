#include <chrono>
#include <iostream>
#include <random>
#include <thread>
#include "robo_cleaner_controller/robo_cleaner_external_bridge.h"
#include "robo_cleaner_controller/coordinate_remapper.h"
#include "robo_cleaner_controller/route_planner.h"
#include "robo_cleaner_controller/direction_coordinate_calculator.h"
#include "robo_cleaner_common/defines/RoboCleanerTopics.h"
#include "robo_cleaner_interfaces/msg/robot_move_type.hpp"
#include "utils/Log.h"

using namespace std::literals;
using namespace std::placeholders;

static void printVector(const std::vector<Coordinate>& path) {
  LOG("printing vector");
  for (const auto& c : path) {
    std::cout << c.toString() << " ";
  }
  std::cout << std::endl;
}

namespace {
    constexpr auto GO_FORWARD = 0;
    constexpr auto TURN_LEFT = 1;
    constexpr auto TURN_RIGHT = 2;
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

void publishUserAuthenticate(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<rclcpp::Publisher<robo_cleaner_interfaces::msg::UserAuthenticate>> userAuthenticatePublisher
) {
  robo_cleaner_interfaces::msg::UserAuthenticate userAuthenticate;
  userAuthenticate.repository = "https://github.com/ZeppelinCode/robotics_v1"; //userParams.repository;
  userAuthenticate.user = "Kristian Sonev"; // userParams.user;
  userAuthenticate.commit_sha = "c59d454827fe48f6e947256d34d7f36a4e31b9a7"; // userParams.commitSha;
  userAuthenticatePublisher->publish(userAuthenticate);
}

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

    constexpr size_t queueSize = 10;
    const rclcpp::QoS qos(queueSize);
    userAuthenticatePublisher = create_publisher<UserAuthenticate>(USER_AUTHENTICATE_TOPIC, qos);
}

// [CRASH] Executing action server but nothing is ready

void RoboCleanerExternalBridge::clean() {
    queryInitialState();
    publishUserAuthenticate(_sharedReferenceToSelf, userAuthenticatePublisher);
    // timer = create_wall_timer(1511ms, std::bind(&RoboCleanerExternalBridge::timerCallback, this));
    timer = create_wall_timer(623ms, std::bind(&RoboCleanerExternalBridge::timerCallback, this));
}

bool isTileDirty(unsigned char tile)  {
  return tile == '1' || tile == '2' || tile == '3';
}

bool RoboCleanerExternalBridge::onlyOneSpotLeftAndImOnIt() {
  int numberOfSpotsLeft = 0;
  Coordinate spot = Coordinate(std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::min());
  for (const auto& node : map.getNodes()) {
    if (node->getBlockType() >= '1' && node->getBlockType() <= '3') {
      numberOfSpotsLeft += 1;
      spot = node->getCoordinate();
    }
    if (numberOfSpotsLeft > 1) {
      LOG("spots left %d; EXITING", numberOfSpotsLeft);
      return false;
    }
  }
  LOG("spots left %d, me: %s, spot: %s, on it: %d", 
    numberOfSpotsLeft,
    robotState.currentNode->getCoordinate().toString().c_str(),
    spot.toString().c_str(),
    spot == robotState.currentNode->getCoordinate()
    );
  return numberOfSpotsLeft == 1 && spot == robotState.currentNode->getCoordinate();
}

bool nothingLeftToExplore(const MapGraph& map) {
  int toExplore = 0;
  for (const auto& node: map.getNodes()) {
    if (node->getBlockType() == '#' || (node->getBlockType() >= '1' && node->getBlockType() <= '3')) {
      toExplore += 1;
    }
  }

  return toExplore == 0 && map.getNodes().size() > 5;
}

void RoboCleanerExternalBridge::timerCallback() {
    std::lock_guard<std::recursive_mutex> l{actionLock};
    waitForAction(moveActionClient, ROBOT_MOVE_ACTION);
    LOG("tick map");
    LOG("%s", coordinate_remapper::graphToMatrix(map).toString().c_str());
    LOG("tick state");
    LOG("%s", robotState.toString().c_str());
    LOG("should recharge %d", shouldRecharge());

    if (isActionRunning) {
      LOG("----- [NO] timer action running");
      return;
    }

    if (nothingLeftToExplore(map) && !isWalkingTowardsChargingStation) {
      shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
      isWalkingTowardsChargingStation = true;
      robotState.action = StateMachine::IDLE;
      return;
    }

    if (shouldRecharge() && robotState.currentNode->getBlockType() == map_graph::CHARGING_STATION_COORDINATE) {
      chargeBatteryToFull();
      return;
    }

    // If we should recharge, issue a routing plan and leave the shortestPathWalker to navigate to the charging station
    if (shouldRecharge() && !isWalkingTowardsChargingStation) {
      shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
      isWalkingTowardsChargingStation = true;
      robotState.action = StateMachine::IDLE;
      return;
    }

    switch (robotState.action) {
    case StateMachine::IDLE:
      {

        // If we should recharge, issue a routing plan and leave the shortestPathWalker to navigate to the charging station
        if (shouldRecharge() && !isWalkingTowardsChargingStation) {
          shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
          isWalkingTowardsChargingStation = true;
          break;
        }

        // Most places around us have been visited; need to go somewhere unknown
        bool successfulNavigationToNearbyCoordinate = false;
        while (shortestPathWalker.hasUntracedCoordinates() && !successfulNavigationToNearbyCoordinate) {
          LOG("has unvisited coordinates");
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

        // BEHIND
        const auto potentialBehindNodeAtCoordinate = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_BEHIND_INDEX]);
        if (!potentialBehindNodeAtCoordinate) {
          turnAround();
          break;
        }
        if (isTileDirty((*potentialBehindNodeAtCoordinate)->getBlockType())) {
          turnAround();
          break;
        }

        // Only one dirty tile left, we need to "scrub" it
        if (shouldRecharge()) {
          shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
          isWalkingTowardsChargingStation = true;
          break;
        }

        if (onlyOneSpotLeftAndImOnIt()) {
          scrubLastSpot();
          break;
        }

        if (shouldRecharge()) {
          shortestPathWalker = route_planner::calculateRouteToCharingStation(map, robotState.currentNode->getCoordinate());
          isWalkingTowardsChargingStation = true;
          break;
        }

        // Everything around me is either clean or a collision => go to closest known unexplored or dirty coordinate
        LOG("couldn't go in any direction, resorting to route planner");
        shortestPathWalker = route_planner::calculateRouteToClosestUnexploredCoordiante(map, robotState.currentNode->getCoordinate());
        LOG("shortest path set");
      }
      break;
    case StateMachine::TURNING_RIGHT:
      goRight();
      break;
    case StateMachine::TURNED_RIGHT_GOING_FORWARD:
      LOG("turned right, so we need to go forward");
      issueMoveOrder(GO_FORWARD);
      break;
    case StateMachine::TURNED_LEFT_GOING_FORWARD:
      LOG("turned left, so we need to go forward");
      issueMoveOrder(GO_FORWARD);
      break;
    case StateMachine::SECOND_HALF_OF_TURN:
      issueMoveOrder(TURN_RIGHT);
    default:
      break;
    }
}

bool RoboCleanerExternalBridge::goToCoordinate(const Coordinate& targetCoord) {
  LOG("go to %s", targetCoord.toString().c_str());
  LOG("current state");
  LOG("%s", robotState.toString().c_str());
  LOG("+++");
  if (robotState.currentNode->getCoordinate() == targetCoord) {
    LOG("same");
    return false;
  }

  // find direction of coordinate
  const auto coordinatesAroundMe = getClockwiseCoordinatesAroundMe();

  if (coordinatesAroundMe[CLOCKWISE_FORWARD_INDEX] == targetCoord) {
    LOG("goToCoordinate forward");
    goForward();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_LEFT_INDEX] == targetCoord) {
    LOG("goToCoordinate left");
    goLeft();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_RIGHT_INDEX] == targetCoord) {
    std::cout << "goToCoordinate right" << std::endl;
    goRight();
    return true;
  }

  if (coordinatesAroundMe[CLOCKWISE_BEHIND_INDEX] == targetCoord) {
    LOG("goToCoordinate behind");
    turnAround();
    return true;
  }

  LOG("goToCoordinate neither? but the current is %s", robotState.currentNode->getCoordinate().toString().c_str());
  printVector(coordinatesAroundMe);
  return true;
}

void RoboCleanerExternalBridge::turnAround() {
  LOG("issue turn around command");
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
      LOG("can't issue move order because action is running");
      return;
    }

    LOG("setting action to running");
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
    LOG("issuing move order %d", static_cast<int>(moveType));
    moveActionClient->async_send_goal(goal_msg, sendGoalOptions);
    LOG("/async move order issued %d", static_cast<int>(moveType));
}

void RoboCleanerExternalBridge::moveGoalResponseCallback(std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
    std::lock_guard<std::recursive_mutex> l{actionLock};
    auto goal_handle = future.get();
    if (!goal_handle) {
      LOGR("move goal was rejected by server");
      return;
    }
    LOGG("Move goal was accepted by server, waiting for resultm id: %s", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
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
    LOG("cancelling goal due to approaching collision %c", feedback->approaching_field_marker);
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
    LOG("move goal result callback");
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        LOG("setting action to not running %s, %d", __FILE__, __LINE__);
        break;
    case rclcpp_action::ResultCode::ABORTED:
        isActionRunning = false;
        LOG("setting action to not running %s, %d", __FILE__, __LINE__);
        return;
    case rclcpp_action::ResultCode::CANCELED:
        isActionRunning = false;
        LOG("setting action to not running %s, %d", __FILE__, __LINE__);
        return;
    default:
        isActionRunning = false;
        LOG("setting action to not running %s, %d", __FILE__, __LINE__);
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
        LOG("result is not success");
        auto newNode = std::make_shared<GraphNode>(calculateNewCoordianteBasedOnDirection(robotState.direction, robotState.currentNode->getCoordinate()), 'X');
        map.addNode(newNode); 
      }
      break;
    case StateMachine::IDLE:
      LOG("wat idle");
      throw std::runtime_error("compile plz");
      break;
    case StateMachine::FIRST_HALF_OF_TURN:
      LOG("first half of turn around done; state before it");
      LOG("%s", robotState.toString().c_str());
      LOG(">>>");
      std::cout << ">>>" << std::endl;
      robotState.direction = calculateDirectionBasedOnCurrentDirectionAndTurnCommand(robotState.direction, StateMachine::TURNING_RIGHT);
      robotState.action = StateMachine::TURNING_RIGHT;
      LOG("first half of turn around done; state after it");
      LOG("%s", robotState.toString().c_str());
      LOG(">>>");
      break;
    default:
      throw std::runtime_error("compile plz");
      break;
  }
  LOG("setting action to not running %s, %d", __FILE__, __LINE__);
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

void RoboCleanerExternalBridge::scrubLastSpot() {
  const auto coordinatesAroundMe = getClockwiseCoordinatesAroundMe();
  const auto inFrontOfMeNode = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_FORWARD_INDEX]);
  if (inFrontOfMeNode && (*inFrontOfMeNode)->getBlockType() == '0') {
    std::vector<Coordinate> scrubPath{};
    for (int scrubTimes = 0; scrubTimes < robotState.currentNode->getBlockType() - 48; scrubTimes++) {
      scrubPath.emplace_back(coordinatesAroundMe[CLOCKWISE_FORWARD_INDEX]);
      scrubPath.emplace_back(robotState.currentNode->getCoordinate());
    }
    shortestPathWalker = ShortestPathWalker(std::move(scrubPath));
    return;
  }

  const auto rightNode = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_RIGHT_INDEX]);
  if (rightNode && (*rightNode)->getBlockType() == '0') {
    std::vector<Coordinate> scrubPath{};
    for (int scrubTimes = 0; scrubTimes < robotState.currentNode->getBlockType() - 48; scrubTimes++) {
      scrubPath.emplace_back(coordinatesAroundMe[CLOCKWISE_RIGHT_INDEX]);
      scrubPath.emplace_back(robotState.currentNode->getCoordinate());
    }
    shortestPathWalker = ShortestPathWalker(std::move(scrubPath));
    return;
  }

  const auto leftNode = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_LEFT_INDEX]);
  if (leftNode && (*leftNode)->getBlockType() == '0') {
    std::vector<Coordinate> scrubPath{};
    for (int scrubTimes = 0; scrubTimes < robotState.currentNode->getBlockType() - 48; scrubTimes++) {
      scrubPath.emplace_back(coordinatesAroundMe[CLOCKWISE_LEFT_INDEX]);
      scrubPath.emplace_back(robotState.currentNode->getCoordinate());
    }
    shortestPathWalker = ShortestPathWalker(std::move(scrubPath));
    return;
  }

  const auto behindNode = map.getNodeAtCoordinate(coordinatesAroundMe[CLOCKWISE_BEHIND_INDEX]);
  if (behindNode && (*behindNode)->getBlockType() == '0') {
    std::vector<Coordinate> scrubPath{};
    for (int scrubTimes = 0; scrubTimes < robotState.currentNode->getBlockType() - 48; scrubTimes++) {
      scrubPath.emplace_back(coordinatesAroundMe[CLOCKWISE_BEHIND_INDEX]);
      scrubPath.emplace_back(robotState.currentNode->getCoordinate());
    }
    shortestPathWalker = ShortestPathWalker(std::move(scrubPath));
    return;
  }
}

// --- BATTERY
void RoboCleanerExternalBridge::chargeBatteryToFull() {
  LOG("charging battery to full");
  std::lock_guard<std::mutex>(this->batteryLock);
  auto request = std::make_shared<ChargeBattery::Request>();
  request->charge_turns = ChargeBattery::Request::CHARGE_UNTIL_FULL;
  auto response = this->chargeBatteryClient->async_send_request(request);

  const auto result = response.get();
  this->robotState.movesLeft = result->battery_status.moves_left;
  this->robotState.maxMovesOnFullEnergy = result->battery_status.max_moves_on_full_energy; 
  isWalkingTowardsChargingStation = false;
  LOG("received recharge resoponse");
  LOG("%s", robotState.toString().c_str());
  LOG("/received recharge resoponse");
  robotState.action = StateMachine::IDLE;
}

bool RoboCleanerExternalBridge::shouldRecharge() {
  std::lock_guard<std::mutex>(this->batteryLock);
  return static_cast<float>(robotState.movesLeft) / static_cast<float>(robotState.maxMovesOnFullEnergy) < 0.52;
}

bool RoboCleanerExternalBridge::isChargeFull() {
  std::lock_guard<std::mutex>(this->batteryLock);
  return static_cast<float>(robotState.movesLeft) / static_cast<float>(robotState.maxMovesOnFullEnergy) == 1.0;
}