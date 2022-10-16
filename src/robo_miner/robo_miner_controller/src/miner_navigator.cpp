#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"
#include <functional>

namespace {
    using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
    constexpr auto LEFT_INDEX = 0;
    constexpr auto FORWARD_INDEX = 1;
    constexpr auto RIGHT_INDEX = 2;
}

static RobotDirection toDirection(int8_t d) {
  switch (d)
  {
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

static RobotDirection getDirectionOnLeftTurn(RobotDirection currentDirection) {
  switch (currentDirection)
  {
  case RobotDirection::LEFT:
    return RobotDirection::DOWN;
  case RobotDirection::DOWN:
    return RobotDirection::RIGHT;
  case RobotDirection::RIGHT:
    return RobotDirection::UP;
  case RobotDirection::UP:
    return RobotDirection::LEFT;
  default:
    return RobotDirection::UNKNOWN;
  }
}

static RobotDirection getDirectionOnRightTurn(RobotDirection currentDirection) {
  switch (currentDirection)
  {
  case RobotDirection::LEFT:
    return RobotDirection::UP;
  case RobotDirection::UP:
    return RobotDirection::RIGHT;
  case RobotDirection::RIGHT:
    return RobotDirection::DOWN;
  case RobotDirection::DOWN:
    return RobotDirection::LEFT;
  default:
    return RobotDirection::UNKNOWN;
  }
}

MinerNavigator::MinerNavigator(
    MoverCommunicator&& moverCommunicator,
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient
) : moverCommunicator{moverCommunicator}, node{node}, initialRobotPositionClient{initialRobotPositionClient} {

  mapGraph = MapGraph{};
  coordinatesTrail = std::stack<Coordinate>{};
}

void MinerNavigator::init() {
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();
  auto result = initialRobotPositionClient->async_send_request(request);

  // Wait for the result. TODO error handling
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 

  const std::shared_ptr<QueryInitialRobotPosition::Response> responseData = result.get();
  uint8_t initialTile = responseData->robot_initial_tile;
  const RobotPositionResponse robotPositionResponse = responseData->robot_position_response;

  
  const auto initialNode = std::make_shared<GraphNode>(GraphNode(Coordinate(0, 0), initialTile));
  initialNode->markVisited();
  mapGraph.addNode(initialNode);

  robotState.direction = toDirection(robotPositionResponse.robot_dir);
  robotState.surroundingTiles = robotPositionResponse.surrounding_tiles;
  robotState.currentNode = initialNode;

  coordinatesTrail.push(Coordinate(0, 0));
  std::cout << "initial state " << robotState.toString() << std::endl;
}

std::vector<std::pair<uint8_t, Coordinate>> MinerNavigator::getValidMovementCoordinates() {
  std::vector<std::pair<uint8_t, Coordinate>> retval{};
  std::vector<uint8_t> possibleDirections{};
  for (int i = 0; i < 3; i++) {
    if (robotState.surroundingTiles[i] != 'X' && robotState.surroundingTiles[i] != '#') {
      std::cout << "picked surrounding tile index " << i << " because the tile value was " << robotState.surroundingTiles[i] << " ";
      possibleDirections.emplace_back(i);
    }
  }
  std::cout << std::endl;

  std::cout << " robot state  " << robotState.toString() << std::endl;

  for (const int8_t idxOfNonCollisionTile: possibleDirections) {
      if (idxOfNonCollisionTile == LEFT_INDEX) {
          auto oldCoordinate = robotState.currentNode->getCoordinate();
          auto potentialDirection = getDirectionOnLeftTurn(robotState.direction);
          const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
          std::cout << "valid movement coordinates left " << static_cast<int>(idxOfNonCollisionTile) << std::endl;
          retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
      if (idxOfNonCollisionTile == FORWARD_INDEX) {
        auto oldCoordinate = robotState.currentNode->getCoordinate();
        auto potentialDirection = robotState.direction;
        const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
        std::cout << "valid movement coordinates forward " << static_cast<int>(idxOfNonCollisionTile) << std::endl;
        retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
      if (idxOfNonCollisionTile == RIGHT_INDEX) {
        auto oldCoordinate = robotState.currentNode->getCoordinate();
        auto potentialDirection = getDirectionOnRightTurn(robotState.direction);
        const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
        std::cout << "valid movement coordinates right " << static_cast<int>(idxOfNonCollisionTile) << std::endl;
        retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
  }
  return retval;
}

std::vector<std::function<void()>> MinerNavigator::getActionVector() {
  std::vector<std::function<void()>> retval{};
  auto validMovementCoordinates = getValidMovementCoordinates();

  for (const auto& thing : validMovementCoordinates) {
    const auto idxOfNonCollisionTile = thing.first;
    const auto coordiante = thing.second;
    if (idxOfNonCollisionTile == LEFT_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordiante)) {
        const auto leftFunction = [this]() {
          this->goLeft();
        };
        retval.emplace_back(leftFunction);
      }
    }

    if (idxOfNonCollisionTile == FORWARD_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordiante)) {
        const auto forwardFunction = [this]() {
          this->goForward();
        };
        retval.emplace_back(forwardFunction);
      }
    }

    if (idxOfNonCollisionTile == RIGHT_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordiante)) {
        const auto rightFunction = [this]() {
          this->goRight();
        };
        retval.emplace_back(rightFunction);
      }
    }
  }
  return retval;
}

std::vector<uint8_t> MinerNavigator::pickNonCollisionTileIndex() {
  std::vector<uint8_t> retval{};
  for (int i = 0; i < 3; i++) {
    if (robotState.surroundingTiles[i] != 'X' && robotState.surroundingTiles[i] != '#') {
      std::cout << "picked surrounding tile index " << i << " because the tile value was " << robotState.surroundingTiles[i] << " ";
      retval.emplace_back(i);
    }
  }
  std::cout << std::endl;
  return retval;
}

void MinerNavigator::exploreMap() {
  for (int i = 0; i < 100; i++) {
    auto possibleActions = getActionVector();
    // We are in some corner and can't go forward, left or right.. Somehow we got to the corner, though, so the only way out is back
    if (possibleActions.empty()) {
      backtrackUntilUnstuck();
      continue;
    }

    // Execute first valid action
    possibleActions[0]();

    // auto possibleIndexes = pickNonCollisionTileIndex();
    // // We are in some corner and can't go forward, left or right.. Somehow we got to the corner, though, so the only way out is back
    // if (possibleIndexes.empty()) {
    //   backtrackUntilUnstuck();
    // }
    // bool managedToMove = false;
    // for (const auto& idxOfNonCollisionTile: possibleIndexes) {
    //   if (idxOfNonCollisionTile == LEFT_INDEX) {
    //       auto oldCoordinate = robotState.currentNode->getCoordinate();
    //       auto potentialDirection = getDirectionOnLeftTurn(robotState.direction);
    //       const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
    //       if (!mapGraph.hasCoordinateBeenVisited(newCoordinate)) {
    //         goLeft();
    //         managedToMove = true;
    //         break;
    //       }
    //   }
    //   if (idxOfNonCollisionTile == FORWARD_INDEX) {
    //     auto oldCoordinate = robotState.currentNode->getCoordinate();
    //     auto potentialDirection = robotState.direction;
    //     const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
    //     if (!mapGraph.hasCoordinateBeenVisited(newCoordinate)) {
    //       goForward();
    //       managedToMove = true;
    //       break;
    //     }
    //   }
    //   if (idxOfNonCollisionTile == RIGHT_INDEX) {
    //     auto oldCoordinate = robotState.currentNode->getCoordinate();
    //     auto potentialDirection = getDirectionOnRightTurn(robotState.direction);
    //     const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
    //     if (!mapGraph.hasCoordinateBeenVisited(newCoordinate)) {
    //       goRight();
    //       managedToMove = true;
    //       break;
    //     }
    //   }
    // }

    // //
    // if (!managedToMove) {
    //   backtrackUntilUnstuck();
    // }
  }

  std::cout << "Map" << std::endl;
  std::cout << mapGraph.toString() << std::endl;

  std::cout << "Coordinates trail" << std::endl;
  while (!coordinatesTrail.empty()) {
    Coordinate coord = coordinatesTrail.top();
    coordinatesTrail.pop();
    std::cout << coord.toString() << std::endl;
  }
}

// bool hasUnvisitedCoordinates(const MapGraph& graph, const std::vector<std::pair<uint8_t, Coordinate>> coordinates) {
bool hasAtLeastOneUnvisitedCoordiante(const MapGraph& graph, const std::vector<std::pair<uint8_t, Coordinate>> coordinates) {
  for (const auto& possibility : coordinates) {
    const auto coordinate = possibility.second;
    if (!graph.hasCoordinateBeenVisited(coordinate)) {
      return true;
    }
  }
  return false;
}

// TODO search for all references to thing
std::pair<int32_t, Coordinate> getCoordinateOnbacktrackPath(
  const std::vector<std::pair<uint8_t, Coordinate>>& coordinates,
  const std::stack<Coordinate>& backtrackPath
  // RobotState& robotState,
  // MoverCommunicator& moverCommunicator
) {
  const auto topCoordinate = backtrackPath.top();
  std::cout << "---" << std::endl;
  std::cout << "top coordinate is " << topCoordinate.toString() << std::endl;
  for (const auto& thing : coordinates) {
    std::cout << "available coordinate " << thing.second.toString() << " at index " << static_cast<int>(thing.first) << std::endl;
  }
  std::cout << "---" << std::endl;
  for (int i = 0; i < 4; i++) {
    // Try to backtrack with what you see
    for (const auto thing : coordinates) {
      const auto coordinate = thing.second;
      if (topCoordinate.x == coordinate.x && topCoordinate.y == coordinate.y) {
        return thing;
      }
    }
    // If you can't find the backtrack coordinate, turn until you do
    // auto result = moverCommunicator.sendTurnRightCommand();
    // robotState.direction = toDirection(result->robot_position_response.robot_dir);
    // robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  }
  // throw std::invalid_argument("no valid location to go to found");
  return std::make_pair(-1, Coordinate(0, 0));
}

void printStack(std::stack<Coordinate> stackCopy) {
  std::cout << "stack copy address " << &stackCopy << std::endl;
  while (!stackCopy.empty()) {
    std::cout << stackCopy.top().toString() << " ";
    stackCopy.pop();
  }
  std::cout << std::endl;
}

void MinerNavigator::backtrackUntilUnstuck() {
  std::cout << "start backtracking" << std::endl;
  auto validMovementCoordinates = getValidMovementCoordinates();

  // First turn around
  if (validMovementCoordinates.empty()) {
    auto result = moverCommunicator.sendTurnRightCommand();
    robotState.direction = toDirection(result->robot_position_response.robot_dir);
    robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
    std::cout << "turning right, state updated to" << robotState.toString() << std::endl;
    result = moverCommunicator.sendTurnRightCommand();
    robotState.direction = toDirection(result->robot_position_response.robot_dir);
    robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
    std::cout << "turning right, state updated to" << robotState.toString() << std::endl;  
  }

  // Next, eliminate current node from stack we want to be comparing only to things that came before it
  std::cout << "stack address before " << &coordinatesTrail << std::endl;
  std::cout << "stack before pop last" << std::endl;
  printStack(coordinatesTrail);
  coordinatesTrail.pop();
  std::cout << "stack after pop last" << std::endl;
  printStack(coordinatesTrail);

  while (true) {
    validMovementCoordinates = getValidMovementCoordinates();
    bool doneBacktracking = hasAtLeastOneUnvisitedCoordiante(mapGraph, validMovementCoordinates);
    if (doneBacktracking) {
      std::cout << "done backtracking" << std::endl;
      break;
    }

    auto backtrackObject = getCoordinateOnbacktrackPath(validMovementCoordinates, coordinatesTrail);
    // If you can't find the right direction using your current orientation, you need to turn
    while (backtrackObject.first == -1) {
      auto result = moverCommunicator.sendTurnRightCommand();
      robotState.direction = toDirection(result->robot_position_response.robot_dir);
      robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
      validMovementCoordinates = getValidMovementCoordinates();
      backtrackObject = getCoordinateOnbacktrackPath(validMovementCoordinates, coordinatesTrail);
    }
    auto index = backtrackObject.first;
    auto coordinate = backtrackObject.second;
    std::cout << "backtacking to " << static_cast<int>(index) << " at coordinate " << coordinate.toString() << std::endl;;

    if (index == LEFT_INDEX) {
      std::cout << "backtracking direction LEFT" << std::endl;
      goLeft();
      std::cout << "stack after pop direction left " << std::endl;
      printStack(coordinatesTrail);
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      std::cout << "stack address before " << &coordinatesTrail << std::endl;
      std::cout << "stack after pop direction left " << std::endl;
      printStack(coordinatesTrail);
      continue;
    }

    if (index == FORWARD_INDEX) {
      std::cout << "backtracking direction FORWARD" << std::endl;
      goForward();
      std::cout << "stack before pop direction forward" << std::endl;
      printStack(coordinatesTrail);
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      std::cout << "stack address before " << &coordinatesTrail << std::endl;
      std::cout << "stack after pop direction forward" << std::endl;
      printStack(coordinatesTrail);
      continue;
    }

    if (index == RIGHT_INDEX) {
      std::cout << "backtracking direction RIGHT" << std::endl;
      goRight();
      std::cout << "stack before pop direction right" << std::endl;
      printStack(coordinatesTrail);
      // TODO nasty hack but needed because each goForward adds onto the stack
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      std::cout << "stack address before " << &coordinatesTrail << std::endl;
      std::cout << "stack after pop direction right" << std::endl;
      printStack(coordinatesTrail);
      continue;
    }
  }
}

void MinerNavigator::goForward() {
  auto tileType = robotState.surroundingTiles[FORWARD_INDEX];
  auto result = moverCommunicator.sendMoveForwardCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;

  auto currentDirection = robotState.direction;
  auto oldCoordinate = robotState.currentNode->getCoordinate();
  const auto newCoordinate = calculateNewCoordianteBasedOnDirection(currentDirection, oldCoordinate);

  auto newNode = std::make_shared<GraphNode>(GraphNode(newCoordinate, tileType));
  newNode->markVisited();
  mapGraph.addNode(newNode);
  robotState.currentNode = newNode;

  coordinatesTrail.push(newCoordinate);
  std::cout << "moved forward, state updated to " << robotState.toString() << std::endl;
}

void MinerNavigator::goLeft() {
  auto result = moverCommunicator.sendTurnLeftCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "turning left, state updated to" << robotState.toString() << std::endl;
  goForward();
}

void MinerNavigator::goRight() {
  auto result = moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "turning right, state updated to" << robotState.toString() << std::endl;
  goForward();
}

void MinerNavigator::goBack() {
  auto result = moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "turning right, state updated to" << robotState.toString() << std::endl;
  moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  std::cout << "turning right, state updated to" << robotState.toString() << std::endl;
  goForward();
}