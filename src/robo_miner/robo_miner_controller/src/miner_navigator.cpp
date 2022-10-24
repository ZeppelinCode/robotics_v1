#include <functional>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>

#include "robo_miner_controller/miner_navigator.h"
#include "robo_miner_controller/shortest_path.h"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"

// static void printStack(std::stack<Coordinate> coords) {
//   while(!coords.empty()) {
//     std::cout << coords.top().toString() << " ";
//     coords.pop();
//   }
// }

static bool isCoordinateInVector(const Coordinate& target, const std::vector<Coordinate>& coordinates) {
  for (const auto& c: coordinates) {
    if (c == target) {
      return true;
    }
  }
  return false;
}
namespace {
    using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
    constexpr auto LEFT_INDEX = 0;
    constexpr auto FORWARD_INDEX = 1;
    constexpr auto RIGHT_INDEX = 2;

    constexpr auto CLOCKWISE_FORWARD_INDEX = 0;
    constexpr auto CLOCKWISE_RIGHT_INDEX = 1;
    constexpr auto CLOCKWISE_BEHIND_INDEX = 2;
    constexpr auto CLOCKWISE_LEFT_INDEX = 3;


  template <class T>
  void printVector(std::vector<T> toPrint, std::string name) {
    std::cout << name << " ";
    for (const auto& a : toPrint) {
      std::cout << a.toString() << ", ";
    }
    std::cout << std::endl;
  }
}

static RobotDirection toDirection(int8_t d) {
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

static int8_t directionToNumber(RobotDirection direction) {
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

static Coordinate calculateNewCoordianteBasedOnDirectionAndTurnIndex(RobotDirection currentDirection, Coordinate oldCoordinate, uint8_t turnIndex) {
  switch (currentDirection) {
  case RobotDirection::LEFT:
    if (turnIndex == LEFT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    if (turnIndex == FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    if (turnIndex == RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::UP:
    if (turnIndex == LEFT_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    if (turnIndex == FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    if (turnIndex == RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::RIGHT:
    if (turnIndex == LEFT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y - 1);
    }
    if (turnIndex == FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    if (turnIndex == RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  case RobotDirection::DOWN:
    if (turnIndex == LEFT_INDEX) {
      return Coordinate(oldCoordinate.x + 1, oldCoordinate.y);
    }
    if (turnIndex == FORWARD_INDEX) {
      return Coordinate(oldCoordinate.x, oldCoordinate.y + 1);
    }
    if (turnIndex == RIGHT_INDEX) {
      return Coordinate(oldCoordinate.x - 1, oldCoordinate.y);
    }
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  default:
    return Coordinate(oldCoordinate.x, oldCoordinate.y);
  }
}

static inline bool isCollisionTile(const RobotState& robotState, size_t tile) {
  return robotState.surroundingTiles[tile] == 'X' || robotState.surroundingTiles[tile] == '#';
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
    std::shared_ptr<rclcpp::Client<QueryInitialRobotPosition>> initialRobotPositionClient,
    std::function<std::vector<Coordinate>(MapStructure&)> submitMapStructureFn,
    std::function<void()> activateMiningValidateFn
) : moverCommunicator{moverCommunicator}, node{node}, 
    initialRobotPositionClient{initialRobotPositionClient}, submitMapStructureFn{submitMapStructureFn},
    activateMiningValidateFn{activateMiningValidateFn}
{

  mapGraph = MapGraph{};
  coordinatesTrail = std::stack<Coordinate>{};
}

void MinerNavigator::init() {
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();
  auto result = initialRobotPositionClient->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get initial position service");
    throw std::invalid_argument("Failed to get initial position of service");
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
}

std::vector<std::pair<uint8_t, Coordinate>> MinerNavigator::getValidMovementCoordinates() {
  std::vector<std::pair<uint8_t, Coordinate>> retval{};
  std::vector<uint8_t> possibleDirections{};
  for (int i = 0; i < 3; i++) {
    if (!isCollisionTile(robotState, i)) {
      possibleDirections.emplace_back(i);
    }
  }

  for (const int8_t idxOfNonCollisionTile: possibleDirections) {
      if (idxOfNonCollisionTile == LEFT_INDEX) {
          auto oldCoordinate = robotState.currentNode->getCoordinate();
          auto potentialDirection = getDirectionOnLeftTurn(robotState.direction);
          const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
          retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
      if (idxOfNonCollisionTile == FORWARD_INDEX) {
        auto oldCoordinate = robotState.currentNode->getCoordinate();
        auto potentialDirection = robotState.direction;
        const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
        retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
      if (idxOfNonCollisionTile == RIGHT_INDEX) {
        auto oldCoordinate = robotState.currentNode->getCoordinate();
        auto potentialDirection = getDirectionOnRightTurn(robotState.direction);
        const auto newCoordinate = calculateNewCoordianteBasedOnDirection(potentialDirection, oldCoordinate);
        retval.emplace_back(std::make_pair(idxOfNonCollisionTile, newCoordinate));
      }
  }
  return retval;
}

std::vector<std::function<void()>> MinerNavigator::getActionVector() {
  std::vector<std::function<void()>> retval{};
  auto validMovementCoordinates = getValidMovementCoordinates();

  for (const auto& [idxOfNonCollisionTile, coordinate]: validMovementCoordinates) {
    if (idxOfNonCollisionTile == LEFT_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordinate)) {
        const auto leftFunction = [this]() {
          this->goLeft();
        };
        retval.emplace_back(leftFunction);
      }
    }

    if (idxOfNonCollisionTile == FORWARD_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordinate)) {
        const auto forwardFunction = [this]() {
          this->goForward();
        };
        retval.emplace_back(forwardFunction);
      }
    }

    if (idxOfNonCollisionTile == RIGHT_INDEX) {
      if (!mapGraph.hasCoordinateBeenVisited(coordinate)) {
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
      retval.emplace_back(i);
    }
  }
  return retval;
}

void MinerNavigator::exploreMap() {
  while (true) {
    auto possibleActions = getActionVector();
    // We are in some corner and can't go forward, left or right.. Somehow we got to the corner, though, so the only way out is back
    if (possibleActions.empty()) {
      backtrackUntilUnstuck();
      continue;
    }

    // Execute first valid action
    possibleActions[0]();
  }

  while (!coordinatesTrail.empty()) {
    coordinatesTrail.pop();
  }
}

bool hasAtLeastOneUnvisitedCoordiante(const MapGraph& graph, const std::vector<std::pair<uint8_t, Coordinate>> coordinates) {
  for (const auto& possibility : coordinates) {
    const auto coordinate = possibility.second;
    if (!graph.hasCoordinateBeenVisited(coordinate)) {
      return true;
    }
  }
  return false;
}

std::pair<int32_t, Coordinate> getCoordinateOnbacktrackPath(
  const std::vector<std::pair<uint8_t, Coordinate>>& coordinates,
  const std::stack<Coordinate>& backtrackPath
) {
  if (backtrackPath.empty()) {
    return std::make_pair(-1, Coordinate(0, 0));
  }
  const auto topCoordinate = backtrackPath.top();
  for (int i = 0; i < 4; i++) {
    // Try to backtrack with what you see
    for (const auto indexCoordinatePair: coordinates) {
      const auto coordinate = indexCoordinatePair.second;
      if (topCoordinate.x == coordinate.x && topCoordinate.y == coordinate.y) {
        return indexCoordinatePair;
      }
    }
  }
  return std::make_pair(-1, Coordinate(0, 0));
}

void MinerNavigator::backtrackUntilUnstuck() {
  auto validMovementCoordinates = getValidMovementCoordinates();

  // First turn around
  if (validMovementCoordinates.empty()) {
    auto result = moverCommunicator.sendTurnRightCommand();
    robotState.direction = toDirection(result->robot_position_response.robot_dir);
    robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
    result = moverCommunicator.sendTurnRightCommand();
    robotState.direction = toDirection(result->robot_position_response.robot_dir);
    robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  }

  // Next, eliminate current node from stack we want to be comparing only to things that came before it
  coordinatesTrail.pop();

  while (true) {
    validMovementCoordinates = getValidMovementCoordinates();
    bool doneBacktracking = hasAtLeastOneUnvisitedCoordiante(mapGraph, validMovementCoordinates);
    if (doneBacktracking) {
      coordinatesTrail.push(robotState.currentNode->getCoordinate());
      break;
    }

    auto backtrackObject = getCoordinateOnbacktrackPath(validMovementCoordinates, coordinatesTrail);
    // if you can't find the right direction using your current orientation, you need to turn
    int spinCount = 0;
    while (backtrackObject.first == -1) {
      // Terminal condition -> we're on the starting tile and have done a 540 turn (everything has been explored)
      if (spinCount > 6) {
        auto solution = coordinate_remapper::getMapContents(mapGraph);
        mapGraph.shiftAllNodeCoordiantesToTheRightBy(solution.topLeftCoordinateBeforeRemapping);
        std::vector<Coordinate> longestTileLink = submitMapStructureFn(solution);
        const auto closestCoordOfLink = getClosestLongestTileLinkToMe(longestTileLink);
        const auto solutionAsMatrix = solution.asMatrix();
        const auto shortestPathToEnterClosestCoordOnLink = shortest_path::shortestPathFromTo(
          solutionAsMatrix, 
          robotState.currentNode->getCoordinate(),
          closestCoordOfLink);
        for (const auto& coord: shortestPathToEnterClosestCoordOnLink) {
          goToCoordinate(coord);
        }
        
        activateMiningValidateFn();
        traceLongestTileLink();
        throw std::invalid_argument("terminal condition reached");
      }
      auto result = moverCommunicator.sendTurnRightCommand();
      robotState.direction = toDirection(result->robot_position_response.robot_dir);
      robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
      validMovementCoordinates = getValidMovementCoordinates();
      backtrackObject = getCoordinateOnbacktrackPath(validMovementCoordinates, coordinatesTrail);
      spinCount += 1;
    }
    auto index = backtrackObject.first;

    if (index == LEFT_INDEX) {
      goLeft();
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      continue;
    }

    if (index == FORWARD_INDEX) {
      goForward();
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      continue;
    }

    if (index == RIGHT_INDEX) {
      goRight();
      // TODO nasty hack but needed because each goForward adds onto the stack
      coordinatesTrail.pop();
      coordinatesTrail.pop();
      continue;
    }
  }
}

static void printStack(std::stack<Coordinate> s) {
  while (!s.empty()) {
    std::cout << s.top().toString() << " ";
    s.pop();
  }
  std::cout << std::endl;
}

bool canMoveToAtLeastOneLocation(
  const RobotState& robotState,
  unsigned char targetCrystalType,
  const std::vector<Coordinate>& visitedDuringTrace) {
  const auto potentialCoordinateLeft = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
    robotState.direction,
    robotState.currentNode->getCoordinate(),
    LEFT_INDEX) ;
  const auto potentialCoordinateForward = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
    robotState.direction,
    robotState.currentNode->getCoordinate(),
    FORWARD_INDEX);
  const auto potentialCoordinateRight = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
    robotState.direction,
    robotState.currentNode->getCoordinate(),
    RIGHT_INDEX);

  auto canLeftBeVisited = robotState.surroundingTiles[LEFT_INDEX] == targetCrystalType && !isCoordinateInVector(potentialCoordinateLeft, visitedDuringTrace);
  auto canForwardBeVisited = robotState.surroundingTiles[FORWARD_INDEX] == targetCrystalType && !isCoordinateInVector(potentialCoordinateForward, visitedDuringTrace);
  auto canRightBeVisited = robotState.surroundingTiles[RIGHT_INDEX] == targetCrystalType && !isCoordinateInVector(potentialCoordinateRight, visitedDuringTrace);

  std::cout << "current location " << robotState.currentNode->getCoordinate().toString() << std::endl;
  std::cout << "state " << robotState.toString() << std::endl;
  std::cout << "left " << canLeftBeVisited << potentialCoordinateLeft.toString() << std::endl;
  std::cout << "forward " << canForwardBeVisited << potentialCoordinateForward.toString() << std::endl;
  std::cout << "right " << canRightBeVisited << potentialCoordinateRight.toString() << std::endl;
  std::cout << "---" << std::endl;
  return canLeftBeVisited || canForwardBeVisited || canRightBeVisited;
}

void MinerNavigator::traceLongestTileLink() {
  // clear so far so that the starting position becomes the current position
  while (!coordinatesTrail.empty()) {
    coordinatesTrail.pop();
  }
  coordinatesTrail.push(robotState.currentNode->getCoordinate());
  std::vector<Coordinate> visitedDuringTrace;

  const auto crystalType = robotState.currentNode->getBlockType();
  std::cout << static_cast<unsigned char>(crystalType) << std::endl;
  while (true) {
    bool allCoordinatesBanned = !canMoveToAtLeastOneLocation(robotState, crystalType, visitedDuringTrace);
    // Can't I backtrack here
    while (allCoordinatesBanned) {
      std::cout << "trail " << std::endl;
      printStack(coordinatesTrail);
      coordinatesTrail.pop(); // current node we're at, we need to pop it so that the previos node remains at the top of the stack
      std::cout << "backtracking from " << robotState.currentNode->getCoordinate().toString()
                << " to " << coordinatesTrail.top().toString();
      goToCoordinate(coordinatesTrail.top());
      coordinatesTrail.pop();
      allCoordinatesBanned = !canMoveToAtLeastOneLocation(robotState, crystalType, visitedDuringTrace);
    }
    if (robotState.surroundingTiles[LEFT_INDEX] == crystalType) {
      const auto potentialCoordinate = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
        robotState.direction,
        robotState.currentNode->getCoordinate(),
        LEFT_INDEX);
      if (!isCoordinateInVector(potentialCoordinate, visitedDuringTrace)) {
        goLeft();
        visitedDuringTrace.emplace_back(robotState.currentNode->getCoordinate());
        continue;
      }
    }

    if (robotState.surroundingTiles[FORWARD_INDEX] == crystalType) {
      const auto potentialCoordinate = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
        robotState.direction,
        robotState.currentNode->getCoordinate(),
        FORWARD_INDEX);
      if (!isCoordinateInVector(potentialCoordinate, visitedDuringTrace)) {
        goForward();
        visitedDuringTrace.emplace_back(robotState.currentNode->getCoordinate());
        continue;
      }
    }

    if (robotState.surroundingTiles[RIGHT_INDEX] == crystalType) {
      const auto potentialCoordinate = calculateNewCoordianteBasedOnDirectionAndTurnIndex(
        robotState.direction,
        robotState.currentNode->getCoordinate(),
        RIGHT_INDEX);
      if (!isCoordinateInVector(potentialCoordinate, visitedDuringTrace)) {
        goRight();
        continue;
      }
    }

    turnAround();
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

  const auto potentiallyExistingNode= mapGraph.getNodeAtCoordinate(newCoordinate);
  auto newNode = std::make_shared<GraphNode>(GraphNode(newCoordinate, tileType));
  if (potentiallyExistingNode) {
    newNode = *potentiallyExistingNode;
  } else {
    newNode->markVisited();
    mapGraph.addNode(newNode);
  }

  robotState.currentNode = newNode;

  coordinatesTrail.push(newCoordinate);
}

void MinerNavigator::goLeft() {
  auto result = moverCommunicator.sendTurnLeftCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  goForward();
}

void MinerNavigator::goRight() {
  auto result = moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;
  goForward();
}


// Points 3 and 4

ssize_t manhattanDistance(const Coordinate& c1, const Coordinate& c2) {
  return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y);
}

Coordinate MinerNavigator::getClosestLongestTileLinkToMe(std::vector<Coordinate> longestTileLinkCoordinates) {
  const auto myCoordinate = robotState.currentNode->getCoordinate();
  auto closestTileToMe = longestTileLinkCoordinates[0];
  auto smallestDistance = manhattanDistance(closestTileToMe, myCoordinate);
  for (const auto& coord : longestTileLinkCoordinates) {
    auto distance = manhattanDistance(coord, myCoordinate);
    if (distance < smallestDistance) {
      smallestDistance = distance;
      closestTileToMe = coord;
    }
  }
  return closestTileToMe;
}

void MinerNavigator::turnAround() {
  auto result = moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result->robot_position_response.robot_dir);
  robotState.surroundingTiles = result->robot_position_response.surrounding_tiles;

  auto result2 = moverCommunicator.sendTurnRightCommand();
  robotState.direction = toDirection(result2->robot_position_response.robot_dir);
  robotState.surroundingTiles = result2->robot_position_response.surrounding_tiles;
}

struct DirectionDistanceCoordinate {
  Coordinate coordinate;
  size_t clockwiseIndex;
  ssize_t distance;

  std::string toString()const {
    return string_format("%s -> (%d, %d)", coordinate.toString().c_str(), clockwiseIndex, distance);
  }
};

static std::vector<DirectionDistanceCoordinate> getSortedClosestCoordinateDirections(
  const RobotState& robotState,
  const std::vector<Coordinate>& aroundMe,
  const Coordinate& target
) {
  (void) robotState;
  std::vector<DirectionDistanceCoordinate> directionDistanceCoordinates{};
  for (size_t clockWiseDirectionIdex = 0; clockWiseDirectionIdex < aroundMe.size(); clockWiseDirectionIdex++) {
    auto coordinate = aroundMe[clockWiseDirectionIdex];
    auto distance = manhattanDistance(coordinate, target);
    directionDistanceCoordinates.emplace_back(DirectionDistanceCoordinate{coordinate, clockWiseDirectionIdex, distance});
  }
  std::sort(directionDistanceCoordinates.begin(), directionDistanceCoordinates.end(), [](const DirectionDistanceCoordinate& p1, DirectionDistanceCoordinate& p2) {
    return p1.distance < p2.distance;
  });

  return directionDistanceCoordinates;
}

DirectionDistanceCoordinate getLocationWithBestCost(const std::vector<DirectionDistanceCoordinate> coordinates, const Coordinate& me) {
  DirectionDistanceCoordinate smallest{Coordinate(0, 0), 100, std::numeric_limits<ssize_t>::max()};
  for (const auto& c : coordinates) {
    if (c.distance < smallest.distance && me.x != c.coordinate.x && me.y != c.coordinate.y) {
      smallest = c;
    }
  }
  return smallest;
}

void populatePossibleVisitLocations(
  std::vector<DirectionDistanceCoordinate>& possibleVisitLocations, 
  const std::vector<DirectionDistanceCoordinate>& currentNodeClosestLocations, 
  const RobotState& robotState
) {
  for (auto location : currentNodeClosestLocations) {
      if (location.clockwiseIndex == CLOCKWISE_FORWARD_INDEX) {
        if (!isCollisionTile(robotState, FORWARD_INDEX)) {
          possibleVisitLocations.emplace_back(location);
        }
        continue;
      }
      if (location.clockwiseIndex == CLOCKWISE_RIGHT_INDEX) {
        if (!isCollisionTile(robotState, RIGHT_INDEX)) {
          possibleVisitLocations.emplace_back(location);
        }
        continue;
      }
      if (location.clockwiseIndex == CLOCKWISE_LEFT_INDEX) {
        if (!isCollisionTile(robotState, LEFT_INDEX)) {
          possibleVisitLocations.emplace_back(location);
        }
        continue;
      }
  }
}

void MinerNavigator::goToCoordinate(const Coordinate& target) {
  // FRONT, RIGHT, BEHIND, LEFT
  std::vector<DirectionDistanceCoordinate> possibleVisitLocations{};
  while (!(robotState.currentNode->getCoordinate().x == target.x && robotState.currentNode->getCoordinate().y == target.y)) {

    auto coordinatesAroundMe = getCoordinatesAroundMe();
    auto currentNodeClosestCoordinateDirections = getSortedClosestCoordinateDirections(robotState, coordinatesAroundMe, target);
    populatePossibleVisitLocations(possibleVisitLocations, currentNodeClosestCoordinateDirections, robotState);
    auto closestOfPossibleVisitLocations = getLocationWithBestCost(possibleVisitLocations, robotState.currentNode->getCoordinate());
    if (closestOfPossibleVisitLocations.distance < currentNodeClosestCoordinateDirections[0].distance) {
      goToCoordinate(closestOfPossibleVisitLocations.coordinate);
    }

    for (const auto& closestCoordinateDirection : currentNodeClosestCoordinateDirections) {
      if (closestCoordinateDirection.clockwiseIndex == CLOCKWISE_FORWARD_INDEX) {
        if (!isCollisionTile(robotState, FORWARD_INDEX)) {
          goForward();
          break;
        }
      }
      if (closestCoordinateDirection.clockwiseIndex == CLOCKWISE_RIGHT_INDEX) {
        if (!isCollisionTile(robotState, RIGHT_INDEX)) {
          goRight();
          break;
        }
      }
      if (closestCoordinateDirection.clockwiseIndex == CLOCKWISE_LEFT_INDEX) {
        if (!isCollisionTile(robotState, LEFT_INDEX)) {
          goLeft();
          break;
        }
      }
      if (closestCoordinateDirection.clockwiseIndex == CLOCKWISE_BEHIND_INDEX) {
        turnAround();
        break;
      }

    }
  }
}

std::vector<Coordinate> MinerNavigator::getCoordinatesAroundMe() {
  // printVector(bannedCoordinates, "banned coordinates");
  // (void) bannedCoordinates;
  uint8_t numericDirectionCurrentlyPointingAt = directionToNumber(robotState.direction);
  std::vector<uint8_t> clockwiseNumericDirections{};
  for (int i = 0; i < 4; i++) {
    clockwiseNumericDirections.emplace_back((i + numericDirectionCurrentlyPointingAt) % 4);
  }
  std::vector<Coordinate> coordinatesAroundMe{};
  for (auto numericDirection : clockwiseNumericDirections) {
    const Coordinate c = calculateNewCoordianteBasedOnDirection(toDirection(numericDirection), robotState.currentNode->getCoordinate());
    coordinatesAroundMe.emplace_back(c);
  }
  return coordinatesAroundMe;
}