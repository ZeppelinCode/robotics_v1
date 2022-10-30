#include <limits>
#include <iostream>
#include "robo_cleaner_controller/route_planner.h"
#include "robo_cleaner_controller/coordinate_remapper.h"
#include "robo_cleaner_controller/shortest_path.h"
#include "utils/Log.h"

static ssize_t calculateManhattanDistance(const Coordinate& c1, const Coordinate& c2) {
  return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y);
}

static void printVector(const std::vector<Coordinate>& path) {
  LOG("print path");
  for (const auto& c : path) {
    std::cout << c.toString() << " ";
  }
  std::cout << std::endl;
}

namespace route_planner {

  ShortestPathWalker calculateRouteToClosestUnexploredCoordiante(MapGraph& mapGraph, const Coordinate& currentCoordinate) {
    auto vectorisedGraph = coordinate_remapper::graphToMatrix(mapGraph);
    LOG("I am at %s", currentCoordinate.toString().c_str());
    LOG("attempting a coordinate remap on this structure\n%s", vectorisedGraph.toString().c_str());
    auto remappedCurrentCoordinateToMatrixSpace = Coordinate(currentCoordinate.x, currentCoordinate.y); // can be a copy constructor but this will do
    remappedCurrentCoordinateToMatrixSpace.x = remappedCurrentCoordinateToMatrixSpace.x - vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
    remappedCurrentCoordinateToMatrixSpace.y = remappedCurrentCoordinateToMatrixSpace.y - vectorisedGraph.topLeftCoordinateBeforeRemapping.y;

    ssize_t smallestManhattanDistance = std::numeric_limits<ssize_t>::max();
    Coordinate closestUnexploredCoordinate{0, 0};
    for (auto row = 0; row < vectorisedGraph.data.size(); row++) {
      for (auto col = 0; col< vectorisedGraph.data[0].size(); col++) {
        Coordinate potentialTargetCoordinate{col, row};
        if (potentialTargetCoordinate == remappedCurrentCoordinateToMatrixSpace) { // Don't compare with myself, will always be shortest distance (0)
          continue;
        }
        auto manhattanDistance = calculateManhattanDistance(remappedCurrentCoordinateToMatrixSpace, potentialTargetCoordinate);
        if (manhattanDistance < smallestManhattanDistance 
          && (vectorisedGraph.data[row][col] == map_graph::UNEXPLORED_COORDINATE
              || vectorisedGraph.data[row][col] == '1'
              || vectorisedGraph.data[row][col] == '2')) {
          smallestManhattanDistance = manhattanDistance;
          closestUnexploredCoordinate = potentialTargetCoordinate;
        }
      }
    }

    auto shortestPath = shortest_path::shortestPathFromTo(vectorisedGraph.data, remappedCurrentCoordinateToMatrixSpace, closestUnexploredCoordinate);

    // Remap back to map graph space
    for (auto& coord : shortestPath) {
      coord.x = coord.x + vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
      coord.y = coord.y + vectorisedGraph.topLeftCoordinateBeforeRemapping.y;
    }
    LOG("===== ROUTE PLANNER =====");
    LOG("remapped current coordinate %s", remappedCurrentCoordinateToMatrixSpace.toString().c_str());
    // std::cout << "location of closest unexplored coordinate " << closestUnexploredCoordinate.toString() << std::endl;
    printVector(shortestPath);
    LOG("=====/ROUTE PLANNER/=====");
    return ShortestPathWalker{std::move(shortestPath)};
  }

  ShortestPathWalker calculateRouteToCharingStation(MapGraph& mapGraph, const Coordinate& currentCoordinate) {
   auto vectorisedGraph = coordinate_remapper::graphToMatrix(mapGraph);
    auto remappedCurrentCoordinateToMatrixSpace = Coordinate(currentCoordinate.x, currentCoordinate.y); // can be a copy constructor but this will do
    remappedCurrentCoordinateToMatrixSpace.x = remappedCurrentCoordinateToMatrixSpace.x - vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
    remappedCurrentCoordinateToMatrixSpace.y = remappedCurrentCoordinateToMatrixSpace.y - vectorisedGraph.topLeftCoordinateBeforeRemapping.y;

    // Find charing station coordinate
    Coordinate charingStationCoordinate{0, 0};
    for (auto row = 0; row < vectorisedGraph.data.size(); row++) {
      for (auto col = 0; col< vectorisedGraph.data[0].size(); col++) {
        if (vectorisedGraph.data[row][col] == map_graph::CHARGING_STATION_COORDINATE) {
          charingStationCoordinate = Coordinate{col, row};
          goto found_charging_coordinate;
        }
      }
    }
    found_charging_coordinate:

    auto shortestPath = shortest_path::shortestPathFromTo(vectorisedGraph.data, remappedCurrentCoordinateToMatrixSpace, charingStationCoordinate);
    // Remap back to map graph space
    for (auto& coord : shortestPath) {
      coord.x = coord.x + vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
      coord.y = coord.y + vectorisedGraph.topLeftCoordinateBeforeRemapping.y;
    }
    std::cout << "===== CHARGING PLANNER =====" << std::endl;
    std::cout << "remapped current coordinate " << remappedCurrentCoordinateToMatrixSpace.toString() << std::endl;
    std::cout << "location of charging station" << charingStationCoordinate.toString() << std::endl;
    std::cout << "route" << std::endl;
    printVector(shortestPath);
    std::cout << "=====/CHARGING PLANNER/=====" << std::endl;
    return ShortestPathWalker{std::move(shortestPath)};
  }
}
