#include <limits>
#include <iostream>
#include "robo_cleaner_controller/route_planner.h"
#include "robo_cleaner_controller/coordinate_remapper.h"
#include "robo_cleaner_controller/shortest_path.h"

static ssize_t calculateManhattanDistance(const Coordinate& c1, const Coordinate& c2) {
  return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y);
}

static void printVector(const std::vector<Coordinate>& path) {
  for (const auto& c : path) {
    std::cout << c.toString() << " ";
  }
  std::cout << std::endl;
}

namespace route_planner {

  ShortestPathWalker calculateRouteToClosestUnexploredCoordiante(const MapGraph& mapGraph, const Coordinate& currentCoordinate) {
    auto vectorisedGraph = coordinate_remapper::graphToMatrix(mapGraph);
    auto remappedCurrentCoordinateToMatrixSpace = Coordinate(currentCoordinate.x, currentCoordinate.y); // can be a copy constructor but this will do
    remappedCurrentCoordinateToMatrixSpace.x = remappedCurrentCoordinateToMatrixSpace.x - vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
    remappedCurrentCoordinateToMatrixSpace.y = remappedCurrentCoordinateToMatrixSpace.y - vectorisedGraph.topLeftCoordinateBeforeRemapping.y;

    // Find closest unexplored coordinate (this can be done as part of the shortest path algorithm but then I would've had to rework it) 
    ssize_t smallestManhattanDistance = std::numeric_limits<ssize_t>::max();
    Coordinate closestUnexploredCoordinate{0, 0};
    for (auto row = 0; row < vectorisedGraph.data.size(); row++) {
      for (auto col = 0; col< vectorisedGraph.data[0].size(); col++) {
        Coordinate potentialTargetCoordinate{col, row};
        if (potentialTargetCoordinate == currentCoordinate) { // Don't compare with myself, will always be shortest distance (0)
          continue;
        }
        auto manhattanDistance = calculateManhattanDistance(remappedCurrentCoordinateToMatrixSpace, potentialTargetCoordinate);
        if (manhattanDistance < smallestManhattanDistance && vectorisedGraph.data[row][col] == map_graph::UNEXPLORED_COORDINATE) {
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
    std::cout << "===== ROUTE PLANNER =====" << std::endl;
    std::cout << "remapped current coordinate " << remappedCurrentCoordinateToMatrixSpace.toString() << std::endl;
    std::cout << "location of closest unexplored coordinate " << closestUnexploredCoordinate.toString() << std::endl;
    std::cout << "route" << std::endl;
    printVector(shortestPath);
    std::cout << "=====/ROUTE PLANNER/=====" << std::endl;
    return ShortestPathWalker{std::move(shortestPath)};
  }

  ShortestPathWalker calculateRouteToCharingStation(const MapGraph& mapGraph, const Coordinate& currentCoordinate) {
   auto vectorisedGraph = coordinate_remapper::graphToMatrix(mapGraph);
    auto remappedCurrentCoordinateToMatrixSpace = Coordinate(currentCoordinate.x, currentCoordinate.y); // can be a copy constructor but this will do
    remappedCurrentCoordinateToMatrixSpace.x = remappedCurrentCoordinateToMatrixSpace.x - vectorisedGraph.topLeftCoordinateBeforeRemapping.x;
    remappedCurrentCoordinateToMatrixSpace.y = remappedCurrentCoordinateToMatrixSpace.y - vectorisedGraph.topLeftCoordinateBeforeRemapping.y;

    // Find closest unexplored coordinate (this can be done as part of the shortest path algorithm but then I would've had to rework it) 
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
    std::cout << "location of closest unexplored coordinate " << charingStationCoordinate.toString() << std::endl;
    std::cout << "route" << std::endl;
    printVector(shortestPath);
    std::cout << "=====/CHARGING PLANNER/=====" << std::endl;
    return ShortestPathWalker{std::move(shortestPath)};
  }
}
