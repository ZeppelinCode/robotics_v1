#ifndef _H_CLOSEST_UNEXPLORED_COORDINATE_DISCOVERER
#define _H_CLOSEST_UNEXPLORED_COORDINATE_DISCOVERER
#include <vector>
#include "robo_cleaner_controller/map_graph.h"
#include "robo_cleaner_controller/shortest_path_walker.h"

namespace route_planner {
  ShortestPathWalker calculateRouteToClosestUnexploredCoordiante(MapGraph& mapGraph, const Coordinate& currentCoordinate);
  ShortestPathWalker calculateRouteToCharingStation(MapGraph& mapGraph, const Coordinate& currentCoordinate);
}

#endif