#ifndef _H_SHORTEST_PATH
#define _H_SHORTEST_PATH

#include <vector>
#include "robo_cleaner_controller/map_graph.h"

namespace shortest_path {
  std::vector<Coordinate> shortestPathFromTo(const std::vector<std::vector<unsigned char>>& map, Coordinate source, Coordinate sink);
}

#endif