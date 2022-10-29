#ifndef _H_SHORTEST_PATH_WALKER
#define _H_SHORTEST_PATH_WALKER
#include "robo_cleaner_controller/map_graph.h"

class ShortestPathWalker {
public:
  ShortestPathWalker();
  ShortestPathWalker(std::vector<Coordinate>&& coordinatesToTrace);
  bool hasUntracedCoordinates();
  Coordinate getNextCoordinate();
private:
  std::vector<Coordinate> coordinatesToTrace;
  size_t nextCoordinateIndex;
};
#endif