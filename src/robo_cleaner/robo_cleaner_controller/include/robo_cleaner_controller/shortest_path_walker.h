#ifndef _H_SHORTEST_PATH_WALKER
#define _H_SHORTEST_PATH_WALKER
#include "robo_cleaner_controller/map_graph.h"

class ShortestPathWalker {
public:
  ShortestPathWalker();
  ShortestPathWalker(std::vector<Coordinate>&& coordinatesToTrace);
  ShortestPathWalker(std::vector<Coordinate>&& coordinatesToTrace, bool isValid);
  bool hasUntracedCoordinates();
  Coordinate getNextCoordinate();
  bool isValid();
  void setInvalid();
private:
  std::vector<Coordinate> coordinatesToTrace;
  size_t nextCoordinateIndex;
  bool mIsValid{true};
};
#endif