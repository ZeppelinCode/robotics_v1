#include "robo_cleaner_controller/shortest_path_walker.h"

ShortestPathWalker::ShortestPathWalker() : coordinatesToTrace{std::vector<Coordinate>{}}, nextCoordinateIndex{0} {}

ShortestPathWalker::ShortestPathWalker(std::vector<Coordinate>&& coordinatesToTrace) : coordinatesToTrace{coordinatesToTrace}, nextCoordinateIndex{0} {}

ShortestPathWalker::ShortestPathWalker(std::vector<Coordinate>&& coordinatesToTrace, bool isValid) 
: coordinatesToTrace{coordinatesToTrace}, nextCoordinateIndex{0}, mIsValid{isValid} {}

bool ShortestPathWalker::hasUntracedCoordinates() {
    return nextCoordinateIndex < coordinatesToTrace.size();
}

bool ShortestPathWalker::isValid() {
    return mIsValid;
}

Coordinate ShortestPathWalker::getNextCoordinate() {
    return coordinatesToTrace[nextCoordinateIndex++];
}