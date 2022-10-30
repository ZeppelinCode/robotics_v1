#include "robo_cleaner_controller/shortest_path.h"

static bool isCoordinateInVector(const Coordinate& target, std::vector<Coordinate>& coordinates) {
  for (const auto& c: coordinates) {
    if (c == target) {
      return true;
    }
  }
  return false;
}

static int getNumberOfUnexploredTilesOnPath(const std::vector<std::vector<unsigned char>>& map, const std::vector<Coordinate>& path) {
    int count = 0;
    for (const auto& coord : path) {
        if (map[coord.y][coord.x] == '#' || (map[coord.y][coord.x] >= '1' && map[coord.y][coord.x] <= '3')) {
            count += 1;
        }
    }
    return count;
}

static void dfs(
    const std::vector<std::vector<unsigned char>>& map,
    Coordinate from,
    Coordinate to,
    std::vector<Coordinate>& currentPath,
    std::vector<Coordinate>& bestPath
) {
    if (!bestPath.empty()) {
        if (currentPath.size() > bestPath.size()) {
            return;
        }
    }

    if (from.x < 0 || from.x >= static_cast<ssize_t>(map[0].size())) {
        return;
    }
    if (from.y < 0 || from.y >= static_cast<ssize_t>(map.size())) {
        return;
    }
    if (isCoordinateInVector(from, currentPath)) {
        return;
    }

    if (map[from.y][from.x] == map_graph::COLLISION_COORDINATE) {
        return;
    }

    currentPath.push_back(from);
    if (from == to) {
        if (currentPath.size() < bestPath.size() || bestPath.empty()) {
            bestPath = std::vector<Coordinate>(currentPath);
        }
        
        if (currentPath.size() == bestPath.size()) {
            if (getNumberOfUnexploredTilesOnPath(map, currentPath) > getNumberOfUnexploredTilesOnPath(map, bestPath)) {
                bestPath = std::vector<Coordinate>(currentPath);
            }
        }
    }

    dfs(map, Coordinate(from.x-1, from.y), to, currentPath, bestPath);
    dfs(map, Coordinate(from.x+1, from.y), to, currentPath, bestPath);
    dfs(map, Coordinate(from.x, from.y-1), to, currentPath, bestPath);
    dfs(map, Coordinate(from.x, from.y+1), to, currentPath, bestPath);
    if (!currentPath.empty()) {
        currentPath.pop_back();
    }

}

std::vector<Coordinate> shortest_path::shortestPathFromTo(const std::vector<std::vector<unsigned char>>& map, Coordinate from, Coordinate to) {
    std::vector<Coordinate> currentPath{};
    std::vector<Coordinate> bestPath{};
    dfs(map, from, to, currentPath, bestPath);
    return bestPath;
}
