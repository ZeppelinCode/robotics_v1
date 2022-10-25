#include "robo_miner_controller/shortest_path.h"

static bool isCoordinateInVector(const Coordinate& target, std::vector<Coordinate>& coordinates) {
  for (const auto& c: coordinates) {
    if (c == target) {
      return true;
    }
  }
  return false;
}

static void dfs(
    const std::vector<std::vector<unsigned char>>& map,
    Coordinate from,
    Coordinate to,
    std::vector<Coordinate>& currentPath,
    std::vector<Coordinate>& bestPath,
    bool* anyPathFound
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

    if (map[from.y][from.x] == 'X') {
        return;
    }

    currentPath.push_back(from);
    if (from == to) {
        if (currentPath.size() < bestPath.size() || bestPath.empty()) {
            *anyPathFound = true;
            bestPath = std::vector<Coordinate>(currentPath);
        }
    }

    dfs(map, Coordinate(from.x-1, from.y), to, currentPath, bestPath, anyPathFound);
    dfs(map, Coordinate(from.x+1, from.y), to, currentPath, bestPath, anyPathFound);
    dfs(map, Coordinate(from.x, from.y-1), to, currentPath, bestPath, anyPathFound);
    dfs(map, Coordinate(from.x, from.y+1), to, currentPath, bestPath, anyPathFound);
    if (!currentPath.empty()) {
        currentPath.pop_back();
    }

}

std::vector<Coordinate> shortest_path::shortestPathFromTo(const std::vector<std::vector<unsigned char>>& map, Coordinate from, Coordinate to) {
    std::vector<Coordinate> currentPath{};
    std::vector<Coordinate> bestPath{};
    bool anyPathFound = false;
    dfs(map, from, to, currentPath, bestPath, &anyPathFound);
    return bestPath;
}
