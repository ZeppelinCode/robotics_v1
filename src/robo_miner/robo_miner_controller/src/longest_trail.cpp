#include "robo_miner_controller/longest_trail.h"
#include <iostream>

bool doesVecContainCoordinate(const std::vector<Coordinate>& data, Coordinate& target) {
    for (auto c : data) {
        if (c.x == target.x && c.y == target.y) {
            return true;
        }
    }
    return false;
}
void longestPathAtCoordinate(
    const std::vector<std::vector<unsigned char>>& data,
    Coordinate coord,
    std::vector<Coordinate>& acc,
    std::vector<Coordinate>& totalVisited
) {
    if (doesVecContainCoordinate(acc, coord)) {
        return;
    }
    if (doesVecContainCoordinate(totalVisited, coord)) {
        return;
    }

    acc.emplace_back(coord);
    totalVisited.emplace_back(coord);
    auto myValue = data[coord.y][coord.x];
    if (coord.x > 0) {
        auto leftValue = data[coord.y][coord.x - 1];
        if (leftValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x - 1, coord.y), acc, totalVisited);
        }
    }
    if (coord.y > 0) {
        auto topValue = data[coord.y - 1][coord.x];
        if (topValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x, coord.y-1), acc, totalVisited);
        }
    }
    if (coord.y < static_cast<ssize_t>(data.size()) - 1) {
        auto bottomValue = data[coord.y + 1][coord.x];
        if (bottomValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x, coord.y+1), acc, totalVisited);
        }
    }
    if (coord.x < static_cast<ssize_t>(data[0].size()) - 1) {
        auto rightValue = data[coord.y][coord.x + 1];
        if (rightValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x + 1, coord.y), acc, totalVisited);
        }
    }
}

// TODO this could be done as a 2d wrapper class with an overloaded [] operator but for the 
// sake of time, an inefficient approach of copying data is used
namespace longest_trail {
    std::vector<Coordinate> getLongestTrail(const MapStructure& mapStructure) {
        std::vector<std::vector<unsigned char>> grid2d{};
        grid2d.resize(mapStructure.rows, std::vector<unsigned char>{});

        for (auto& v : grid2d) {
            v.resize(mapStructure.cols, '0');
        }

        // Convert the map structure to a 2d variant
        for (int i = 0; i < mapStructure.rows; i++) {
            for (int j = 0; j < mapStructure.cols; j++) {
                grid2d[i][j] = mapStructure.data[i*mapStructure.cols + j];
            }
        }

        std::cout << "remapped" << std::endl;
        for (const auto& row : grid2d) {
            for (const auto& v: row) {
                std::cout << v;
            }
            std::cout << std::endl;
        }

        std::vector<Coordinate> longestConnected{};
        std::vector<Coordinate> totalVisited{};
        std::vector<Coordinate> currentAcc{};
        for (size_t i = 0; i < grid2d.size(); i++) {
            for (size_t j = 0; j < grid2d[0].size(); j++) {
                std::cout << i << " " << j << std::endl;
                currentAcc = std::vector<Coordinate>{};
                longestPathAtCoordinate(grid2d, Coordinate(j, i), currentAcc, totalVisited);

                if (currentAcc.size() > longestConnected.size()) {
                    longestConnected = currentAcc;
                }
            }
        }

        return longestConnected;
    }
}