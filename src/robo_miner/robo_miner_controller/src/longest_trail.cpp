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
void longestPathAtCoordinate(const std::vector<std::vector<unsigned char>>& data, Coordinate coord, std::vector<Coordinate>& acc) {
    if (doesVecContainCoordinate(acc, coord)) {
        return;
    }
    //     acc.add((i, j))
    acc.emplace_back(coord);
//     if j > 0:
//         left_value = d[i][j-1]
//         if left_value== my_value:
//             lp2(d, i, j-1, acc)
    auto myValue = data[coord.y][coord.x];
    if (coord.x > 0) {
        auto leftValue = data[coord.y][coord.x - 1];
        if (leftValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x - 1, coord.y), acc);
        }
    }
//     if i > 0:
//         top_value = d[i - 1][j]
//         if top_value == my_value:
//             lp2(d, i - 1, j, acc)
    if (coord.y > 0) {
        auto topValue = data[coord.y - 1][coord.x];
        if (topValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x, coord.y-1), acc);
        }
    }
//     if i < len(d)-1:
//         bottom_value = d[i + 1][j]
//         if bottom_value== my_value:
//             lp2(d, i + 1, j, acc)
    if (coord.y < static_cast<ssize_t>(data.size()) - 1) {
        auto bottomValue = data[coord.x][coord.y + 1];
        if (bottomValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.x, coord.y+1), acc);
        }
    }
//     if j < len(d[0]) - 1:
//         right_value = d[i][j+1]
//         if right_value == my_value:
//             lp2(d, i, j+1, acc)
    if (coord.x < static_cast<ssize_t>(data[0].size()) - 1) {
        auto rightValue = data[coord.y][coord.x + 1];
        if (rightValue == myValue) {
            longestPathAtCoordinate(data, Coordinate(coord.y, coord.x + 1), acc);
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

        std::cout << "finding longest path" << std::endl;
        std::vector<Coordinate> longestConnected{};
        longestPathAtCoordinate(grid2d, Coordinate(0, 0), longestConnected);
        return longestConnected;
    }
}