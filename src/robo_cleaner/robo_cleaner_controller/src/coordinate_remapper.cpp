#include "robo_cleaner_controller/coordinate_remapper.h"
#include <memory>
#include <limits>
#include <cmath>
#include <iostream>
#include "utils/Log.h"

static bool isPseudoWalkableCoordionate(unsigned char coordinate) {
    return coordinate != 'X' && coordinate != '#';
}

static bool isReachable(const std::vector<std::vector<unsigned char>>& matrixGraphRepresentation, size_t rowi, size_t coli) {
    auto above = 'X';
    if (rowi > 0) {
        above = matrixGraphRepresentation[rowi-1][coli];
    }

    auto right = 'X';
    if (coli  < matrixGraphRepresentation[0].size() - 1) {
        right = matrixGraphRepresentation[rowi][coli + 1];
    }

    auto left = 'X';
    if (coli > 0) {
        left = matrixGraphRepresentation[rowi][coli - 1];
    }

    auto below = 'X';
    if (rowi < matrixGraphRepresentation.size() - 1) {
        below = matrixGraphRepresentation[rowi+1][coli];
    }
    return isPseudoWalkableCoordionate(above) || isPseudoWalkableCoordionate(right)
        || isPseudoWalkableCoordionate(left) || isPseudoWalkableCoordionate(below);
}
namespace coordinate_remapper {
    static std::pair<Coordinate, Coordinate> findTopLeftCornerAndBottomRightCorner(const MapGraph& graph) {
        ssize_t smallestX = std::numeric_limits<ssize_t>::max();
        ssize_t smallestY = std::numeric_limits<ssize_t>::max();
        ssize_t largestX = std::numeric_limits<ssize_t>::min();
        ssize_t largestY = std::numeric_limits<ssize_t>::min();
        for (const auto& node : graph.getNodes()) {
            auto coordinate = node->getCoordinate();
            if (coordinate.x < smallestX) {
                smallestX = coordinate.x;
            }
            if (coordinate.x > largestX) {
                largestX = coordinate.x;
            }
            if (coordinate.y < smallestY) {
                smallestY = coordinate.y;
            }
            if (coordinate.y > largestY) {
                largestY = coordinate.y;
            }
        }
        return std::make_pair(Coordinate(smallestX, smallestY), Coordinate(largestX, largestY));
    }


    VectorisedMapStructure graphToMatrix(const MapGraph& graph) {
        std::vector<std::vector<unsigned char>> matrixGraphRepresentation{};

        auto [topLeftCorner, bottomRightCorner] = findTopLeftCornerAndBottomRightCorner(graph);
        auto matrixWidth = std::abs(topLeftCorner.x - bottomRightCorner.x) + 1;
        auto matrixHeight = std::abs(topLeftCorner.y - bottomRightCorner.y) + 1;
        matrixGraphRepresentation.resize(matrixHeight, std::vector<unsigned char>{});
        for (auto& row: matrixGraphRepresentation) {
            row.resize(matrixWidth, map_graph::UNEXPLORED_COORDINATE);
        }

        for (const auto& node: graph.getNodes()) {
            auto nodeCoordinate = node->getCoordinate();
            auto nodeTile = node->getBlockType();
            nodeCoordinate.x = nodeCoordinate.x - topLeftCorner.x;
            nodeCoordinate.y = nodeCoordinate.y - topLeftCorner.y;
            matrixGraphRepresentation[nodeCoordinate.y][nodeCoordinate.x] = nodeTile;
        }

        // Prune unreachable destinations
        for (size_t rowi = 0; rowi < matrixGraphRepresentation.size(); rowi++) {
            for (size_t coli = 0; coli < matrixGraphRepresentation[rowi].size(); coli++) {
                if (!isReachable(matrixGraphRepresentation, rowi, coli)) {
                    matrixGraphRepresentation[rowi][coli] = 'X';
                }
            }
        }

        return VectorisedMapStructure{
            std::move(matrixGraphRepresentation),
            topLeftCorner
        };

    }
}