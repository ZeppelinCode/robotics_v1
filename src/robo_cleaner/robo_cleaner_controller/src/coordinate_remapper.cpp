#include "robo_cleaner_controller/coordinate_remapper.h"
#include <memory>
#include <limits>
#include <cmath>
#include <iostream>

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

        return VectorisedMapStructure{
            std::move(matrixGraphRepresentation),
            topLeftCorner
        };

    }
}