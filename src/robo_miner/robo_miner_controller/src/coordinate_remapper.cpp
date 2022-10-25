#include "robo_miner_controller/coordinate_remapper.h"
#include <memory>
#include <limits>

namespace coordinate_remapper {
    Coordinate findTopLeftCorner(const MapGraph& graph) {
        std::shared_ptr<GraphNode> topLeftNode = nullptr;
        ssize_t smallestX = std::numeric_limits<ssize_t>::max();
        ssize_t smallestY = std::numeric_limits<ssize_t>::max();
        for (const auto& node : graph.getNodes()) {
            auto coordinate = node->getCoordinate();
            if (coordinate.x < smallestX) {
                smallestX = coordinate.x;
                topLeftNode = node;
            }
            if (coordinate.y < smallestY) {
                smallestY = coordinate.y;
                topLeftNode = node;
            }
        }
        return topLeftNode->getCoordinate();
    }

    MapStructure getMapContents(const MapGraph& graph) {
        std::vector<std::pair<Coordinate, char>> allEntriesInGraph{};

        Coordinate topLeftCoordinate = Coordinate(std::numeric_limits<ssize_t>::max(), std::numeric_limits<ssize_t>::max());
        ssize_t smallestX = std::numeric_limits<ssize_t>::max();
        ssize_t smallestY = std::numeric_limits<ssize_t>::max();

        // Extract top left coordinate and flatten all nodes into a local data structure
        for (const auto& node : graph.getNodes()) {
            auto coordinate = node->getCoordinate();
            auto blockType = node->getBlockType();
            allEntriesInGraph.emplace_back(std::make_pair(coordinate, blockType));

            if (coordinate.x < smallestX) {
                smallestX = coordinate.x;
                topLeftCoordinate = coordinate;
            }

            if (coordinate.y < smallestY) {
                smallestY = coordinate.y;
                topLeftCoordinate = coordinate;
            }
        }

        // make top left corner be (0, 0) and adjust all other nodes in grid accordingly
        for (auto &entry : allEntriesInGraph) {
            entry.first.x = entry.first.x - smallestX;
            entry.first.y = entry.first.y - smallestY;
        }

        auto largestX = std::numeric_limits<ssize_t>::min();
        auto largestY = std::numeric_limits<ssize_t>::min();
        for (const auto& entry: allEntriesInGraph) {
            if (entry.first.x > largestX) {
                largestX = entry.first.x;
            }

            if (entry.first.y > largestY) {
                largestY = entry.first.y;
            }
        }

        
        const auto rows = largestY + 1;
        const auto cols = largestX + 1;
        std::vector<unsigned char> retVector{};
        retVector.resize(rows * cols, 'X');

        for (const auto& entry: allEntriesInGraph) {
            const auto index = entry.first.y * cols + entry.first.x;
            retVector.at(index) = entry.second;
        }

        return MapStructure{
            std::move(retVector),
            largestY + 1,
            largestX + 1,
            Coordinate(smallestX, smallestY)
        };
    }
}