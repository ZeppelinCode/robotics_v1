#ifndef _H_COORDINATE_REMAPPER
#define _H_COORDINATE_REMAPPER

#include <utility>
#include <vector>
#include "robo_cleaner_controller/map_graph.h"
#include <sstream>

struct VectorisedMapStructure {
    std::vector<std::vector<unsigned char>> data;
    Coordinate topLeftCoordinateBeforeRemapping;

    std::string toString() {
        std::stringstream representation;
        representation << "vectorised map structure" << std::endl << "=====" << std::endl;
        representation << "top left corner " << topLeftCoordinateBeforeRemapping.toString() << std::endl << "map" << std::endl << "---" << std::endl;
        for (const auto& row : data) {
            for (auto col : row) {
                representation << col << "";
            }
            representation << std::endl;
        }
        representation << std::endl;
        return representation.str();
    }
};

namespace coordinate_remapper{
    VectorisedMapStructure graphToMatrix(const MapGraph& graph);
}

#endif