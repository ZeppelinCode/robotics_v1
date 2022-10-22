#ifndef _H_COORDINATE_REMAPPER
#define _H_COORDINATE_REMAPPER

#include <utility>
#include <vector>
#include "robo_miner_controller/map_graph.h"
#include <sstream>

struct MapStructure {
    std::vector<unsigned char> data;
    ssize_t rows;
    ssize_t cols;
    Coordinate topLeftCoordinateBeforeRemapping;

    std::string toString() {
        std::stringstream representation;
        representation << "rows: " << rows 
                << ", cols: " << cols << std::endl;
        for (auto c : data) {
            representation << c << "";
        }
        representation << std::endl;
        return representation.str();
    }
};

namespace coordinate_remapper{
    Coordinate findTopLeftCorner(const MapGraph& graph);
    MapStructure getMapContents(const MapGraph& graph);
}

#endif