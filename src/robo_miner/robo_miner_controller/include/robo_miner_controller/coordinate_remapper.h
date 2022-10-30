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

    std::vector<std::vector<unsigned char>> asMatrix()const {
        std::vector<std::vector<unsigned char>> retval{};
        retval.resize(rows, std::vector<unsigned char>{});

        for (auto& v : retval) {
            v.resize(cols, '0');
        }

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                retval[i][j] = data[i*cols + j];
            }
        }

        return retval;
    }
};

namespace coordinate_remapper{
    Coordinate findTopLeftCorner(const MapGraph& graph);
    MapStructure getMapContents(const MapGraph& graph);
}

#endif