#ifndef _H_LONGEST_SEQUENCE
#define _H_LONGEST_SEQUENCE

#include <vector>

// TODO Coordinate and MapStructure have been copy-pasted from robo_miner_controller
// Ideally they would be extractd into robo_miner_common or something similar but 
// for the sake of time they were copy-pasted here
struct Coordinate {
    ssize_t x;
    ssize_t y;

    Coordinate(ssize_t x, ssize_t y) : x{x}, y{y} {}

    bool operator==(const Coordinate& o)const {
        return o.x == x && o.y == y;
    }
};

struct MapStructure {
    std::vector<unsigned char> data;
    ssize_t rows;
    ssize_t cols;

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

std::vector<Coordinate> getLongestSequence(const MapStructure& mapStructure);


#endif