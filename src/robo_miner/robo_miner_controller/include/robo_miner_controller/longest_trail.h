#ifndef _H_LONGEST_TRAIL
#define _H_LONGEST_TRAIL

#include <vector>
#include "robo_miner_controller/map_graph.h"
#include "robo_miner_controller/coordinate_remapper.h"

namespace longest_trail {

  std::vector<Coordinate> getLongestTrail(const MapStructure& mapStructure);

}

#endif