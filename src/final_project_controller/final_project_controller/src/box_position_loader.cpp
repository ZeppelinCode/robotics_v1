#include <sstream>
#include "final_project_controller/box_position_loader.h"
#include "final_project_controller/misc.h"

constexpr auto BOX_COORDINATES_PATH = "/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/box_coordinates.txt";

std::vector<std::string> splitStringBy(const std::string& str, const char c)
{
    std::vector<std::string> tokens;
 
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, c)) {
        tokens.push_back(token);
    }
 
    return tokens;
}

namespace bpl {

std::vector<BoxPosition> loadBoxPositions() {
    using namespace std;
    vector<BoxPosition> retval;
    string sBoxCoordinates = misc::readFileToString(BOX_COORDINATES_PATH);
    auto sBoxLines = splitStringBy(sBoxCoordinates, '\n');
    for (const auto& line: sBoxLines) {
        auto sElementsInLine = splitStringBy(line, ' ');
        retval.emplace_back(BoxPosition{
            stod(sElementsInLine[0]),
            stod(sElementsInLine[1]),
            stod(sElementsInLine[2]),
            stod(sElementsInLine[3]),
            stod(sElementsInLine[4]),
            stod(sElementsInLine[5]),
        });
    }

    return retval;
}

}