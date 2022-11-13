#include <sstream>
#include "final_project_controller/box_position.h"


BoxPosition::BoxPosition(double x, double y, double z, double rx, double ry, double rz): x{x}, y{y}, z{z}, rx{rx}, ry{ry}, rz{rz} {}

std::string BoxPosition::str()const {
    std::stringstream representation;
    representation << "BoxPosition(" 
                   << x << ", " 
                   << y << ", " 
                   << z << ", " 
                   << rx << ", "
                   << ry << ", "
                   << rz << ")";
    return representation.str();
}