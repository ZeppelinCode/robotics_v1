#include "final_project_controller/misc.h"

#include <fstream>
#include <sstream>

namespace misc {
    std::string readFileToString(std::string path) {
    std::ifstream textFile(path);
    std::stringstream buffer;
    buffer << textFile.rdbuf();
    return buffer.str();
    }
}