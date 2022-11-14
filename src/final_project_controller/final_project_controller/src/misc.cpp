#include "final_project_controller/misc.h"

#include <fstream>
#include <sstream>

namespace misc {

std::vector<std::string> splitStringBy(const std::string& str, const char c) {
    std::vector<std::string> tokens;
 
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, c)) {
        tokens.push_back(token);
    }
 
    return tokens;
}

std::string readFileToString(std::string path) {
    std::ifstream textFile(path);
    std::stringstream buffer;
    buffer << textFile.rdbuf();
    return buffer.str();
}

}