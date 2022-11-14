#ifndef _H_MISC
#define _H_MISC

#include <string>
#include <vector>

namespace misc {
  std::string readFileToString(std::string path);
  std::vector<std::string> splitStringBy(const std::string& str, const char c);
}

#endif