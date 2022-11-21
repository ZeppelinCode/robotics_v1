#ifndef _H_SCRIPT_BUILDER
#define _H_SCRIPT_BUILDER

#include <string>
#include <sstream>

class ScriptBuilder {
public:
  ScriptBuilder();
  void beginWithDefaultHeader();
  void beginWithCustomHeader(const std::string& customHeader);
  void addCommand(const std::string& command); // TODO can be a fluent builder but don't want to fight C++ compiler
  std::string str();
private:
  std::stringstream fullScript{};
};
#endif