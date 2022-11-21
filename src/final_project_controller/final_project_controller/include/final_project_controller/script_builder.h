#ifndef _H_SCRIPT_BUILDER
#define _H_SCRIPT_BUILDER

#include <string>
#include <sstream>

class ScriptBuilder {
public:
  ScriptBuilder();
  ScriptBuilder& beginWithDefaultHeader();
  void beginWithCustomHeader(const std::string& customHeader);
  ScriptBuilder& addCommand(const std::string& command);
  std::string str();
private:
  std::stringstream fullScript{};
};
#endif