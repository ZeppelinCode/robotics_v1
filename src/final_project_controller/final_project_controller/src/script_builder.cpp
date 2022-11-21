#include "final_project_controller/script_builder.h"

constexpr auto INDENTATION = "  ";

ScriptBuilder::ScriptBuilder() {
}

ScriptBuilder& ScriptBuilder::beginWithDefaultHeader() {
    fullScript << "def script():\n";
    return *this;
}

void ScriptBuilder::beginWithCustomHeader(const std::string& header) {
    fullScript << header << '\n';
}

ScriptBuilder& ScriptBuilder::addCommand(const std::string& command) {
    fullScript << INDENTATION << command << '\n';
    return *this;
}

std::string ScriptBuilder::str() {
    fullScript << "end\n\n\n";
    return fullScript.str();
}