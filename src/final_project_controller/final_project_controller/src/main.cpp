#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "urscript_common/defines/UrScriptTopics.h"
#include "final_project_controller/misc.h"
#include "final_project_controller/config_loader.h"
#include "final_project_controller/command_executor.h"

// Path adjustments: Go home script inside main
// Config loader constructor <- configs
// Command executor constructor <- open/close gripper scripts
int main(int32_t argc, char *argv[]) {
    rclcpp::InitOptions initOptions;
    initOptions.shutdown_on_sigint = true;
    rclcpp::init(argc, argv, initOptions);

    const auto configLoader = std::make_shared<ConfigLoader>();

    using UrScriptSrv = urscript_interfaces::srv::UrScript;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("final_project_controller_node");
    auto urScriptClient = node->create_client<UrScriptSrv>("urscript_service", rmw_qos_profile_services_default);

    const auto commandExecutor = CommandExecutor{node, urScriptClient, configLoader};
    const auto goToHome = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/go_to_home.urscript");

    std::cout << "going home" << std::endl;
    commandExecutor.executeServiceRequest(goToHome);
    commandExecutor.closeGripper();
    commandExecutor.openGripper();
    std::cout << "running pick and place sequence" << std::endl;
    commandExecutor.runPickAndPlaceSequence();

    rclcpp::shutdown();
    return 0;
}

