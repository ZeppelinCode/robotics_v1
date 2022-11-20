#ifndef _H_COMMAND_EXECUTOR
#define _H_COMMAND_EXECUTOR

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "urscript_interfaces/srv/ur_script.hpp"
#include "final_project_controller/box_position.h"
#include "final_project_controller/config_loader.h"

class CommandExecutor {
public:
  CommandExecutor(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client,
    std::shared_ptr<ConfigLoader> configLoader
  );

  void executeServiceRequest(const std::string& scriptToExecute)const;
  void closeGripper()const;
  void openGripper()const;
  void runPickAndPlaceSequence()const;
private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client;
  std::shared_ptr<ConfigLoader> configLoader; // config lifetime is almost static, this reference shouldn't be an issue

  std::string openGripperScript;
  std::string closeGripperScript;

  void grabBox(const BoxPosition& position)const;
  void placeBox(const BoxPosition& position)const;
  void executeActionsInSuccession(const std::vector<std::function<void()>>& functs)const;
  std::string hoverAbove(const BoxPosition& boxPosition)const;
  std::string hoverAboveYMargin(const BoxPosition& boxPosition)const;
  std::string hoverAboveYMarginOnEntry(const BoxPosition& boxPosition)const;
  double offsetBy(int n)const;
};
#endif