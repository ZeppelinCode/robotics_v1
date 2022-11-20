
#include <algorithm> // std::min
#include "final_project_controller/command_executor.h"
#include "final_project_controller/config_loader.h"
#include "final_project_controller/misc.h"

std::string goLinearlyTo(BoxPosition boxPosition) {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", " << boxPosition.z
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end\n\n\n";
  return acc.str();
}

std::string CommandExecutor::hoverAbove(const BoxPosition& boxPosition)const {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y 
      << ", " << std::max(configLoader->minimumHoverHeight, boxPosition.z + configLoader->hoverAboveBox) // 0.15
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end\n\n\n";
  return acc.str();
}

std::string CommandExecutor::hoverAboveYMargin(const BoxPosition& boxPosition)const {
  std::stringstream acc;
  acc << "def go_linearly_to():\n";
  acc << "\tmovel(p[" 
      << boxPosition.x
      << ", " << boxPosition.y + configLoader->offsetBetweenBoxes
      << ", " << std::max(configLoader->minimumHoverHeight, boxPosition.z + configLoader->hoverAboveBox) // 0.15
      << ", " << boxPosition.rx
      << ", " << boxPosition.ry
      << ", " << boxPosition.rz
      << "], a=1.0, v=0.8)\n";
  acc << "end\n\n\n";
  return acc.str();
}

CommandExecutor::CommandExecutor(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<urscript_interfaces::srv::UrScript>::SharedPtr client,
  std::shared_ptr<ConfigLoader> configLoader
) : node{node}, client{client}, configLoader{configLoader} {

  this->openGripperScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/open_gripper.urscript");
  this->closeGripperScript = misc::readFileToString("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/resources/close_gripper.urscript");
}

void CommandExecutor::executeServiceRequest(const std::string &scriptToExecute)const
{
  using UrScriptSrv = urscript_interfaces::srv::UrScript;
  auto request = std::make_shared<UrScriptSrv::Request>();
  request->data = scriptToExecute;
  auto result = client->async_send_request(request);
  std::cout << scriptToExecute << std::endl;
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to move service with command forward");
  }
  auto response = result.get();
  std::cout << "----" << std::endl;
}

void CommandExecutor::openGripper()const {
  if (!configLoader->isGripperEnabled) {
    std::cout << "opening gripper 0" << std::endl;
    return;
  }
  std::cout << "opening gripper 1" << std::endl;
  executeServiceRequest(openGripperScript);
}

void CommandExecutor::closeGripper()const {
  if (!configLoader->isGripperEnabled) {
    std::cout << "closing gripper 0" << std::endl;
    return;
  }
  std::cout << "closing gripper 1" << std::endl;
  executeServiceRequest(closeGripperScript);
}

void CommandExecutor::executeActionsInSuccession(const std::vector<std::function<void()>>& functs)const {
  int i = 1;
  for (const auto& func: functs) {
    func();
    std::cout << "action #" << i++ << std::endl;
  }
}

void CommandExecutor::grabBox(const BoxPosition& position)const {
    executeServiceRequest(hoverAbove(position));
    executeServiceRequest(goLinearlyTo(position));

    closeGripper();

    executeServiceRequest(hoverAbove(position));
}

void CommandExecutor::placeBox(const BoxPosition& position)const {
  executeServiceRequest(hoverAboveYMargin(position));
  executeServiceRequest(hoverAbove(position));
  executeServiceRequest(goLinearlyTo(position));

  openGripper();

  executeServiceRequest(hoverAbove(position));
  executeServiceRequest(hoverAboveYMargin(position));
}

double CommandExecutor::offsetBy(int n)const {
  return configLoader->offsetBetweenBoxes * n; // 0.115 * n
}

void CommandExecutor::runPickAndPlaceSequence()const {
  const auto& boxPositions = configLoader->boxPositions;
  const auto& initTarget = configLoader->initialTargetCoordinates;
  const auto& easyRotations = configLoader->easyBoxRotations;
  const auto& weird4sTargetRotations = configLoader->fourthStairWeirdRotations;
  const auto& finalOrientation = configLoader->finalOrientation;
  std::vector<std::function<void()>> functions;

  // 10, 1, 2, 11, 3, 4, 12, 5, 13, 7

  // First stair
  functions.emplace_back([&] { grabBox(boxPositions[9]);  }); // Grip before ascent
  functions.emplace_back([&] { placeBox(BoxPosition(
    initTarget.x, initTarget.y, initTarget.z, easyRotations.x, easyRotations.y, easyRotations.z));  
  });

  // Second stair
  functions.emplace_back([&] { grabBox(boxPositions[0]);  });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y + offsetBy(1), initTarget.z, easyRotations.x, easyRotations.y, easyRotations.z));  
  });
  functions.emplace_back([&] { grabBox(boxPositions[1]);  });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y + offsetBy(1), initTarget.z + offsetBy(1), easyRotations.x, easyRotations.y, easyRotations.z));  
  });

  // Third stair
  functions.emplace_back([&] { grabBox(boxPositions[10]); });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y+ offsetBy(2), initTarget.z, easyRotations.x, easyRotations.y, easyRotations.z));  
  });
  functions.emplace_back([&] { grabBox(boxPositions[2]); });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y + offsetBy(2), initTarget.z + offsetBy(1), easyRotations.x, easyRotations.y, easyRotations.z));  
  });
  functions.emplace_back([&] { grabBox(boxPositions[3]); });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y + offsetBy(2), initTarget.z + offsetBy(2), easyRotations.x, easyRotations.y, easyRotations.z));  
  });

  // Fourth stair
  functions.emplace_back([&] { grabBox(boxPositions[11]); });
  functions.emplace_back([&] { placeBox(
    BoxPosition(initTarget.x, initTarget.y + offsetBy(3), initTarget.z, easyRotations.x, easyRotations.y, easyRotations.z));  
  });
  functions.emplace_back([&] { grabBox(boxPositions[4]);  });
  functions.emplace_back([&] { placeBox(BoxPosition(
    initTarget.x, initTarget.y+ offsetBy(3), initTarget.z+ offsetBy(1), easyRotations.x, easyRotations.y, easyRotations.z));  
  });

  // Fourth stair: weird pick orientation
  functions.emplace_back([&] { grabBox(boxPositions[12]);  });
  functions.emplace_back([&] { placeBox(BoxPosition( // 0.197, -2.206, 1.829
    initTarget.x, initTarget.y+ offsetBy(3), initTarget.z + offsetBy(2), weird4sTargetRotations.x, weird4sTargetRotations.y, weird4sTargetRotations.z));
  });
  functions.emplace_back([&] { grabBox(boxPositions[6]);  });
  functions.emplace_back([&] { placeBox( // 0.197, -2.206, 1.829
    BoxPosition(initTarget.x, initTarget.y + offsetBy(3), initTarget.z + offsetBy(3), weird4sTargetRotations.x, weird4sTargetRotations.y, weird4sTargetRotations.z));
  });

  // 8, 6, 14, 9
  // Final 4 cubes
  // Can be done by hovering above
  functions.emplace_back([&] { grabBox(boxPositions[7]);  });
  functions.emplace_back([&] { placeBox(  // 0.197, -2.206, 1.829
    BoxPosition(initTarget.x, initTarget.y + offsetBy(3), initTarget.z + offsetBy(4), weird4sTargetRotations.x, weird4sTargetRotations.y, weird4sTargetRotations.z));
  });
  functions.emplace_back([&] { grabBox(boxPositions[5]);  });
  functions.emplace_back([&] { placeBox(BoxPosition(
    initTarget.x, initTarget.y + offsetBy(3), initTarget.z + offsetBy(5), easyRotations.x, easyRotations.y, easyRotations.z));
  });

  // Need to approach from the side
  functions.emplace_back([&] { grabBox(boxPositions[13]); });
  functions.emplace_back([&] { executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(4), initTarget.z + offsetBy(6), finalOrientation.x, finalOrientation.y, finalOrientation.z))); 
  });
  functions.emplace_back([&] { executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(3), initTarget.z+ offsetBy(6), finalOrientation.x, finalOrientation.y, finalOrientation.z))); 
  });
  functions.emplace_back([&] { openGripper(); });
  functions.emplace_back([&] { executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(4), initTarget.z + offsetBy(6), finalOrientation.x, finalOrientation.y, finalOrientation.z)));
  });

  // Need to approach from the side
  functions.emplace_back([&] { grabBox(boxPositions[8]);  });
  functions.emplace_back([&] { executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(4), initTarget.z + offsetBy(7), finalOrientation.x, finalOrientation.y, finalOrientation.z)));
  });
  functions.emplace_back([&] {executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(3), initTarget.z + offsetBy(7), finalOrientation.x, finalOrientation.y, finalOrientation.z)));
  });
  functions.emplace_back([&] { openGripper(); });
  functions.emplace_back([&] { executeServiceRequest(goLinearlyTo(BoxPosition( // 0.022, -2.758, 2.493
    initTarget.x, initTarget.y + offsetBy(4), initTarget.z + offsetBy(7), finalOrientation.x, finalOrientation.y, finalOrientation.z)));
  });
  executeActionsInSuccession(functions);
}