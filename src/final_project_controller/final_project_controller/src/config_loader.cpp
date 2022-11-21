#include <iostream>
#include <fstream>
#include "final_project_controller/config_loader.h"
#include "final_project_controller/json_parser.h"
#include "final_project_controller/box_position_loader.h"

ConfigLoader::ConfigLoader() {
    using json = nlohmann::json;
    std::ifstream f("/home/ubuntu/workspace/robotics_v1/src/final_project_controller/final_project_controller/config/json_config.json");
    json config = json::parse(f);

    this->isGripperEnabled = config["isGripperEnabled"];
    this->offsetBetweenBoxes = config["offsetBetweenBoxes"];
    this->minimumHoverHeight = config["minimumHoverHeight"];
    this->hoverAboveBox = config["hoverAboveBox"];
    this->initialTargetCoordinates.x = config["targetX"];
    this->initialTargetCoordinates.y = config["targetY"];
    this->initialTargetCoordinates.z = config["targetZ"];
    this->easyBoxRotations.x = config["easyBoxRotationX"];
    this->easyBoxRotations.y = config["easyBoxRotationY"];
    this->easyBoxRotations.z = config["easyBoxRotationZ"];
    this->fourthStairWeirdRotations.x = config["fourthStairWeirdRotationX"];
    this->fourthStairWeirdRotations.y = config["fourthStairWeirdRotationY"];
    this->fourthStairWeirdRotations.z = config["fourthStairWeirdRotationZ"];
    this->finalOrientation.x = config["finalOrientationX"];
    this->finalOrientation.y = config["finalOrientationY"];
    this->finalOrientation.z = config["finalOrientationZ"];
    this->blendingRadius = config["blendingRadius"];
    this->additionalBackAwayYSafety = config["additionalBackAwayYSafety"];
    this->avoidBaseLocation.x = config["avoidBaseLocation"]["x"];
    this->avoidBaseLocation.y = config["avoidBaseLocation"]["y"];
    this->avoidBaseLocation.z = config["avoidBaseLocation"]["z"];
    this->shouldAvoidBaseLocation = config["shouldAvoidBaseLocation"];
    
    std::cout << "config read:" << std::endl;
    std::cout << "isGripperEnabled: " << isGripperEnabled << std::endl;
    std::cout << "offset offset between boxes: " << offsetBetweenBoxes << std::endl;
    std::cout << "minimum hover height: " << minimumHoverHeight<< std::endl;
    std::cout << "hover above box: " << hoverAboveBox << std::endl;
    std::cout << "target x: " << initialTargetCoordinates.x << std::endl;
    std::cout << "target y: " << initialTargetCoordinates.y << std::endl;
    std::cout << "target z: " << initialTargetCoordinates.z << std::endl;
    std::cout << "easy rx: " << easyBoxRotations.x << std::endl;
    std::cout << "easy ry: " << easyBoxRotations.y << std::endl;
    std::cout << "easy rz: " << easyBoxRotations.z << std::endl;
    std::cout << "4s weird rx: " << fourthStairWeirdRotations.x << std::endl;
    std::cout << "4s weird ry: " << fourthStairWeirdRotations.y << std::endl;
    std::cout << "4s weird rz: " << fourthStairWeirdRotations.z << std::endl;
    std::cout << "final rx: " << finalOrientation.x << std::endl;
    std::cout << "final ry: " << finalOrientation.y << std::endl;
    std::cout << "final rz: " << finalOrientation.z << std::endl;
    std::cout << "blending radius: " << blendingRadius << std::endl;
    std::cout << "additional back away y safety: " << additionalBackAwayYSafety << std::endl;
    std::cout << "avoid base location (" << avoidBaseLocation.x << ", " << avoidBaseLocation.y << ", " <<  avoidBaseLocation.z << ")" << std::endl;
    std::cout << "should avoid base location(" << shouldAvoidBaseLocation << std::endl;

    std::cout << "loading box positions" << std::endl;
    this->boxPositions = bpl::loadBoxPositions();
    for (const auto& bp: boxPositions) {
      std::cout << bp.str() << std::endl;
    }
    std::cout << "done loading box positions" << std::endl;
}