#ifndef _H_JOINT_ROTATIONS
#define _H_JOINT_ROTATIONS

#include <string>
#include <sstream>

struct JointRotations {
  double elbow{};
  double shoulerLift{};
  double shoulerPan{};
  double wrist1{};
  double wrist2{};
  double wrist3{};

  JointRotations();

  std::string str() {
    std::stringstream representation;
    representation << "JointRotations(" 
                   << "shoulerLift{" << shoulerLift << "}, " 
                   << "shoulerPan{" << shoulerPan << "}, " 
                   << "elbow{" << elbow << "}, " 
                   << "wrist1{" << wrist1 << "}, " 
                   << "wrist2{" << wrist2 << "}, " 
                   << "wrist3{" << wrist3 << "})" ;
    return representation.str();
  }
};

#endif