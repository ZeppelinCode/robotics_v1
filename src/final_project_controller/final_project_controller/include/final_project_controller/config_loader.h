#ifndef _H_CONFIG_LOADER
#define _H_CONFIG_LOADER

#include <string>
#include <vector>
#include "final_project_controller/box_position.h"

template <typename T>
struct Vec3 {
  T x;
  T y;
  T z;
};

struct ConfigLoader {
// public:
  ConfigLoader();
  void loadConfig();
// private:
  bool isGripperEnabled{};
  double offsetBetweenBoxes{};
  double minimumHoverHeight{};
  double hoverAboveBox{};
  Vec3<double> initialTargetCoordinates{};
  Vec3<double> easyBoxRotations{};
  Vec3<double> fourthStairWeirdRotations{};
  Vec3<double> finalOrientation{};
  std::vector<BoxPosition> boxPositions;
  double blendingRadius;
};
#endif