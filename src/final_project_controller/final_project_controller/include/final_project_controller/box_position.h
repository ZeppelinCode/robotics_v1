#ifndef _H_BOX_POSITION
#define _H_BOX_POSITION

#include <string>

struct BoxPosition {
  BoxPosition(double x, double y, double z, double rx, double ry, double rz);
  std::string str()const;

  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
};
#endif