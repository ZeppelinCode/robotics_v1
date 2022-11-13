#ifndef _H_ANGLE_AXIS_PROVIDER
#define _H_ANGLE_AXIS_PROVIDER

#include <rclcpp/rclcpp.hpp>
#include "urscript_interfaces/srv/get_eef_angle_axis.hpp"
#include <geometry_msgs/msg/vector3.hpp>

class AngleAxisProvider {
public:
  AngleAxisProvider(std::shared_ptr<rclcpp::Node> node);
  geometry_msgs::msg::Vector3 getAngleAxis();
private:
  rclcpp::Node::SharedPtr _node;
  rclcpp::Client<urscript_interfaces::srv::GetEefAngleAxis>::SharedPtr _angleAxisClient;
};
#endif