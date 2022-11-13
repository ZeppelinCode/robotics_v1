#ifndef _H_JOINT_STATE_SUBSCRIBER
#define _H_JOINT_STATE_SUBSCRIBER

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include "final_project_controller/joint_rotations.h"

class JointStateSubscriber {
public:
  JointStateSubscriber(std::shared_ptr<rclcpp::Node> node);
  JointRotations getCurrentJointState();
private:
  std::recursive_mutex jointStateLock{};
  JointRotations _currentJointState{};
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subscriber;
  void joint_state_callback(std::shared_ptr<sensor_msgs::msg::JointState> msg);
};
#endif