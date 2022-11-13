#include "final_project_controller/joint_state_subscriber.h"
#include <iostream>


JointStateSubscriber::JointStateSubscriber(std::shared_ptr<rclcpp::Node> node) : node{node} {
    _subscriber = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&JointStateSubscriber::joint_state_callback, this, std::placeholders::_1)
        );
}

void JointStateSubscriber::joint_state_callback(std::shared_ptr<sensor_msgs::msg::JointState> msg) {
    std::lock_guard<std::recursive_mutex> l{jointStateLock};
// - elbow_joint
// - shoulder_lift_joint
// - shoulder_pan_joint
// - wrist_1_joint
// - wrist_2_joint
// - wrist_3_joint
    for (size_t i = 0; i < msg->name.size(); i++) {
        auto jointName = msg->name[i];
        if (jointName == "elbow_joint") {
            _currentJointState.elbow = msg->position[i];
            continue;
        }
        if (jointName == "shoulder_lift_joint") {
            _currentJointState.shoulerLift = msg->position[i];
            continue;
        }
        if (jointName == "shoulder_pan_joint") {
            _currentJointState.shoulerPan = msg->position[i];
            continue;
        }
        if (jointName == "wrist_1_joint") {
            _currentJointState.wrist1 = msg->position[i];
            continue;
        }
        if (jointName == "wrist_2_joint") {
            _currentJointState.wrist2 = msg->position[i];
            continue;
        }
        if (jointName == "wrist_3_joint") {
            _currentJointState.wrist3 = msg->position[i];
            continue;
        }
    }
    // std::cout << "joint state update " << _currentJointState.str() << std::endl;
}


JointRotations JointStateSubscriber::getCurrentJointState() {
    std::lock_guard<std::recursive_mutex> l{jointStateLock};
    return _currentJointState;
}
