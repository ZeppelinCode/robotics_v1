#include "robo_miner_controller/param_provider.h"


ParamProvider::ParamProvider(std::shared_ptr<rclcpp::Node> node) : node{node} {
  node->declare_parameter<std::string>(USER_PARAM_NAME, DEFAULT_USER);
  node->declare_parameter<std::string>(REPOSITORY_PARAM_NAME, DEFAULT_REPOSITORY);
  node->declare_parameter<std::string>(COMMIT_SHA_PARAM_NAME, DEFAULT_COMMIT_SHA);
}

UserData ParamProvider::getUserParams() {
    UserData userData;
    node->get_parameter(USER_PARAM_NAME, userData.user);
    node->get_parameter(REPOSITORY_PARAM_NAME, userData.repository);
    node->get_parameter(COMMIT_SHA_PARAM_NAME, userData.commitSha);
    return userData;
}