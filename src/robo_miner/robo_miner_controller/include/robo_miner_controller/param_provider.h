#ifndef _H_PARAM_PROVIDER
#define _H_PARAM_PROVIDER

#include <rclcpp/rclcpp.hpp>
#include <string>

struct UserData {
  std::string user = "not_set";
  std::string repository = "not_set";
  std::string commitSha = "not_set";
};

namespace {
  constexpr auto USER_PARAM_NAME = "user";
  constexpr auto REPOSITORY_PARAM_NAME = "repository";
  constexpr auto COMMIT_SHA_PARAM_NAME = "commit_sha";

  constexpr auto DEFAULT_USER = "not_set";
  constexpr auto DEFAULT_REPOSITORY = "not_set";
  constexpr auto DEFAULT_COMMIT_SHA = "not_set";
}

class ParamProvider {
public:
  ParamProvider(std::shared_ptr<rclcpp::Node> node);
  UserData getUserParams();
private:
  std::shared_ptr<rclcpp::Node> node;
};
#endif