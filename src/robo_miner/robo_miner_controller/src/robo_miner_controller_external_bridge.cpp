#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "robo_miner_controller/robo_miner_controller_external_bridge.h"
// #include "robo_miner_common/defines/RoboMinerTopics.h"
#include "robo_miner_interfaces/msg/robot_position_response.hpp"

namespace {
    constexpr auto NODE_NAME = "RoboMinerControllerExternalBridge";


    template <typename T>
    void waitForService(const T &client) {
        using namespace std::literals;
        while (!client->wait_for_service(1s)) {
            std::cout << "Service: [" << client->get_service_name()
            << "] not available. Waiting for 1s ..." << std::endl;
        }
    }

}

RoboMinerControllerExternalBridge::RoboMinerControllerExternalBridge() : Node(NODE_NAME) {

}

int32_t RoboMinerControllerExternalBridge::init() {
    constexpr auto queueSize = 10;
    const rclcpp::QoS qos(queueSize);

    // initialRobotPositionClient = create_client<QueryInitialRobotPosition>(QUERY_INITIAL_ROBOT_POSITION_SERVICE);
    initialRobotPositionClient = create_client<QueryInitialRobotPosition>("query_initial_robot_position", rmw_qos_profile_services_default);
    waitForService(initialRobotPositionClient);

    return EXIT_SUCCESS;
}

void RoboMinerControllerExternalBridge::run() {
    using RobotPositionResponse = robo_miner_interfaces::msg::RobotPositionResponse;
    auto request = std::make_shared<QueryInitialRobotPosition::Request>();
    std::cout << "sending request to " << initialRobotPositionClient->get_service_name() << std::endl;
    auto response = this->initialRobotPositionClient->async_send_request(request);
    std::cout << "waiting for response" << std::endl;
    const std::shared_ptr<QueryInitialRobotPosition::Response> responseData;
    
    // if (rclcpp::spin_until_future_complete(this, response) == rclcpp::FutureReturnCode::SUCCESS) {
    //  responseData = response.get();
    // }
    uint8_t initialTile = responseData->robot_initial_tile;
    const RobotPositionResponse rpr = responseData->robot_position_response;

    std::cout << "initial tile " << initialTile << std::endl;
    std::cout << "orientation: " << rpr.robot_dir << std::endl;

    std::cout << "surrounding tiles" << std::endl;
    for (int i = 0; i < 3; i++) {
        std::cout << rpr.surrounding_tiles[i] << " ";
    }
    std::cout << std::endl;
}