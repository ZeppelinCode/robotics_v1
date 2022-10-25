#ifndef ROBO_MINER_COMMON_ROBOMINERTOPICS_H_
#define ROBO_MINER_COMMON_ROBOMINERTOPICS_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

constexpr auto USER_AUTHENTICATE_TOPIC = "user_authenticate";
constexpr auto ROBOT_MOVE_SERVICE = "move_robot";
constexpr auto QUERY_INITIAL_ROBOT_POSITION_SERVICE = "query_initial_robot_position";
constexpr auto FIELD_MAP_VALIDATE_SERVICE = "field_map_validate";
constexpr auto LONGEST_SEQUENCE_VALIDATE_SERVICE = "longest_sequence_validate";
constexpr auto ACTIVATE_MINING_VALIDATE_SERVICE = "activate_mining_validate";
constexpr auto LONGEST_SEQUENCE_CALCULATE_SERVICE = "longest_sequence_calculate";

constexpr auto FIELD_MAP_REVEALED_TOPIC = "field_map_revealed";
constexpr auto SHUTDOWN_CONTROLLER_TOPIC = "shutdown_controller";
constexpr auto TOGGLE_DEBUG_INFO_TOPIC = "toggle_debug_info";
constexpr auto TOGGLE_HELP_PAGE_TOPIC = "toggle_help_page";
constexpr auto DEBUG_MSG_TOPIC = "debug_msg";

#endif /* ROBO_MINER_COMMON_ROBOMINERTOPICS_H_ */
