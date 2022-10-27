cmake_minimum_required(VERSION 3.5)
project(robo_cleaner_controller)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robo_cleaner_interfaces REQUIRED)
find_package(robo_cleaner_common REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
  ${PROJECT_NAME}
    ${_SRC_DIR}/main.cpp
    ${_INC_DIR}/robo_cleaner_external_bridge.h
    ${_SRC_DIR}/robo_cleaner_external_bridge.cpp
    ${_INC_DIR}/map_graph.h
    ${_SRC_DIR}/map_graph.cpp
    ${_INC_DIR}/shortest_path.h
    ${_SRC_DIR}/shortest_path.cpp
    ${_INC_DIR}/robot_state.h
    ${_SRC_DIR}/robot_state.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
      robo_cleaner_common::robo_cleaner_common
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        rclcpp
        rclcpp_action
        ament_index_cpp
        robo_cleaner_interfaces
        std_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
