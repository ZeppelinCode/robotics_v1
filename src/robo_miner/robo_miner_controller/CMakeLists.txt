cmake_minimum_required(VERSION 3.10)
project(
  robo_miner_controller
  LANGUAGES CXX
  VERSION 1.0.0
)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robo_miner_interfaces REQUIRED)
find_package(robo_miner_common REQUIRED)

# TODO
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(_INC_FOLDER_NAME include)
set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${_INC_FOLDER_NAME}/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(
  ${PROJECT_NAME}
    ${_SRC_DIR}/main.cpp
    ${_INC_DIR}/map_graph.h
    ${_SRC_DIR}/map_graph.cpp
    ${_INC_DIR}/robo_miner_experiment.h
    ${_SRC_DIR}/robo_miner_experiment.cpp
    ${_INC_DIR}/mover_communicator.h
    ${_SRC_DIR}/mover_communicator.cpp
    ${_INC_DIR}/miner_navigator.h
    ${_SRC_DIR}/miner_navigator.cpp
    ${_INC_DIR}/coordinate_remapper.h
    ${_SRC_DIR}/coordinate_remapper.cpp
    ${_INC_DIR}/param_provider.h
    ${_SRC_DIR}/param_provider.cpp
    ${_INC_DIR}/shortest_path.h
    ${_SRC_DIR}/shortest_path.cpp
)

target_include_directories(robo_miner_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# TODO
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        robo_miner_common::robo_miner_common
)

ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        rclcpp
        ament_index_cpp
        robo_miner_interfaces
        std_msgs
)


install(TARGETS robo_miner_controller
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
