cmake_minimum_required(VERSION 3.10)
project(
  final_project_controller 
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
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ur_control_gui REQUIRED)
find_package(urscript_interfaces REQUIRED)
find_package(urscript_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
    ${_INC_DIR}/box_position.h
    ${_SRC_DIR}/box_position.cpp
    ${_INC_DIR}/box_position_loader.h
    ${_SRC_DIR}/box_position_loader.cpp
    ${_INC_DIR}/misc.h
    ${_SRC_DIR}/misc.cpp
    ${_INC_DIR}/joint_state_subscriber.h
    ${_SRC_DIR}/joint_state_subscriber.cpp
    ${_INC_DIR}/joint_rotations.h
    ${_SRC_DIR}/joint_rotations.cpp
    ${_INC_DIR}/angle_axis_provider.h
    ${_SRC_DIR}/angle_axis_provider.cpp
)

target_include_directories(final_project_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
        rclcpp
        ament_index_cpp
        std_msgs
        sensor_msgs
        urscript_interfaces
        tf2
        tf2_ros
        tf2_geometry_msgs
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        urscript_common::urscript_common
        # ${Boost_LIBRARIES}
        # boost_thread #boosts linker flag for -lpthread
)

install(TARGETS final_project_controller
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
