################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_manager)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules

  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math

  op3_base_module
  op3_head_control_module
  op3_action_module
  op3_kinematics_dynamics
  model_based_control
  walk_zmp
  kicking
  robot_state_module
  odometry
  capture_step_module
  capture_step_module_msgs
  quintic_walk
)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)
set(Qt5_DIR "/opt/qt59/lib/cmake/Qt5")
find_package(Qt5 REQUIRED COMPONENTS SerialPort)
find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES op3_manager
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(op3_manager src/op3_manager.cpp)

add_dependencies(op3_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_manager
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Eigen3_LIBRARIES}
  Qt5::SerialPort
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
