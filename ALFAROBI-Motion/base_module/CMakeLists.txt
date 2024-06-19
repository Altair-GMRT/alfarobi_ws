################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_base_module)

add_compile_options(-std=c++11)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  robotis_device
  robotis_math
  dynamixel_sdk
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  op3_kinematics_dynamics
)

find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

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
  INCLUDE_DIRS include
  LIBRARIES op3_base_module
  CATKIN_DEPENDS robotis_math op3_kinematics_dynamics
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(op3_base_module
  src/base_module.cpp
  src/base_module_state.cpp
)

add_dependencies(op3_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_base_module
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
