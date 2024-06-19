################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(walk_zmp)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    dynamixel_sdk
    eigen_conversions
    op3_kinematics_dynamics
    robotis_controller_msgs
    robotis_device
    robotis_framework_common
    robotis_math
    roscpp
    rc_utils
    kdl_conversions
    robot_state_publisher
    tf2_kdl
    tf2_ros
    zmp_walking_module_msgs
    tf
    tf2    
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
    LIBRARIES walk_zmp
    )

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/walk_zmp.cpp
    src/zmp_walk_parameter.cpp
    src/zmp_team_darwin_kinematics.cpp
    src/kinematics.cpp
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
