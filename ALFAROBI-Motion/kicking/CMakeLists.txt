################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(kicking)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    dynamixel_sdk
    eigen_conversions
    robotis_controller_msgs
    robotis_device
    robotis_framework_common
    kdl_conversions
    robotis_math
    roscpp
    kicking_module_msgs
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
    LIBRARIES kicking
    LIBRARIES walk_zmp
    )

################################################################################
# Build
################################################################################
include_directories(
    include
    ../walk_zmp/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/kicking.cpp
    ../walk_zmp/src/kinematics.cpp
    ../walk_zmp/src/zmp_team_darwin_kinematics.cpp
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
