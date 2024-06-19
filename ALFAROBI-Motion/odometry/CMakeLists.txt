################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(odometry)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    dynamixel_sdk
    eigen_conversions
    op3_kinematics_dynamics
    robotis_controller
    robotis_controller_msgs
    robotis_device
    robotis_framework_common
    robotis_math
    geometry_msgs
    roscpp
    rc_utils
    kdl_conversions
    robot_state_publisher
    )

find_package(Eigen3 REQUIRED)
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
    LIBRARIES odometry
    )

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    ${LAPACK_INCLUDE_DIRS}
    ${BLAS_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    )

file(GLOB_RECURSE utility)

add_library(${PROJECT_NAME}
    src/odometry.cpp
    ${utility}

    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${YAML_CPP_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    )

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################

