cmake_minimum_required(VERSION 2.8.3)
project(quintic_walk)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  std_msgs
  quintic_walk_msgs
  robotis_controller_msgs
  robotis_device
  robotis_framework_common
  robotis_math
  tf
  tf_conversions
  eigen_conversions
  op3_kinematics_dynamics
  model_based_control
  analyze_msgs
  densis_msgs
  
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS system chrono thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quintic_walk
#  CATKIN_DEPENDS robotis_controller_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/quinticwalk.cpp
   src/walk_engine.cpp
   src/Footstep.cpp
   src/AnalyticIKSolver.cpp
   src/Utils/Combination.cpp
   src/Utils/NewtonBinomial.cpp
   src/Utils/TrajectoryUtils.cpp
   src/Spline/Polynom.cpp
   src/Spline/SmoothSpline.cpp
   src/Spline/Spline.cpp
   src/fuzzy.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
   ${Eigen3_LIBRARIES}
   ${Boost_LIBRARIES}
 )
