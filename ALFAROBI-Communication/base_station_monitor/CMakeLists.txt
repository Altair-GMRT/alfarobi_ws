cmake_minimum_required(VERSION 2.8.3)
project(base_station_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
# robot_komunikasi
  roscpp
  std_msgs
#  team_communication
)

set(Qt5_DIR "/opt/qt59/lib/cmake/Qt5")

find_package(Qt5 REQUIRED COMPONENTS Gui Widgets)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES base_station_monitor
#  CATKIN_DEPENDS geometry_msgs robot_komunikasi roscpp std_msgs team_communication
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(base_station_node base_monitor_node.cpp
     basemonitor.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(base_station_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(base_station_node
   ${catkin_LIBRARIES}
   Qt5::Gui
   Qt5::Core
   Qt5::Widgets
 )
