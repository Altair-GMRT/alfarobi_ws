cmake_minimum_required(VERSION 2.8.3)
project(team_communication)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
 add_compile_options(-std=c++11)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 team_comm.msg
 robot_data.msg
 team_data.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES win_comm_chat
#  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(server_node src/server_node.cpp)
add_executable(client_node src/client_node.cpp)
# add_executable(communicationSocket src/communicationSocket.cpp)
add_executable(robi src/robi.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(server_node
  ${catkin_LIBRARIES}
)

target_link_libraries(client_node
  ${catkin_LIBRARIES}
)

# target_link_libraries(communicationSocket
#   ${catkin_LIBRARIES}
# )

target_link_libraries(robi
  ${catkin_LIBRARIES}
)
