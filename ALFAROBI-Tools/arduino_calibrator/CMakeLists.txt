cmake_minimum_required(VERSION 2.8)
project(arduino_calibrator)

add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib)

set(Qt5_DIR "/opt/qt59/lib/cmake/Qt5")
#set(Qt5Widgets_DIR "/opt/qt59/lib/cmake/Qt5Widgets")
#set(Qt5Gui_DIR "/opt/qt59/lib/cmake/Qt5Gui")
#set(Qt5SerialPort_DIR "/opt/qt59/lib/cmake/Qt5SerialPort")


set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Gui Widgets SerialPort Qml Charts OpenGL)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs roslib
)

include_directories(
    ${YAML_CPP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
 )

add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    arduinocontroller.cpp
    robotitem.cpp
    chart.cpp
    mainwindow.ui
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES}
    Qt5::OpenGL Qt5::Charts Qt5::Gui Qt5::Core Qt5::SerialPort Qt5::Widgets)
