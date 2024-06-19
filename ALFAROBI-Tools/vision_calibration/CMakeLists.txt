cmake_minimum_required(VERSION 3.0.2)
project(vision_calibration)
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set(Qt5_DIR "/opt/qt59/lib/cmake/Qt5")
set(Qt5Widgets_DIR "/opt/qt59/lib/cmake/Qt5Widgets")
set(Qt5Gui_DIR "/opt/qt59/lib/cmake/Qt5Gui")
set(OpenCV_DIR "usr/local/share/OpenCV")
find_package(catkin REQUIRED roscpp)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV 3.3 REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME} "main.cpp"
    image_clip.cpp
    main_window.cpp
    gui.ui
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)

