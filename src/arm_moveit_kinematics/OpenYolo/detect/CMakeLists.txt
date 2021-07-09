cmake_minimum_required(VERSION 3.0.0)
project(arm_moveit_kinematics)

find_package(OpenCV REQUIRED)
add_library(detect detect.cpp)
target_include_directories(detect PUBLIC include)
target_link_libraries(detect ${OpenCV_LIBS})