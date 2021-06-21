cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0)

find_package(OpenCV REQUIRED)
add_library(detect detect.cpp)
target_include_directories(detect PUBLIC include)
target_link_libraries(detect ${OpenCV_LIBS})