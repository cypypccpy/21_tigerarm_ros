cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0)

find_package(OpenCV REQUIRED)
add_library(PlaneExtract PlaneExtract.cpp)
target_include_directories(PlaneExtract PUBLIC include)
target_link_libraries(PlaneExtract ${OpenCV_LIBS})