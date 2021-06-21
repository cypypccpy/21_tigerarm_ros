cmake_minimum_required (VERSION 2.8)

add_library(SerialPort STATIC SerialPort.cpp)

target_include_directories(SerialPort PUBLIC include)
