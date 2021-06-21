add_library(Parameter STATIC Parameter.cpp)

target_link_libraries(Parameter ${OpenCV_LIBS})

target_include_directories(Parameter PUBLIC include)
