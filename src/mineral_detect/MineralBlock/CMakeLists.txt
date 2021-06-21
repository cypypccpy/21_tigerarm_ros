add_library(MineralBlock STATIC Engineer_Locator.cpp Engineer_Corner.cpp Engineer_Block.cpp Engineer_Block.cpp)

target_link_libraries(MineralBlock ${OpenCV_LIBS})
target_link_libraries(MineralBlock Parameter SerialPort CmdlineParser)

link_directories(include)
target_include_directories(MineralBlock PUBLIC include)
