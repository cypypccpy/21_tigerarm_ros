add_library(CmdlineParser STATIC CmdlineParser.cpp)

link_directories(include)

target_include_directories(CmdlineParser PUBLIC include)
