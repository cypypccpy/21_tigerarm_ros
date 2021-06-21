//
// Created by Carcuis on 2021/4/2.
//

#ifndef INC_21_VISION_AERIAL_CMDLINEPARSER_H
#define INC_21_VISION_AERIAL_CMDLINEPARSER_H

#include "cmdline.h"
#include <iostream>
#include <sys/utsname.h>
using namespace std;

class CmdlineParser
{
public:
    CmdlineParser(int argc, char *argv[]);

    bool input_from_file = false;
    string input_file_path;
    bool record_output_frame = false;
    bool record_color_frame = false;
    bool record_binary = false;
    bool record_edge = false;
    string record_file_path;
    bool user_frame_rate = false;
    int frame_rate = 60;
    bool show_GUI_windows = false;
    bool show_color_frame = false;
    bool show_binary = false;
    bool show_edge = false;
    bool original_frame = false;
    bool manual_wb = false;
    bool manual_set_exposure = false;
    int exposure;
    int gain;
    bool manual_set_gain = false;
    bool multi_cam = false;
    bool full_screen = false;
    bool draw_aux_line = false;
    int initial_state = 0;
    bool manual_state = false;
    bool with_yolo = false;
    bool manual_mask_rect = false;
    float up_rect_ratio = 0.0;
    float down_rect_ratio = 0.0;
    float left_rect_ratio = 0.0;
    float right_rect_ratio = 0.0;
    int manual_mask_state = 0;

private:
    cmdline::parser parser;
    struct utsname uts{};
    string system_name;
//    string record_file_dir;
    string file_name;
//    int argc;
//    char **argv;
};

#endif //INC_21_VISION_AERIAL_CMDLINEPARSER_H
