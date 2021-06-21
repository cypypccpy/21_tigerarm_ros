//
// Created by Carcuis on 2021/4/2.
//

#include "CmdlineParser.h"

CmdlineParser::CmdlineParser(int argc, char *argv[])
{
    parser.add<string>("input", 'i', "input from native video file", false, "/path/to/video");
    parser.add("show", 's', "show all GUI output frame window(s)");
    parser.add<int>("chooseShowFrame", 'S', "choose frame to show, color-binary-edge", false, 111);
    parser.add("original", 'o', "toggle original camera stream mode");
    parser.add("record", 'r', "record stream to default ~/Videos/,linux or ~/Movies/,darwin");
    parser.add<int>("chooseRecordFrame", 'R', "choose which frame to record, color-binary-edge", false, 111);
    parser.add<string>("outputFile", 'f', "specify record file path", false, "/path/to/record01.avi");
    parser.add<int>("frameRate", 'F', "specify record frame rate", false, 60, cmdline::range(10, 200));
    parser.add("manualWB", 'b', "toggle manual white balance");
    parser.add<int>("exposure", 'e', "set camera exposure", false, 700, cmdline::range(1, 1000000));
    parser.add<int>("gain", 'g', "set camera gain", false, 40, cmdline::range(0, 1000));
    parser.add("multiCam", 'm', "enable multi camera mode");
    parser.add("fullScreen", 'l', "enable full screen");
    parser.add("drawAuxLine", 'x', "draw auxiliary line for the operator");
    parser.add<int>("initState", 't', "declare initial action state", false, 0, cmdline::range(0, 3));
    parser.add("manualState", 'T', "toggle manual state mode");
    parser.add("withYolo", 'y', "detect with yolo submodule");
    parser.add<int>("manualMaskRect", 'M', "manually set mask rect, up-down-left-right-state", false, 102030403);

    parser.parse_check(argc, argv);

    if (parser.exist("input"))
    {
        this->input_from_file = true;
        this->input_file_path = parser.get<string>("input");
    }

    if (parser.exist("original"))
    {
        this->original_frame = true;
        cout << "\033[40;1;36mOriginal frame ON\033[0m" << endl;
    }

    if (parser.exist("frameRate"))
    {
        this->user_frame_rate = true;
        this->frame_rate = parser.get<int>("frameRate");
    }

    if (parser.exist("record") || parser.exist("chooseRecordFrame"))
    {
        if (parser.exist("chooseRecordFrame"))
        {
            int s = parser.get<int>("chooseRecordFrame");
            this->record_color_frame = s / 100;
            this->record_binary = s % 100 / 10;
            this->record_edge  = s % 10;
            if ((record_binary || record_edge) && original_frame)
            {
                this->record_binary = false;
                this->record_edge = false;
                cout << "\033[40;1;35mCan only record original frame with --original!\033[0m" << endl;
                cout << parser.usage() << endl;
                exit(1);
            }
            if ((record_color_frame + record_binary + record_edge) != 1)
            {
                cout << "\033[40;1;35mPlease choose one frame to record!\033[0m" << endl;
                cout << parser.usage() << endl;
                exit(1);
            }
        } else
        {
            this->record_color_frame = true;
            this->record_binary = false;
            this->record_edge = false;
        }

        this->record_output_frame = true;
        string HOME = getenv("HOME");
        if (parser.exist("outputFile"))
        {
            this->record_file_path = parser.get<string>("outputFile");
        } else
        {
            time_t t = time(nullptr);
            char t_str[64];
            strftime(t_str, sizeof(t_str), "%Y%m%d_%H%M%S", localtime(&t));
            this->file_name = (string)t_str;
            uname(&uts);
            this->system_name = uts.sysname;
            if (system_name == "Linux")
            {
                this->record_file_path = HOME + "/Videos/" + file_name + ".avi";
            } else if (system_name == "Darwin")
            {
                this->record_file_path = HOME + "/Movies/" + file_name + ".avi";
            } else
            {
                cout << system_name << " (system type) is not supported" << endl;
                exit(1);
            }
        }
        cout << "\033[40;1;32mRecording to " << record_file_path << " ...\033[0m" << endl;
        cout << "\033[40;1;32mRecording frame rate: " << frame_rate << " per second\033[0m" << endl;
    }

    if (parser.exist("show") && !parser.exist("chooseShowFrame"))
    {
        this->show_GUI_windows = true;
        this->show_color_frame = true;
        this->show_binary = !original_frame;
        this->show_edge = !original_frame;
        cout << "\033[40;1;33mGUI windows ON\033[0m" << endl;
    }

    if (parser.exist("chooseShowFrame"))
    {
        int s = parser.get<int>("chooseShowFrame");
        this->show_color_frame = s / 100;
        this->show_binary = s % 100 / 10;
        this->show_edge  = s % 10;
        if ((show_binary || show_edge) && original_frame)
        {
            this->show_binary = false;
            this->show_edge = false;
            cout << "\033[40;1;35mCan only show original frame with --original!  Turning off GUI...\033[0m" << endl;
        }
        if (show_color_frame || show_edge || show_binary)
        {
            this->show_GUI_windows = true;
            cout << "\033[40;1;33mGUI windows ON\033[0m" << endl;
        } else
        {
            this->show_GUI_windows = false;
        }
    }


    if (parser.exist("manualWB"))
    {
        this->manual_wb = true;
        cout << "\033[40;1;37mManual WB ON\033[0m" << endl;
    }

    if (parser.exist("exposure"))
    {
        this->manual_set_exposure = true;
        this->exposure = parser.get<int>("exposure");
    }

    if (parser.exist("gain"))
    {
        this->manual_set_gain = true;
        this->gain = parser.get<int>("gain");
    }

    if (parser.exist("multiCam"))
    {
        if (parser.exist("input"))
        {
            cout << "\033[40;1;35mCan not open camera with --input!  Turning off MultiCam...\033[0m" << endl;
            this->multi_cam = false;
        } else
        {
            cout << "\033[40;1;36mMulti Camera Mode ON\033[0m" << endl;
            this->multi_cam = true;
        }
    }

    if (parser.exist("fullScreen"))
    {
        cout << "\033[40;1;36mFull Screen ON\033[0m" << endl;
        this->full_screen = true;
    }

    if (parser.exist("drawAuxLine"))
    {
        cout << "\033[40;1;34mOperator Auxiliary Lines ON\033[0m" << endl;
        this->draw_aux_line = true;
    }

    if (parser.exist("initState"))
    {
        this->initial_state = parser.get<int>("initState");
        this->manual_state = true;
        cout << "\033[40;1;36mInitial State: " << this->initial_state << "\033[0m" << endl;
    }

    if (parser.exist("manualState"))
    {
        this->initial_state = 0;
        this->manual_state = true;
        cout << "\033[40;1;36mManual State ON\033[0m" << endl;
    }

    if (parser.exist("withYolo"))
    {
        this->with_yolo = true;
    }

    if (parser.exist("manualMaskRect"))
    {
        this->manual_mask_rect = true;
        int s = parser.get<int>("manualMaskRect");
        this->up_rect_ratio = float (s / 10000000) / 100;
        this->down_rect_ratio = float (s % 10000000 / 100000) / 100;
        this->left_rect_ratio  = float (s % 100000 / 1000) / 100;
        this->right_rect_ratio  = float (s % 1000 / 10) / 100;
        this->manual_mask_state = s % 10;
        cout << "\033[40;1;36mManual Mask Rect ON\033[0m" << endl;
    }
}