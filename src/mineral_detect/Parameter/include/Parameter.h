//
// Created by cui on 2020/12/9.
//

#ifndef INC_21_VISION_AERIAL_PARAMETER_H
#define INC_21_VISION_AERIAL_PARAMETER_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct CameraParam
{
    explicit CameraParam(const string &path);

    Mat cameraMatrix_6mm;
    Mat distCoeff_6mm;
    Mat cameraMatrix_12mm;
    Mat distCoeff_12mm;
    Mat cameraMatrix_6mm_1;
    Mat distCoeff_6mm_1;
    Mat cameraMatrix_4mm;
    Mat distCoeff_4mm;
    Mat cameraMatrix_4mm_1;
    Mat distCoeff_4mm_1;

    string serial_1;
    string serial_2;

    double cam_exposure_up;
    double cam_gain_up;
    double cam_gain_KB_up;
    double cam_gain_KG_up;
    double cam_gain_KR_up;

    double cam_exposure_down;
    double cam_gain_down;
    double cam_gain_KB_down;
    double cam_gain_KG_down;
    double cam_gain_KR_down;
} static camera_param("Settings/CameraParam.yml");
//using camera_param_ptr = shared_ptr<CameraParam>;

struct MineralBlockParam
{
    explicit MineralBlockParam(const string &path);
    int adaptive_thredshold_block_size;
    double adaptive_thredshold_c;
    double rect_max_width_height_ratio_eng;
    double min_contour_area_eng;
    double max_contour_area_eng;
    float roi_border_ratio_x;
    float roi_border_ratio_y;
    float max_angle;
    float block_ratio;
    float block_vertical_fault_tolerance_angle;
    double max_size_ratio;
    double max_delta_angle;
} static mineral_block_param("Settings/MineralBlockParam.yml");

#endif //INC_21_VISION_AERIAL_PARAMETER_H
