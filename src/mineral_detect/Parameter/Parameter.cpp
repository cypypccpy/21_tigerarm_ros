//
// Created by cui on 2020/12/9.
//

#include "Parameter.h"
#include <iostream>

/**
 * @brief 从yml文件中读取相机参数
 *
 * @param path 文件路径
 */
CameraParam::CameraParam(const string &path) {
    FileStorage fs(path, FileStorage::READ);
    fs["camera_matrix_1"] >> this->cameraMatrix_6mm;
    fs["distortion_coefficients_1"] >> this->distCoeff_6mm;
    fs["camera_matrix_2"] >> this->cameraMatrix_12mm;
    fs["distortion_coefficients_2"] >> this->distCoeff_12mm;
    fs["camera_matrix_3"] >> this->cameraMatrix_6mm_1;
    fs["distortion_coefficients_3"] >> this->distCoeff_6mm_1;
    fs["camera_matrix_4"] >> this->cameraMatrix_4mm;
    fs["distortion_coefficients_4"] >> this->distCoeff_4mm;
    fs["camera_matrix_5"] >> this->cameraMatrix_4mm_1;
    fs["distortion_coefficients_5"] >> this->distCoeff_4mm_1;
    fs["cam_gain_up"] >> this->cam_gain_up;
    fs["cam_exposure_up"] >> this->cam_exposure_up;
    fs["cam_gain_KB_up"] >> this->cam_gain_KB_up;
    fs["cam_gain_KG_up"] >> this->cam_gain_KG_up;
    fs["cam_gain_KR_up"] >> this->cam_gain_KR_up;
    fs["cam_gain_down"] >> this->cam_gain_down;
    fs["cam_exposure_down"] >> this->cam_exposure_down;
    fs["cam_gain_KB_down"] >> this->cam_gain_KB_down;
    fs["cam_gain_KG_down"] >> this->cam_gain_KG_down;
    fs["cam_gain_KR_down"] >> this->cam_gain_KR_down;
    fs["serial_1"] >> this->serial_1;
    fs["serial_2"] >> this->serial_2;
}

/**
 * @brief 从yml文件中读取矿石识别参数
 *
 * @param path 文件路径
 */
MineralBlockParam::MineralBlockParam(const string &path)
{
    FileStorage fs(path, FileStorage::READ);
    fs["adaptive_thredshold_block_size"] >> this->adaptive_thredshold_block_size;
    fs["adaptive_thredshold_c"] >> this->adaptive_thredshold_c;
    fs["rect_max_width_height_ratio_eng"] >> this->rect_max_width_height_ratio_eng;
    fs["min_contour_area_eng"] >> this->min_contour_area_eng;
    fs["max_contour_area_eng"] >> this->max_contour_area_eng;
    fs["roi_border_ratio_x"] >> this->roi_border_ratio_x;
    fs["roi_border_ratio_y"] >> this->roi_border_ratio_y;
    fs["max_angle"] >> this->max_angle;
    fs["block_ratio"] >> this->block_ratio;
    fs["block_vertical_fault_tolerance_angle"] >> this->block_vertical_fault_tolerance_angle;
    fs["max_size_ratio"] >> this->max_size_ratio;
    fs["max_delta_angle"] >> this->max_delta_angle;
}