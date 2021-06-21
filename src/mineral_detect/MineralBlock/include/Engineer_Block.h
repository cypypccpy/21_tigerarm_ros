//
// Created by cui on 2021/4/23.
//

#ifndef INC_21_VISION_MINERALBLOCK_ENGINEER_BLOCK_H
#define INC_21_VISION_MINERALBLOCK_ENGINEER_BLOCK_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include "Engineer_Corner.h"
#include "Parameter.h"

using namespace std;
using namespace cv;

enum Two_Corners_Position
{
    UP      = 1,
    DOWN    = 2,
    LEFT    = 3,
    RIGHT   = 4
};

class Engineer_Block
{
public:

    explicit Engineer_Block(vector<Ptr<Engineer_Corner>> & corner_ptrs_vect);

    explicit Engineer_Block(vector<Ptr<Engineer_Corner>> &corner_ptrs_vect, const Point2f &R_point1,
                            const Point2f &R_point2, bool m_found_R_1, bool m_found_R_2);

    explicit Engineer_Block(vector<Ptr<Engineer_Corner>> &corner_ptrs_vect, Two_Corners_Position &position);

    void calcEularAngles();

    void calcCameraCoordinate();

    void calcMineralBlockCoordinate(bool use_up_cor = false);

public:
    Mat rvec, tvec;     //SolvePnP原始结果
    Mat Rvec, Tvec;     //转换为CV_32F结果
    Mat Rmat;           //罗德里格斯变换结果

    double angle_yaw, angle_pitch, angle_row; //相机相对于矿石的欧拉角

    Point3f camera_coordinate; //相机世界坐标 -- 相对于"固定"世界位置的矿石 (世界坐标系)
    Point3f mineral_block_coordinate; //矿石在相机坐标系中的坐标

private:
    const vector<Point3f> block_four_corners{{-60, -60, 0}, //左上
                                             {-60, 60,  0},  //左下
                                             {60,  -60, 0},  //右上
                                             {60,  60,  0}};  //右下
    const vector<Point3f> block_two_corners_up{{-75, -75, 0},  //左上
                                               {-75, -45, 0},  //左下
                                               {75, -75, 0},  //右上
                                               {75, -45, 0}};  //右下
    const vector<Point3f> block_two_corners_left{{-75, -75, 0}, //左上
                                                 {-75, 75,  0},  //左下
                                                 {-45,  -75, 0},  //右上
                                                 {-45,  75,  0}};  //右下
    const vector<Point3f> block_two_corners_right{{45, -75, 0}, //左上
                                                  {45, 75,  0},  //左下
                                                  {75,  -75, 0},  //右上
                                                  {75,  75,  0}};  //右下
    const vector<Point3f> block_two_corners_down{{-75, 45, 0}, //左上
                                                 {-75, 75,  0},  //左下
                                                 {75,  45, 0},  //右上
                                                 {75,  75,  0}};  //右下
    const vector<Point3f> block_two_corners_up_six{{-60, -60, 0}, //左中心
                                                   {60, -60,  0},  //右中心
                                                   {-75, -75, 0},  //左上
                                                   {-75, -45, 0},  //左下
                                                   {75, -75, 0},  //右上
                                                   {75, -45, 0}};  //右下
    //FIXME 以下使用R点上的角点进行pnp解算效果不佳，可能需要更精准的坐标数据
    const vector<Point3f> block_two_corners_up_with_R_point1{{-60, -60, 0}, //左中心
                                                             {60, -60,  0},  //右中心
                                                             {-39.226, -42.336, 0}};  //R点1
    const vector<Point3f> block_two_corners_up_with_R_point2{{-60, -60, 0}, //左中心
                                                             {60, -60,  0},  //右中心
                                                             {-4.522, -26.195, 0}};  //R点2
    const vector<Point3f> block_two_corners_up_with_R_point1_2{{-60, -0, 0}, //左中心
                                                               {60, -0,  0},  //右中心
                                                               {-39.226, -42.336, 0},  //R点1
                                                               {-4.522, -26.195, 0}};  //R点2
};


#endif //INC_21_VISION_MINERALBLOCK_ENGINEER_BLOCK_H
