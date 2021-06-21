//
// Created by Carcuis on 2021/1/20.
//
// Modified by Carcuis to engineer on 2021/03/12
//

#ifndef INC_21_VISION_AERIAL_ENGINEER_LOCATIONBLOCK_H
#define INC_21_VISION_AERIAL_ENGINEER_LOCATIONBLOCK_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <algorithm>
#include "Parameter.h"

using namespace cv::ml;

enum Engineer_CornerVertexPos
{
    TOP_LEFT        = 1,
    BOTTOM_LEFT     = 2,
    TOP_RIGHT       = 3,
    BOTTOM_RIGHT    = 4,
    CENTER          = 5
};

enum Engineer_CornerType
{
    THREE_CORNER_RECT   =1,
    COMPLETE_RECT       =2,
    OTHERS              =3
};

enum Engineer_CornerOrientation
{
    LEFT_UP         = 1,
    LEFT_DOWN       = 2,
    RIGHT_UP        = 3,
    RIGHT_DOWN      = 4,
    NONE            = 5 //for COMPLETE_RECT
};

class Engineer_Corner : public RotatedRect
{
public:
    //Engineer_Corner类构造函数
    Engineer_Corner(const RotatedRect &m_corner, const Engineer_CornerType &m_corner_type,
                    const Engineer_CornerOrientation &m_corner_orientation,
                    const vector<Point3f> &m_vertices_vect);

    //角点继承RotatedRect类
//    RotatedRect corner;

    //角点属性
    Engineer_CornerType corner_type;
    Engineer_CornerOrientation corner_orientation;

    vector<Point3f> vertices_vect; //左上左下右上右下

    //分数
    double score = 0;

};


#endif //INC_21_VISION_AERIAL_ENGINEER_LOCATIONBLOCK_H
