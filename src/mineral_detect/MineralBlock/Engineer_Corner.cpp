//
// Created by Carcuis on 2021/1/20.
//
// Modified by Carcuis to engineer on 2021/03/12
//

#include "Engineer_Corner.h"


/**
 * @brief Engineer_Corner类构造函数
 *
 * @param m_corner 筛选后角点旋转矩形
 * @param m_corner_type 筛选后角点对应类型
 */
Engineer_Corner::Engineer_Corner(const RotatedRect &m_corner, const Engineer_CornerType &m_corner_type,
                                 const Engineer_CornerOrientation &m_corner_orientation,
                                 const vector<Point3f> &m_vertices_vect)
{
//    this->corner = m_corner;
    this->corner_type = m_corner_type;
    this->angle = m_corner.angle;
    if (fabs(this->angle) == 90)
    {
        this->size.width = m_corner.size.height;
        this->size.height = m_corner.size.width;
    } else
    {
        this->size = m_corner.size;
    }
    this->center = m_corner.center;
    this->corner_orientation = m_corner_orientation;
    this->vertices_vect = m_vertices_vect;
    //按左上，左下，右上，右下的顺序排序
    sort(vertices_vect.begin(), vertices_vect.end(),
         [](const Point3f &vertex1, const Point3f &vertex2) -> bool {
             return vertex1.z < vertex2.z;
         });

}
