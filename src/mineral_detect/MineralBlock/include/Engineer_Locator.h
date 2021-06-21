//
// Created by Carcuis on 2021/1/13.
//
// Modified by Carcuis to engineer on 2021/03/12
//

#ifndef INC_21_VISION_AERIAL_ENGINEER_LOCATOR_H
#define INC_21_VISION_AERIAL_ENGINEER_LOCATOR_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <numeric>
#include <algorithm>
#include "CmdlineParser.h"
#include "Parameter.h"
#include "Engineer_Corner.h"
#include "Engineer_Block.h"
#include "DataStruct.h"

using namespace std;
using namespace cv;

enum Engineer_Current_State
{
    NONE_STATE              = 0,
    SMALL_ISLAND_AIMING     = 1,
    BIG_ISLAND_AIMING       = 2,
    ROTATING_BLOCK          = 3
};

class Engineer_Locator
{
public:
    Engineer_Locator(Ptr<CmdlineParser> &parser, int frame_width, int frame_height);

    //!----原定位点识别----

    //判断旋转矩形是否重合
    bool isOverlap(RotatedRect &rotatedRect, vector<RotatedRect> &vect);

    //判断旋转矩形是否符合比例
    bool fitRatio(RotatedRect &rotatedRect, double ratio);

    //对重合的旋转矩形更新
    void updateRect(RotatedRect &rotatedRect, size_t position);

    //计算角点顶点坐标
    void calcCornerVertices(const RotatedRect &rect);

    //计算ROI区域二值和
    void calcRoiSum(const RotatedRect & rect, const vector<Point3f> &point_vect, Mat &bin_img);

    //判断角点类型 (残角矩形 / 完整矩形)
    void calcCornerType();

    //绘制角点
    void drawCorners(const RotatedRect &rect, int index, Mat &src_img);

    //存储筛选后角点
//    void preserveCorners(const RotatedRect &rect, int index);

    //!------工程重写和新增-------

    //工程: 识别矿石, 解算矿石方位, 矿石翻转控制, 处理串口发送数据
    void blockLocator(Mat &src_img, int m_current_state);

    //创建掩码图
    bool createMaskImage(Mat &src_img, Mat &dst_img, Engineer_Current_State m_current_state, bool manual_ratio,
                         bool drawEdge);

    //二值化
    void imageToBinary_Engineer(Mat &src_img);

    //寻找轮廓
    void findShapes_Engineer();

    //判断旋转矩形是否越界
    bool overBorder_Engineer(RotatedRect &rotatedRect, int border_x, int border_y, float ratio_x, float ratio_y);

    //计算与y轴偏移角度
    float getAngle(const RotatedRect &rect);

    //匹配角点
    void matchCorners_Engineer(vector<RotatedRect> &vect, Mat &bin_img);

    //计算角点求和
    void calcCornerSum(const RotatedRect &rect, vector<Point3f> &vect, Mat &bin_img);

    //检测矿石
    int detectBlock(vector<Ptr<Engineer_Corner>> &ptrs_vect);

    //计算角点分数
    void calcCornerScore(vector<Ptr<Engineer_Corner>> &m_corner_ptrs_vect);

    //计算角点期望邻角点位置
    vector<Point2f> calcExpectedNearbyPositions(const Ptr<Engineer_Corner> &corner_ptr);

    //计算附近的各种角点个数
    void calcNearbyCornersCount(vector<Ptr<Engineer_Corner>> &m_corner_ptrs_vect, size_t corner_ptr_index,
                                vector<Point2f> expected_nearby_positions, int &three_corner_count,
                                int &complete_corner_count);

    //计算两点距离
    double getDistance(const Point2f &point1, const Point2f &point2);

    //判断一个旋转矩形与另外一个是否符合大小
    bool fitSizeTwo(const RotatedRect &rect1, const RotatedRect &rect2, double ratio);

    //判断一个旋转矩形与另外一个是否符合角度
    bool fitAngleTwo(const RotatedRect &rect1, const RotatedRect &rect2, double m_delta_angle);

    //绘制角点分数数据
    void drawScore(Mat &src_img, const Ptr<Engineer_Corner> &m_corner);

    //判断四边形畸变程度是否符合阈值
    bool fitDistortionCoeff(const Point2f &point1, const Point2f &point2, const Point2f &point3, const Point2f &point4);

    //绘制pnp结算数据
    void drawPnpData(Mat &src_img);

    //绘制安全区边界
    void drawSafetyZoneEdge(Mat &src_img, double up_ratio, double down_ratio, double left_ratio, double right_ratio);

    //绘制操作手辅助框
    void drawAuxiliaryBox(Mat &src_img);

    //处理串口发送数据
    int getDataSign(int previous_data_sign, int m_block_surface, Mat &src_img, bool stop_at_every_surface);
    int getDataSurface(int m_block_surface, Mat &src_img);

    //处理发送串口数据
    void process_data_send(bool is_found_m);

    //打印输出接收串口数据
    void print_data_get() const;

    //打印输出发送串口数据
    void print_data_send() const;

    //判断旋转复位状态
    bool get_rotate_reset_sign();

    //!----------------------

    //!----原定位点识别----

    Mat color_img;
    int img_cols;
    int img_rows;

    //当前动作状态
    Engineer_Current_State current_state = NONE_STATE;
    bool manual_state = false;

    //mask
    Mat mask_img;
    bool manual_mask_rect = false;
    float up_rect_ratio = 0.0;
    float down_rect_ratio = 0.0;
    float left_rect_ratio = 0.0;
    float right_rect_ratio = 0.0;
    int manual_mask_state = 0;

    //帧数
    uint8_t frame_count = 0;

    //二值化图像
    Mat Gray_img;
    Mat Bin_img;
//    Mat Canny_img;
    Mat Edge_img;

    //Canny阈值
//    int canny_threshold1;
//    int canny_threshold1_min = 1;
//    int canny_threshold1_max = 2000;
//    int canny_threshold2;
//    int canny_threshold2_min = 1;
//    int canny_threshold2_max = 2000;

    //轮廓容器
    vector<vector<Point>> contours; //原始findContours检测结果
    vector<vector<Point>> contours_first_checkout; //第一次筛除结果
    vector<RotatedRect> contours_vect; //第三次筛除结果

    //待更新的重合旋转矩形在容器中的索引值
    size_t overlapping_rect_index;

    //求和核
    double sum_core_size; //正矩形求和核大小
//    double sum_core_radius; //圆形求和核半径
    Rect sum_core_rect; //求和核ROI矩阵
    Mat sum_core_mat; //待求和ROI区域二值图

    //求和结果
    double sum_core_result;
    vector<double> sum_core_result_vect; //求和核结果容器
//    vector<double> sum_corner_result_vect; //求和角点结果容器
//    double sum_result_variance; //求和结果方差

    //求和安全区顶点
//    Point2f safe_zone_l_t; //左上
//    Point2f safe_zone_l_b; //左下
//    Point2f safe_zone_r_t; //右上
//    Point2f safe_zone_r_b; //右下

    //角点顶点
    vector<Point3f> corner_vertices_vect; //包含角点坐标x,y和位置z的容器

    //角点类型
    vector<Engineer_CornerType> corner_types_vect;

    //角点朝向
    vector<Engineer_CornerOrientation> corner_orientations_vect;

    //筛除结果
//    vector<RotatedRect> found_corners_vect; //最终筛除角点容器
//    vector<CornerType> found_corner_types_vect; //对应角点种类

    //存放指向Corner类智能指针的容器
    vector<Ptr<Engineer_Corner>> corner_ptrs_vect;

    //存放其他类型Corner的智能指针容器
//    vector<Ptr<Engineer_Corner>> OtherTypeCorner_ptrs_vect;

    //!----工程新增----

    //坐标解算结果
    Point3f camera_coordinate; //x->towards_right, y->towards_down, z->towards_forward, origin->block_center
    Point3f mineral_block_coordinate;
    Scalar euler_angles; //yaw-pitch-row

    //PnP数据
    Mat rvec, tvec;

//    bool eng_found_block = false;
    int block_surface;

    uint8_t data_sign = 1;
    uint8_t data_surface = 0;

    //串口数据结构
    vector<DataStruct_Get> data_struct_get;
    DataStruct_Send data_struct_send;

    bool block_is_found = false;

    bool draw_aux_lines = false;
};

#endif //INC_21_VISION_AERIAL_ENGINEER_LOCATOR_H
