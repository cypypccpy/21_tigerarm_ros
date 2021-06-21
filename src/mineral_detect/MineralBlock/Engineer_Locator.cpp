//
// Created by Carcuis on 2021/1/16.
//
// Modified by Carcuis to engineer on 2021/03/12
//

#include "Engineer_Locator.h"

Engineer_Locator::Engineer_Locator(Ptr<CmdlineParser> &parser, int frame_width, int frame_height)
{
    this->draw_aux_lines = parser->draw_aux_line;
    this->manual_state = parser->manual_state;
    if (parser->manual_mask_rect)
    {
        this->manual_mask_rect = true;
        this->up_rect_ratio = parser->up_rect_ratio;
        this->down_rect_ratio = parser->down_rect_ratio;
        this->left_rect_ratio = parser->left_rect_ratio;
        this->right_rect_ratio = parser->right_rect_ratio;
        this->manual_mask_state = parser->manual_mask_state;
    }

    this->img_cols = frame_width;
    this->img_rows = frame_height;
//    this->Bin_img = Mat::zeros(Size(img_cols, img_rows), CV_8UC1);
    this->Edge_img = Mat::zeros(Size(img_cols, img_rows), CV_8UC1);
}

/**
 * @brief 工程: 识别矿石, 解算矿石方位, 矿石翻转控制, 处理串口发送数据
 *
 * @param src_img 输入图像
 */
void Engineer_Locator::blockLocator(Mat &src_img, int m_current_state)
{
    this->print_data_get();

    //获取当前动作状态
    if (!this->data_struct_get.empty() && !this->manual_state)
    {
        this->current_state = static_cast<Engineer_Current_State>(data_struct_get.back().current_status);
    } else if (this->manual_state)
    {
        this->current_state = static_cast<Engineer_Current_State>(m_current_state);
    }

    if (src_img.cols != this->img_cols) this->img_cols = src_img.cols;
    if (src_img.rows != this->img_rows) this->img_rows = src_img.rows;

    this->block_is_found = false;
    this->color_img = src_img;
    this->mask_img = 0;
    this->Edge_img = 0;
    this->frame_count++;
//    cout << "\n\033[34m\033[01mFRAME COUNT:\033[0m" << (int) frame_count << endl;

    //清空容器
    this->contours.clear();
    if (frame_count % 3 == 0)  //持续采集三帧数据
    {
        this->contours_first_checkout.clear();
        this->contours_vect.clear();
//        cout << "\033[32mCLEARED CONTAINERS!!\033[0m" << endl;
    }

    //创建掩码图
    bool with_mask = this->createMaskImage(src_img, mask_img, this->current_state, this->manual_mask_rect, true);

    //二值化
    this->imageToBinary_Engineer(with_mask ? mask_img : src_img);
    //查找轮廓
    this->findShapes_Engineer();

    //匹配角点
    this->matchCorners_Engineer(contours_vect, this->Bin_img);
    //进行判断
    this->block_surface = this->detectBlock(corner_ptrs_vect);

    //绘制pnp结算数据
    this->drawPnpData(src_img);
    //绘制操作手辅助框
    if (this->draw_aux_lines)   this->drawAuxiliaryBox(src_img);

    //处理发送数据
    if (this->get_rotate_reset_sign())
        this->data_sign = 1;
    if (current_state == ROTATING_BLOCK)
        this->data_sign = this->getDataSign(this->data_sign, this->block_surface, src_img, false);
    this->data_surface = this->getDataSurface(this->block_surface, src_img);

    this->process_data_send(this->block_is_found);
    this->print_data_send();

    //清空容器
//    this->found_corners_vect.clear();
//    this->found_corner_types_vect.clear();
    this->corner_types_vect.clear();
    this->corner_orientations_vect.clear();
    this->corner_ptrs_vect.clear();
//    this->OtherTypeCorner_ptrs_vect.clear();
}

/**
 * @brief 创建掩码图
 *
 * @param src_img 源图像
 * @param dst_img 输出掩码图
 * @param m_current_state 当前动作状态值
 * @param manual_ratio 是否为手动大小模式
 * @param drawEdge 是否绘制边界
 *
 * @return bool 掩码图创建成功
 */
bool Engineer_Locator::createMaskImage(Mat &src_img, Mat &dst_img, Engineer_Current_State m_current_state,
                                       bool manual_ratio, bool drawEdge)
{
    double up_ratio, down_ratio, left_ratio, right_ratio;

    switch (m_current_state)
    {
        case NONE_STATE:
            up_ratio = 0, down_ratio = 1, left_ratio = 0, right_ratio = 1;
            return false;
        case ROTATING_BLOCK:
            up_ratio = 0.08, down_ratio = 0.52, left_ratio = 0.35, right_ratio = 0.65;
            break;
        case SMALL_ISLAND_AIMING:
            up_ratio = 0.34, down_ratio = 0.5, left_ratio = 0.2, right_ratio = 0.8;
            break;
        case BIG_ISLAND_AIMING:
            up_ratio = 0.45, down_ratio = 0.53, left_ratio = 0.3, right_ratio = 0.7;
            break;
        default:
            break;
    }
    if (manual_ratio && this->manual_mask_state == m_current_state)
    {
        up_ratio = this->up_rect_ratio;
        down_ratio = this->down_rect_ratio;
        left_ratio = this->left_rect_ratio;
        right_ratio = this->right_rect_ratio;
    }

    Mat mask_mat = Mat::zeros(img_rows, img_cols, CV_8UC1);

    bool illegal = (up_ratio < down_ratio) && (left_ratio < right_ratio) && (down_ratio <= 1 && right_ratio <= 1)
            && (up_ratio >= 0 && left_ratio >= 0);
    if (!illegal)
    {
        cout << "Mask ratio error!" << endl;
        return false;
    }

    //绘制安全区边界
    if (drawEdge)
    {
        this->drawSafetyZoneEdge(src_img, up_ratio, down_ratio, left_ratio, right_ratio);
    }

    Rect mask_rect(left_ratio * img_cols, up_ratio * img_rows, (right_ratio - left_ratio) * img_cols, (down_ratio - up_ratio) * img_rows);
    mask_mat(mask_rect).setTo(255);
    src_img.copyTo(dst_img, mask_mat);
    return true;
}

/**
 * @brief 工程：二值化处理图像
 *
 * @param src_img 待处理的图像
 */
void Engineer_Locator::imageToBinary_Engineer(Mat &src_img)
{
    //!----灰度-阈值法处理二值化----
    cvtColor(src_img, Gray_img, COLOR_BGR2GRAY);

    Mat temp_img;
    double thresh = threshold(Gray_img, temp_img, 0, 255, THRESH_OTSU|THRESH_BINARY_INV);
    threshold(Gray_img, Bin_img, thresh - 5, 255, THRESH_BINARY_INV);

//    adaptiveThreshold(Gray_img, Bin_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,
//                      mineral_block_param.adaptive_thredshold_block_size, mineral_block_param.adaptive_thredshold_c);

    //!----通道相减并行处理二值化----
//    Mat bin_img = Mat::zeros(Size(src_img.cols, src_img.rows), CV_8UC1);
//    const size_t rows = src_img.rows;
//    parallel_for_(Range(0, rows), ParallelBGR2Binary(src_img, bin_img));
//    bin_img.copyTo(this->Bin_img);

    //!----二值化平滑处理----
//    dilate(Bin_img, Bin_img, getStructuringElement(MORPH_RECT, Size(7, 7)));
//    GaussianBlur(Bin_img, Bin_img, Size(3, 3), 3, 3);
//    medianBlur(Bin_img, Bin_img, 5);
}


/**
 * @brief 工程：寻找匹配轮廓
 */
void Engineer_Locator::findShapes_Engineer()
{
    findContours(Bin_img, contours, RETR_LIST, CHAIN_APPROX_NONE);
    drawContours(Edge_img, contours, -1, 255);
    cout << "\033[01mInitial Contours.size: " << contours.size() << "\033[0m" << endl;
    if (contours.empty()) { return; }

    //第一次筛选，排除过小和过大面积轮廓 contours --> contours_first_checkout
    for (const auto& contour : contours)
    {
        if (contour.size() > mineral_block_param.min_contour_area_eng * img_cols / 640 &&
            contour.size() < mineral_block_param.max_contour_area_eng * img_cols / 640)
        {
            contours_first_checkout.push_back(contour);
//            putText(src_img, to_string(contour.size()), minAreaRect(contour).center, 0, 0.5, Scalar(0, 255, 0));
        } else
        {
//            if (contour.size() > 300)
//                putText(src_img, to_string(contour.size()), minAreaRect(contour).center, 0, 0.5, Scalar(0, 0, 255));
//            if (contour.size() < 100)
//                putText(src_img, to_string(contour.size()), minAreaRect(contour).center, 0, 0.5, Scalar(255, 0, 0));
            continue;
        }
    }
    cout << "\033[01m<First Checkout> contours.size: " << contours_first_checkout.size() << "\033[0m" << endl;
    if (contours_first_checkout.empty()) { return; }

    //旋转矩形拟合存储角点方案
//    cout << "\033[01m[last frame] contours_vect.size:" << contours_vect.size() << "\033[0m" << endl;
    for (const auto& contour : contours_first_checkout)
    {
        RotatedRect contour_rect = minAreaRect(contour);
        if (fitRatio(contour_rect, mineral_block_param.rect_max_width_height_ratio_eng)) //第二次筛选 判断符合比例
        {
            if (isOverlap(contour_rect, contours_vect)) //判断是否重合
            {
                //对重合的旋转矩形轮廓更新
                updateRect(contour_rect, overlapping_rect_index);
                continue;
            }
            if (overBorder_Engineer(contour_rect, img_cols, img_rows,
                                    mineral_block_param.roi_border_ratio_x, mineral_block_param.roi_border_ratio_y))
            {
                //第三次筛选，判断是否越界
                continue;
            }
            if (this->getAngle(contour_rect) > mineral_block_param.max_angle)
            {
                //第四次删选，判断角度是否偏移过大
//                char str[20];
//                sprintf(str, "%.2f", getAngle(contour_rect));
//                putText(src_img, (string)str, contour_rect.center, 0, 0.5, Scalar(0, 255, 255), 2);
                continue;
            } else
            {
//                char str[20];
//                sprintf(str, "%.2f", getAngle(contour_rect));
//                sprintf(str, "%.2f", contour_rect.angle);
//                putText(src_img, (string)str, contour_rect.center, 0, 0.5, Scalar(0, 255, 255), 1);
            }
            contours_vect.push_back(contour_rect);
//            cout << "\033[01;36m contour.size() = " << contour.size() << "\033[0m" << endl;
//            cout << "\033[01;36m contour_rect.angle() = " << contour_rect.angle << "\033[0m" << endl;
        }
    }
//    cout << "\033[01m[curt frame] contours_vect.size:" << contours_vect.size() << "\033[0m" << endl;
    if (contours_vect.empty()) { return; }

    //将筛选后轮廓从左到右排序
    sort(contours_vect.begin(), contours_vect.end(),
         [](const RotatedRect &rect1, const RotatedRect &rect2) -> bool {
          return rect1.center.x < rect2.center.x;
    });
}

/**
 * @brief 计算与y轴偏移角度
 *
 * @param rect 待计算旋转矩形
 */
float Engineer_Locator::getAngle(const RotatedRect &rect)
{
    float angle = fabs(rect.angle);
    if (angle < 45)
    {
        return angle;
    } else
    {
        return (90 - angle);
    }
}

/**
 * @brief 判断旋转矩形是否符合比例
 *
 * @param rotatedRect 待判断旋转矩形
 *
 * @return bool: true / false
 */
bool Engineer_Locator::fitRatio(RotatedRect &rotatedRect, double ratio)
{
    bool fitRatio = (MAX(rotatedRect.size.width, rotatedRect.size.height) <
                     (MIN(rotatedRect.size.width, rotatedRect.size.height) * ratio));
    return fitRatio;
}

/**
 * @brief 判断旋转矩形是否重合
 *
 * @param rotatedRect 待判断旋转矩形
 * @param vec 参与对比的旋转矩形容器
 *
 * @return bool: true / false
 */
bool Engineer_Locator::isOverlap(RotatedRect &rotatedRect, vector<RotatedRect> &vect)
{
    bool isOverlap = false;

    Point2f new_left;
    Point2f new_right;

    Point2f left;
    Point2f right;

    new_left.x = rotatedRect.center.x - rotatedRect.size.width / 2;
    new_left.y = rotatedRect.center.y - rotatedRect.size.height / 2;
    new_right.x = rotatedRect.center.x + rotatedRect.size.width / 2;
    new_right.y = rotatedRect.center.y + rotatedRect.size.height / 2;

    for (size_t i = 0; i < vect.size(); i++)
    {
        left.x = vect[i].center.x - vect[i].size.width / 2;
        left.y = vect[i].center.y - vect[i].size.height / 2;
        right.x = vect[i].center.x + vect[i].size.width / 2;
        right.y = vect[i].center.y + vect[i].size.height / 2;

        isOverlap =  isOverlap || (
                new_left.x < right.x && new_left.y < right.y &&
                new_right.x > left.x && new_right.y > left.y
        );
        if (isOverlap)
        {
            overlapping_rect_index = i;
            break;
        }
    }

    return isOverlap;
}

/**
 * @brief 工程：判断旋转矩形是否越界
 *
 * @param rotatedRect 待判断旋转矩形
 * @param x Mat 边界 cols
 * @param y Mat 边界 rows
 *
 * @return bool: true / false
 */
bool Engineer_Locator::overBorder_Engineer(RotatedRect &rotatedRect, int border_x, int border_y, float ratio_x,
                                           float ratio_y)
{
    Point2f vertices[4];
    rotatedRect.points(vertices);
    bool overBorder = false;
    for (const auto &vertex : vertices)
    {
        overBorder = overBorder || (vertex.x < ((float)border_x * (1 - ratio_x)) || vertex.y < ((float)border_y * (1 - ratio_y)) ||
                                    vertex.x > ((float)border_x * ratio_x) || vertex.y > ((float)border_y * ratio_y));
//        overBorder = overBorder || (vertex.x < ((float)border_x * 0.4) || vertex.y < ((float)border_y * (1 - ratio_y)) ||
//                vertex.x > ((float)border_x) || vertex.y > ((float)border_y * ratio_y));
    }
    return overBorder;
}

/**
 * @brief 对重合的旋转矩形更新
 *
 * @param rotatedRect 新的旋转矩形
 * @param position 待更新的重合旋转矩形在容器中的索引值
 */
void Engineer_Locator::updateRect(RotatedRect &rotatedRect, size_t position)
{
    contours_vect[position] = rotatedRect;
}

/**
 * @brief 工程：匹配角点
 *
 * @param vect 待匹配旋转矩形容器
 * @param bin_img 所需二值化图像
 */
void Engineer_Locator::matchCorners_Engineer(vector<RotatedRect> &vect, Mat &bin_img)
{
    int index = 0;
    for (const auto &rect : vect)
    {
        //计算四顶点坐标
        calcCornerVertices(rect);
        //计算四顶点和中心点邻域二值和
        calcRoiSum(rect, corner_vertices_vect, Bin_img);
        //判断顶点类型 (左上 左下 右上: 残角矩形，右下: 完整矩形)
        calcCornerType();
        //计算角点求和
//        if (corner_types_vect[index]!=3) calcCornerSum(rect, corner_vertices_vect, Bin_img);
        //绘制角点
        drawCorners(rect, index, color_img);
        //存储角点
//        preserveCorners(rect, index);

        //创建角点对象
        if(corner_types_vect[index] != 3) //分离OTHERS类型角点
        {
            this->corner_ptrs_vect.push_back((Ptr<Engineer_Corner>) new Engineer_Corner(rect, corner_types_vect[index],
                                                                                        corner_orientations_vect[index],
                                                                                        corner_vertices_vect));
        } else
        {
//            this->OtherTypeCorner_ptrs_vect.push_back((Ptr<Engineer_Corner>)new Engineer_Corner(rect, corner_types_vect[index]));
        }

        index++;
        this->sum_core_result_vect.clear();
        this->corner_vertices_vect.clear();
    }
    cout << "\033[01m[FINAL] corner_ptrs_vect.size:" << corner_ptrs_vect.size() << "\033[0m" << endl;

}

/**
 * @brief 计算角点四顶点 坐标 并区分位置（左上 右上 左下 右下）
 *
 * @param rect 待计算角点
 */
void Engineer_Locator::calcCornerVertices(const RotatedRect &rect)
{
//    Rect temp_rect = rect.boundingRect();
//    if (temp_rect.tl().x > 0 && temp_rect.tl().y > 0 && temp_rect.br().x < img_cols && temp_rect.br().y < img_rows)
//    {
//        Mat temp_mat = Edge_img(temp_rect);
//        vector<Vec4f> lines;
//        HoughLinesP(temp_mat, lines, 1, CV_PI/180, 10, 5, 5);
////    HoughLinesP(Edge_img, lines, 1, CV_PI/180, 10, 30, 5);
//    for (size_t i = 0; i < lines.size(); i++) {
//            Vec4f hline = lines[i]; //数组获取直线
//            line(color_img, Point(hline[0], hline[1]) + temp_rect.tl(), Point(hline[2], hline[3]) + temp_rect.tl(), Scalar(0, 0, 255), 2);
////            line(color_img, Point(hline[0], hline[1]), Point(hline[2], hline[3]), Scalar(0, 0, 255), 2);
//        }
//    }

    Engineer_CornerVertexPos corner_vertex_pos; //角点顶点位置
    Point2f rect_vertices[4];
    rect.points(rect_vertices);
    for (uint32_t i = 0; i < 4; i++)
    {
        if (rect_vertices[i].x < rect.center.x)
        {
            if (rect_vertices[i].y < rect.center.y)
            {
                if (rect_vertices[(i + 1) % 4].x < rect.center.x &&
                    rect_vertices[(i + 1) % 4].y < rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 1) % 4].x) ? TOP_LEFT : TOP_RIGHT;
                else if (rect_vertices[(i + 3) % 4].x < rect.center.x &&
                         rect_vertices[(i + 3) % 4].y < rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 3) % 4].x) ? TOP_LEFT : TOP_RIGHT;
                else
                    corner_vertex_pos = TOP_LEFT;
            } else
            {
                if (rect_vertices[(i + 1) % 4].x < rect.center.x &&
                    rect_vertices[(i + 1) % 4].y > rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 1) % 4].x) ? BOTTOM_LEFT : BOTTOM_RIGHT;
                else if (rect_vertices[(i + 3) % 4].x < rect.center.x &&
                         rect_vertices[(i + 3) % 4].y > rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 3) % 4].x) ? BOTTOM_LEFT : BOTTOM_RIGHT;
                else
                    corner_vertex_pos = BOTTOM_LEFT;
            }
        } else
        {
            if (rect_vertices[i].y < rect.center.y)
            {
                if (rect_vertices[(i + 1) % 4].x > rect.center.x &&
                    rect_vertices[(i + 1) % 4].y < rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 1) % 4].x) ? TOP_LEFT : TOP_RIGHT;
                else if (rect_vertices[(i + 3) % 4].x > rect.center.x &&
                         rect_vertices[(i + 3) % 4].y < rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 3) % 4].x) ? TOP_LEFT : TOP_RIGHT;
                else
                    corner_vertex_pos = TOP_RIGHT;
            } else
            {
                if (rect_vertices[(i + 1) % 4].x > rect.center.x &&
                    rect_vertices[(i + 1) % 4].y > rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 1) % 4].x) ? BOTTOM_LEFT : BOTTOM_RIGHT;
                else if (rect_vertices[(i + 3) % 4].x > rect.center.x &&
                         rect_vertices[(i + 3) % 4].y > rect.center.y)
                    corner_vertex_pos = (rect_vertices[i].x < rect_vertices[(i + 3) % 4].x) ? BOTTOM_LEFT : BOTTOM_RIGHT;
                else
                    corner_vertex_pos = BOTTOM_RIGHT;
            }
        }
        Point3f corner_vertex; //角点顶点
        corner_vertex.x = rect_vertices[i].x;
        corner_vertex.y = rect_vertices[i].y;
        corner_vertex.z = corner_vertex_pos;
        corner_vertices_vect.push_back(corner_vertex);
    }
}

/**
 * @brief 计算角点某ROI区域二值和 - 旋转矩形五点邻域
 *
 * @param rect 待计算角点
 * @param point_vect 角点四顶点<Point3f>容器
 * @param bin_img 所需二值化图像
 */
void Engineer_Locator::calcRoiSum(const RotatedRect & rect, const vector<Point3f> &point_vect, Mat &bin_img)
{
    //提前判断求和核越界 - 暂弃用
//    for (const auto & point : point_vect)
//    {
//        sum_core_size = MIN(point.x, rect.size.width) / 3;
//        safe_zone_l_t.x = point.x - sum_core_size / 2;
//        safe_zone_l_t.y = point.y - sum_core_size / 2;
//          ....
//        if (sum_core_rect.x < 0 || sum_core_rect.y < 0 ||
//            sum_core_rect.x + sum_core_rect.width > img_cols ||
//            sum_core_rect.y + sum_core_rect.height > img_rows)
//        {
//            cout << "求和核越界!!!" << endl;
//            return;
//        }
//    }
    for (const auto & point : point_vect)
    {
//        cout << "\n";
        int pos = point.z;

        switch (pos)
        {
            case TOP_LEFT:
                sum_core_size = MIN(rect.size.height, rect.size.width) / 4;
                sum_core_rect.x = point.x;
                sum_core_rect.y = point.y;
//                cout << "(TOP_L)";
                break;
            case BOTTOM_LEFT:
                sum_core_size = MIN(rect.size.height, rect.size.width) / 4;
                sum_core_rect.x = point.x;
                sum_core_rect.y = point.y - sum_core_size;
//                cout << "(BOT_L)";
                break;
            case TOP_RIGHT:
                sum_core_size = MIN(rect.size.height, rect.size.width) / 4;
                sum_core_rect.x = point.x - sum_core_size;
                sum_core_rect.y = point.y;
//                cout << "(TOP_R)";
                break;
            case BOTTOM_RIGHT:
                sum_core_size = MIN(rect.size.height, rect.size.width) / 4;
                sum_core_rect.x = point.x - sum_core_size;
                sum_core_rect.y = point.y - sum_core_size;
//                cout << "(BOT_R)";
                break;
            default:
                break;
        }

        sum_core_rect.width = sum_core_size;
        sum_core_rect.height = sum_core_rect.width;

        if (sum_core_rect.x < 0 || sum_core_rect.y < 0 ||
            sum_core_rect.x + sum_core_rect.width > img_cols ||
            sum_core_rect.y + sum_core_rect.height > img_rows)
        {
//            cout << "求和核ROI越界!!!" << endl;
            return;
        }

//        cout << "sum_core_size : " << sum_core_size << "\t";
        sum_core_mat = bin_img(sum_core_rect);
        sum_core_result = sum(sum_core_mat)[0];
//        cout  << sum_core_result << "\t";
        sum_core_result_vect.push_back(sum_core_result);
    }
//    cout << "\n";
    //CENTER
    sum_core_size = MIN(rect.size.height, rect.size.width) / 12;
//    cout << "(CENTR)sum_core_size : " << sum_core_size << "\t";
    sum_core_rect.x = rect.center.x - sum_core_size / 2;
    sum_core_rect.y = rect.center.y - sum_core_size / 2;
    sum_core_rect.width = sum_core_size;
    sum_core_rect.height = sum_core_rect.width;
    sum_core_mat = bin_img(sum_core_rect);
    sum_core_result = sum(sum_core_mat)[0];
//    cout  << sum_core_result << endl;
    sum_core_result_vect.push_back(sum_core_result);
}

/**
 * @brief 判断角点类型（残角矩形，完整矩形, 其他）
 */
void Engineer_Locator::calcCornerType()
{
    Engineer_CornerType corner_type;
    int zero_count = 0; //零值个数
    int zero_index;
    for (int i = 0; i < 5; i++)
    {
        //判断残角个数
        if (sum_core_result_vect[i] < 250) //容错值
        {
            zero_count++;
            if (i != 4)
                zero_index = i;
        }
    }

    //计算非零值方差 -暂弃用
//    double sum = accumulate(begin(sum_core_result_vect), end(sum_core_result_vect), 0.0);
//    double mean =  sum / sum_core_result_vect.size(); //均值
//    double accum = 0;
////    for_each (begin(sum_core_result_vect), end(sum_core_result_vect),
////                   [&](const double d) { accum  += (d - mean) * (d - mean); });
//    for (uint16_t i = 0; i < 5 && i != zero_index; i++)
//        accum  += (i - mean) * (i - mean);
//    sum_result_variance = sqrt( accum / (double) (sum_core_result_vect.size() - zero_count));
//    cout << "Variance: " <<  sum_result_variance << endl;

    if (zero_count == 0) //完整矩形
    {
        corner_type = COMPLETE_RECT;
//        cout <<  "\033[32m" << "COMPLETE_RECT!!!" << "\033[0m" << endl;
    }
//    else if (sum_core_result_vect[4] < 250 && zero_count == 2) //残角矩形
    else if (zero_count == 2) //残角矩形
    {
        corner_type = THREE_CORNER_RECT;
//        cout << "\033[36m" << "THREE_CORNER_RECT" << "\033[0m" << endl;
    }
    else //其他
    {
        corner_type = OTHERS;
//        cout << "\033[34m" << "OTHERS" << "\033[0m" << endl;
    }
    corner_types_vect.push_back(corner_type);
    corner_orientations_vect.push_back(zero_count == 2 ?
            static_cast<Engineer_CornerOrientation>((int)corner_vertices_vect[zero_index].z) : NONE);
//    corner_orientations_vect.push_back(LEFT_UP);
//    if (zero_count == 2)
//    {
//        putText(color_img, to_string(int(static_cast<Engineer_CornerOrientation>((int)corner_vertices_vect[zero_index].z))),
//                Point2f(corner_vertices_vect[zero_index].x, corner_vertices_vect[zero_index].y), 0, 1, Scalar(0, 0, 255));
//        cout << "enum: " << corner_vertices_vect[zero_index].z << endl;
//    }
}

/**
 * @brief 计算角点透射变换后二值和
 *
 * @param rect
 * @param bin_img
 */
void Engineer_Locator::calcCornerSum(const RotatedRect &rect, vector<Point3f> &vect, Mat &bin_img)
{
    vector<Point2f> vect_src;

    for (size_t i_1 = 0; i_1 < vect.size(); i_1++)
    {
        if (vect[i_1].z == TOP_LEFT)
        {
            vect_src.emplace_back(vect[i_1].x, vect[i_1].y);
            for (size_t i_2 = 0; i_2 < vect.size(); i_2++)
            {
                if (i_2 == i_1)
                    continue;
                if (vect[i_2].z == BOTTOM_LEFT)
                {
                    vect_src.emplace_back(vect[i_2].x, vect[i_2].y);
                    for (size_t i_3 = 0; i_3 < vect.size(); i_3++)
                    {
                        if (i_3 == i_2 || i_3 == i_1)
                            continue;
                        if (vect[i_3].z == TOP_RIGHT)
                        {
                            vect_src.emplace_back(vect[i_3].x, vect[i_3].y);
                            for (size_t i_4 = 0; i_4 < 4; i_4++)
                            {
                                if (i_4 == i_3 || i_4 == i_2 || i_4 == i_1)
                                    continue;
                                vect_src.emplace_back(vect[i_4].x, vect[i_4].y);
                                break;
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        }
    }

    const vector<Point2f> vect_dst{{0,0}, //左上
                                   {0,30},  //左下
                                   {30,0},  //右上
                                   {30,30}};  //右下

//    if (vect_src.empty())
//        return;
//    Mat transform_mat = getPerspectiveTransform(vect_src, vect_dst);
//    Mat perspective_img; //变换结果图像
//    warpPerspective(Bin_img, perspective_img, transform_mat, Size(30, 30), INTER_LINEAR);
//    imshow("ROI", perspective_img);
}

/**
 * @brief 绘制角点
 *
 * @param rect 待绘制角点旋转矩形
 * @param index 待绘制角点索引值
 * @param src_img 待绘制目标图像
 */
void Engineer_Locator::drawCorners(const RotatedRect &rect, int index, Mat &src_img)
{
    Scalar color;
    Point2f vertices[4];
    rect.points(vertices);
    switch (corner_types_vect[index])
    {
        case (1):
            color = Scalar(255, 255, 0);
            break;
        case (2):
            color = Scalar(0, 255, 0);
            break;
        case (3):
            color = Scalar(255, 0, 0);
            break;
        default:
            color = Scalar(0, 0, 255);
            break;
    }
    for (int i_p = 0; i_p < 4; i_p++)
    {
        line(src_img, vertices[i_p], vertices[(i_p + 1) % 4], color, 2, 8, 0);
    }
}

/**
 * @brief 存储筛选后角点
 *
 * @param rect 待储存角点
 * @param index 待储存角点索引值
 */
//void Engineer_Locator::preserveCorners(const RotatedRect &rect, int index)
//{
//    if(corner_types_vect[index] != 3) //筛除OTHERS类型角点
//    {
//        this->found_corners_vect.push_back(rect);
//        this->found_corner_types_vect.push_back(corner_types_vect[index]);
//    }
//}


/**
 * @brief 检测矿石, 计算六轴数据, 并返回当前识别到的矿石面的值
 *
 * @param ptrs_vect 待判断角点指针容器
 *
 * @return int 0-R, 1-Bar_code, 2-Upside_down_R, 3-Blank, 4-Block_not_found, 5-Left_R, 6-Right_R
 */
int Engineer_Locator::detectBlock(vector<Ptr<Engineer_Corner>> &ptrs_vect)
{
    static int result = 4; //Initial: Cannot find block_four_corners and keep overturning

    int three_corner_count = 0;
    int complete_count = 0;

    int score_not_zero_count = 0;
    int score_equals_10_count = 0;
    int score_over_10_count = 0;
//    int quasi_corner_count = 0;

    this->block_is_found = false;

    //为每个角点打分
    this->calcCornerScore(ptrs_vect);

    //绘制角点分数数据
    for (const auto & corner_ptr : ptrs_vect)
    {
        drawScore(color_img, corner_ptr);
    }

    //对各分数区间计数
    for (const auto & corner_ptr : ptrs_vect)
    {
        if (corner_ptr->score != 0)
        {
            score_not_zero_count++;
            if (corner_ptr->score == 10)
                score_equals_10_count++;
            else if (corner_ptr->score > 10)
                score_over_10_count++;
        }
    }

    if (score_not_zero_count == 0)
    {
        this->block_is_found = false;
        return result;  //return last result;
    }

    //按分数排序
    sort(ptrs_vect.begin(), ptrs_vect.end(),
         [](const Ptr<Engineer_Corner> &corner1, const Ptr<Engineer_Corner> &corner2) -> bool {
             return corner1->score > corner2->score;
         });

    if (score_over_10_count >= 4) //四角点情况
    {
        this->block_is_found = true;

        //按左上-左下-右上-右下的顺序排序
        sort(ptrs_vect.begin(), ptrs_vect.begin() + 4,
             [](const Ptr<Engineer_Corner> &corner1, const Ptr<Engineer_Corner> &corner2) -> bool {
                 return corner1->center.x < corner2->center.x;
             });
        sort(ptrs_vect.begin(), ptrs_vect.begin() + 2,
             [](const Ptr<Engineer_Corner> &corner1, const Ptr<Engineer_Corner> &corner2) -> bool {
                 return corner1->center.y < corner2->center.y;
             });
        sort(ptrs_vect.begin() + 2, ptrs_vect.begin() + 4,
             [](const Ptr<Engineer_Corner> &corner1, const Ptr<Engineer_Corner> &corner2) -> bool {
                 return corner1->center.y < corner2->center.y;
             });

        //判断是否符合比例
        this->block_is_found = this->fitDistortionCoeff(ptrs_vect[0]->center, ptrs_vect[1]->center, ptrs_vect[2]->center, ptrs_vect[3]->center);

        if (this->block_is_found)
        {
            //计算各类型角点个数
            for (size_t i = 0; i < 4; i++)
            {
                if (ptrs_vect[i]->corner_type == THREE_CORNER_RECT)
                {
                    three_corner_count++;
                } else
                {
                    complete_count++;
                }
            }
            if (three_corner_count == 4 || complete_count > 2)
            {
                this->block_is_found = false;
            }
        }

        //匹配block，solvePnP
        if (this->block_is_found)
        {
            Ptr<Engineer_Block> engineer_block;
            vector<Ptr<Engineer_Corner>> temp_ptrs_vect(ptrs_vect.begin(), ptrs_vect.begin() + 4);
            engineer_block = new Engineer_Block(temp_ptrs_vect);

            this->rvec = engineer_block->rvec;
            this->tvec = engineer_block->tvec;

            this->camera_coordinate = engineer_block->camera_coordinate;
            this->mineral_block_coordinate = engineer_block->mineral_block_coordinate;
            this->euler_angles[0] = engineer_block->angle_yaw;
            this->euler_angles[1] = engineer_block->angle_pitch;
            this->euler_angles[2] = engineer_block->angle_row;
            this->block_is_found = this->camera_coordinate.x != -9999;

            //绘制方块
            line(this->color_img, ptrs_vect[0]->center, ptrs_vect[3]->center, Scalar(255, 255, 255), 2);
            line(this->color_img, ptrs_vect[1]->center, ptrs_vect[2]->center, Scalar(255, 255, 255), 2);
            line(this->color_img, ptrs_vect[0]->center, ptrs_vect[1]->center, Scalar(0, 0, 255), 2);
            line(this->color_img, ptrs_vect[0]->center, ptrs_vect[2]->center, Scalar(0, 0, 255), 2);
            line(this->color_img, ptrs_vect[3]->center, ptrs_vect[1]->center, Scalar(0, 0, 255), 2);
            line(this->color_img, ptrs_vect[3]->center, ptrs_vect[2]->center, Scalar(0, 0, 255), 2);
        }
    } else if (score_equals_10_count >= 2) //两角点情况
    {
        Two_Corners_Position position; //方位 --上，下，左，右

        //分离10分角点
        vector<Ptr<Engineer_Corner>> ten_score_ptrs_vect(ptrs_vect.begin() + score_over_10_count,
                                                         ptrs_vect.begin() + score_equals_10_count + score_over_10_count);

        //将10分角点按右-左的顺序排序
        sort(ten_score_ptrs_vect.begin(), ten_score_ptrs_vect.end(),
             [](const Ptr<Engineer_Corner> &corner1, const Ptr<Engineer_Corner> &corner2) -> bool {
                 return corner1->center.x > corner2->center.x;
             });

        double tan_value = fabs((ten_score_ptrs_vect[0]->center.x - ten_score_ptrs_vect[1]->center.x) /
                (ten_score_ptrs_vect[0]->center.y - ten_score_ptrs_vect[1]->center.y));
        bool fitTanValue = tan_value < 0.364 || tan_value > 2.746; //tan(20)
        bool fitSize = fitSizeTwo(*ten_score_ptrs_vect[0], *ten_score_ptrs_vect[1], mineral_block_param.max_size_ratio);
        bool fitAngle = fitAngleTwo(*ten_score_ptrs_vect[0], *ten_score_ptrs_vect[1], mineral_block_param.max_delta_angle);

        this->block_is_found = fitTanValue && fitSize && fitAngle;

        //!以下为取最右面的两个角点进行匹配

        //排除错误匹配
        if (ten_score_ptrs_vect[0]->corner_type == THREE_CORNER_RECT &&
            ten_score_ptrs_vect[1]->corner_type == THREE_CORNER_RECT)
        {
            if (ten_score_ptrs_vect[0]->corner_orientation + ten_score_ptrs_vect[1]->corner_orientation == 5)
            {
                this->block_is_found = false;
            }
        } else if (ten_score_ptrs_vect[0]->corner_type == COMPLETE_RECT &&
                   ten_score_ptrs_vect[1]->corner_type == COMPLETE_RECT)
        {
            this->block_is_found = false;
        }

        if (this->block_is_found)
        {
            //判断上，下，左，右方位
            bool situation_1;  //上下方位true左右方位false
            int situation_2;  //上下: 上true下false 左右: 左true右false
            if (fabs(ten_score_ptrs_vect[0]->center.x - ten_score_ptrs_vect[1]->center.x) >
                fabs(ten_score_ptrs_vect[0]->center.y - ten_score_ptrs_vect[1]->center.y))
            {
                situation_1 = true;
                if (ten_score_ptrs_vect[0]->corner_orientation % 2 == 0 ||
                    ten_score_ptrs_vect[1]->corner_orientation % 2 == 0)
                    situation_2 = true;
                else
                    situation_2 = false;
            } else
            {
                situation_1 = false;
                if (ten_score_ptrs_vect[0]->corner_orientation > 2 ||
                    ten_score_ptrs_vect[1]->corner_orientation > 2)
                    situation_2 = true;
                else
                    situation_2 = false;
            }

            //按左右或上下排序、存储方位并判断错误匹配
            if (situation_1) //上下
            {
                sort(ten_score_ptrs_vect.begin(), ten_score_ptrs_vect.begin() + 2,
                     [](const Ptr<Engineer_Corner> &corner_ptr_1, const Ptr<Engineer_Corner> &corner_ptr_2) -> bool {
                         return corner_ptr_1->center.x < corner_ptr_2->center.x;
                     });

                if (situation_2)
                    position = UP;
                else
                    position = DOWN;

                if (ten_score_ptrs_vect[0]->corner_orientation <= 2 ||
                    ten_score_ptrs_vect[1]->corner_orientation == 3 || ten_score_ptrs_vect[1]->corner_orientation == 4)
                { //左面的向左 或 右面的向右
                    this->block_is_found = false;
                }

                double distance = getDistance(ten_score_ptrs_vect[0]->center, ten_score_ptrs_vect[1]->center);
                if (fabs(distance/(ten_score_ptrs_vect[0]->size.width + ten_score_ptrs_vect[1]->size.width)-2) > 0.5)
                {
                    this->block_is_found = false;
                }
            } else //左右
            {
                sort(ten_score_ptrs_vect.begin(), ten_score_ptrs_vect.begin() + 2,
                     [](const Ptr<Engineer_Corner> &corner_ptr_1, const Ptr<Engineer_Corner> &corner_ptr_2) -> bool {
                         return corner_ptr_1->center.y < corner_ptr_2->center.y;
                     });

                if (situation_2)
                    position = LEFT;
                else
                    position = RIGHT;

                if (ten_score_ptrs_vect[0]->corner_orientation == 1 || ten_score_ptrs_vect[0]->corner_orientation == 3
                    || ten_score_ptrs_vect[1]->corner_orientation % 2 == 0)
                { //上面的向上 或 下面的向下
                    this->block_is_found = false;
                }

                double distance = getDistance(ten_score_ptrs_vect[0]->center, ten_score_ptrs_vect[1]->center);
                if (fabs(distance/(ten_score_ptrs_vect[0]->size.height + ten_score_ptrs_vect[1]->size.height)-2) > 0.5)
                {
                    this->block_is_found = false;
                }
            }
        }

        //匹配block，solvePnP
        if (this->block_is_found)
        {
            Ptr<Engineer_Block> engineer_block;
            vector<Ptr<Engineer_Corner>> temp_ten_score_ptrs_vect(ten_score_ptrs_vect.begin(), ten_score_ptrs_vect.begin() + 2);

            //R上两个角点
            Point2f R_point_first(0, 0), R_point_second(0, 0);
//            bool found_R_1 = false, found_R_2 = false;

            if (position == UP)
            {
                Point2f R_first_expected, R_second_expected; //R上角点期望位置
                vector<KeyPoint> keypoints;
                Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(10);
                detector->detect(mask_img,keypoints);
//                drawKeypoints(color_img, keypoints, color_img, Scalar::all(255), DrawMatchesFlags::DRAW_OVER_OUTIMG|DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                //FIXME fast角点结果依然不佳，可能的原因是R上找到的角点不稳定或世界坐标不准确
                if (!keypoints.empty())
                {
                    float delta_x = fabs(temp_ten_score_ptrs_vect[0]->center.x - temp_ten_score_ptrs_vect[1]->center.x);
                    float mean_height = (temp_ten_score_ptrs_vect[0]->size.height + temp_ten_score_ptrs_vect[1]->size.height) / 2;

                    Point2f up_line_center = (temp_ten_score_ptrs_vect[0]->center + temp_ten_score_ptrs_vect[1]->center) / 2;

//                    R_first_expected = (up_line_center + 0.917 * temp_ten_score_ptrs_vect[0]->center) / 1.917;
//                    R_second_expected = (up_line_center + 0.058 * temp_ten_score_ptrs_vect[0]->center) / 1.058;
                    R_first_expected = (up_line_center + 1.045 * temp_ten_score_ptrs_vect[0]->center) / 2.045;
                    R_second_expected = (up_line_center + 0.081 * temp_ten_score_ptrs_vect[0]->center) / 1.081;

//                    R_first_expected.y += 0.5888 * mean_height;
//                    R_second_expected.y += 1.1268 * mean_height;
                    R_first_expected.y += 0.5333 * mean_height;
                    R_second_expected.y += 1.1333 * mean_height;

//                    circle(color_img, R_first_expected, 2, Scalar(0, 255, 255), 2);
//                    circle(color_img, R_second_expected, 2, Scalar(0, 255, 255), 2);

                    //寻找与期望位置最近的fast角点，并判断是否在期望角点的一定邻域内
                    int index_1 = 0, index_2 = 0;
                    double distance_1, distance_2;
                    for (size_t i = 0; i < keypoints.size(); i++)
                    {
                        if (getDistance(R_first_expected, keypoints[i].pt) < getDistance(R_first_expected, keypoints[index_1].pt))
                        {
                            index_1 = i;
                            distance_1 = getDistance(R_first_expected, keypoints[index_1].pt);
                        }
                        if (getDistance(R_second_expected, keypoints[i].pt) < getDistance(R_second_expected, keypoints[index_2].pt))
                        {
                            index_2 = i;
                            distance_2 = getDistance(R_second_expected, keypoints[index_2].pt);
                        }
                    }
                    if (distance_1 / delta_x < 0.1)
                    {
                        R_point_first = keypoints[index_1].pt;
//                        found_R_1 = true;
                        circle(color_img, R_point_first, 2, Scalar(0, 255, 255), 2);
                    }
                    if (distance_2 / delta_x < 0.05)
                    {
                        R_point_second = keypoints[index_2].pt;
//                        found_R_2 = true;
                        circle(color_img, R_point_second, 2, Scalar(0, 255, 255), 2);
                    }
//                    putText(color_img, to_string(distance_1/delta_x), R_point_first, 0, 0.6, Scalar(0, 255, 255));
//                    putText(color_img, to_string(distance_2/delta_x), R_point_second, 0, 0.6, Scalar(0, 255, 255));
                }
            }

            //利用找到的 fast 角点solvePnP，暂弃用
//            if (position == UP && (found_R_1 || found_R_2))
//            if (position == UP && (found_R_1 && found_R_2))
//            { //USE points on 'R' ,found with fast corner detect
//                engineer_block = new Engineer_Block(temp_ten_score_ptrs_vect, R_point_first, R_point_second, found_R_1,
//                                                    found_R_2);
//            } else
            {
                engineer_block = new Engineer_Block(temp_ten_score_ptrs_vect, position);
            }

            this->rvec = engineer_block->rvec;
            this->tvec = engineer_block->tvec;

            this->camera_coordinate = engineer_block->camera_coordinate;
            this->mineral_block_coordinate = engineer_block->mineral_block_coordinate;
            this->euler_angles[0] = engineer_block->angle_yaw;
            this->euler_angles[1] = engineer_block->angle_pitch;
            this->euler_angles[2] = engineer_block->angle_row;
            this->block_is_found = this->camera_coordinate.x != -9999; //匹配错误
        }

        //绘制方块
        if (this->block_is_found)
        {
            RotatedRect rect1, rect2;

            rect1.size = Size2f(2 * (getAngle(*ten_score_ptrs_vect[0]) == 0 ? ten_score_ptrs_vect[0]->size.height : ten_score_ptrs_vect[0]->size.width),
                                2 * (getAngle(*ten_score_ptrs_vect[0]) == 0 ? ten_score_ptrs_vect[0]->size.width : ten_score_ptrs_vect[0]->size.height));
            rect1.angle = ten_score_ptrs_vect[0]->angle;
            rect1.center = ten_score_ptrs_vect[0]->center;

            rect2.size = Size2f(2 * (getAngle(*ten_score_ptrs_vect[1]) == 0 ? ten_score_ptrs_vect[1]->size.height : ten_score_ptrs_vect[1]->size.width),
                                2 * (getAngle(*ten_score_ptrs_vect[1]) == 0 ? ten_score_ptrs_vect[1]->size.width : ten_score_ptrs_vect[1]->size.height));
            rect2.angle = ten_score_ptrs_vect[1]->angle;
            rect2.center = ten_score_ptrs_vect[1]->center;

            line(this->color_img, ten_score_ptrs_vect[0]->center, ten_score_ptrs_vect[1]->center, Scalar(255, 255, 255), 2);
            ellipse(color_img, rect1, Scalar(0, 0, 255), 2);
            ellipse(color_img, rect2, Scalar(0, 0, 255), 2);
        }
    }

//    if (this->block_is_found)
//    {
//    } else if (result == -1) //No block_four_corners found and last frame is R
//    {
//        return 1; //Keep overturning to continue testing
//    } else //No block_four_corners found
//    {
//        return result; //Return Last result
//    }

    if (!this->block_is_found)
    {
        return result;
    }

    //四角点情况: 判断是哪个面
    if (three_corner_count == 2 && complete_count == 2)
    {
        Point2f block_center(0, 0);
        for (size_t i = 0; i < 4; i++)
        {
            block_center += ptrs_vect[i]->center;
        }
        block_center /= 4;
        Rect rect(block_center.x - (ptrs_vect[2]->center.x - ptrs_vect[0]->center.x) / 4,
                  block_center.y - (ptrs_vect[1]->center.y - ptrs_vect[0]->center.y) / 6,
                  (ptrs_vect[2]->center.x - ptrs_vect[0]->center.x) / 2,
                  (ptrs_vect[1]->center.y - ptrs_vect[0]->center.y) / 3
        );
        rectangle(color_img, rect, Scalar(255, 255, 255), 2);
        //方块中心点邻域ROI
        Mat center_roi = this->Bin_img(rect);
        double sum_result = sum(center_roi)[0];
        if (sum_result > 2550) //Bar code (bottom) surface
        {
            result = 1;
        } else //Top surface
        {
            result = 3;
        }
    } else if (three_corner_count == 3 && complete_count == 1)
    {
        int complete_rect_index;
        for (size_t i = 0; i < 4; i++)
        {
            if (ptrs_vect[i]->corner_type == COMPLETE_RECT)
                complete_rect_index = i;
        }

        switch (complete_rect_index)
        {
            case 0: // upside-down R
                result = 2;
                break;
            case 1: // left R
                result = 5;
                break;
            case 2: // right R
                result = 6;
                break;
            case 3: // normal R
                result = 0;
                break;
            default:
                break;
        }
    } else if (three_corner_count + complete_count == 0)
    {
        result = 4;
    }

    return result;
}

/**
 * @brief 计算角点分数
 *
 * @param m_corner_ptrs_vect 角点智能指针
 *
 * @return double 分数
 */
void Engineer_Locator::calcCornerScore(vector<Ptr<Engineer_Corner>> &m_corner_ptrs_vect)
{
    vector<Point2f> expected_nearby_corner_positions;
    for (size_t i = 0; i < m_corner_ptrs_vect.size(); i++)
    {
        int three_corner_count = 0, complete_corner_count = 0;
        expected_nearby_corner_positions = this->calcExpectedNearbyPositions(m_corner_ptrs_vect[i]);
        calcNearbyCornersCount(m_corner_ptrs_vect, i, expected_nearby_corner_positions,
                               three_corner_count, complete_corner_count);
        m_corner_ptrs_vect[i]->score += 10 * three_corner_count;
        m_corner_ptrs_vect[i]->score += 10 * complete_corner_count;
        if (m_corner_ptrs_vect[i]->corner_type == THREE_CORNER_RECT)
        {
            if ((three_corner_count == 2 && complete_corner_count == 1) ||
                (three_corner_count == 1 && complete_corner_count == 2))
            {
                m_corner_ptrs_vect[i]->score += 20;
            }
        } else if (m_corner_ptrs_vect[i]->corner_type == COMPLETE_RECT)
        {
            if ((three_corner_count == 3 && complete_corner_count == 0) ||
                (three_corner_count == 2 && complete_corner_count == 1))
            {
                m_corner_ptrs_vect[i]->score += 20;
            }
        }
    }
}

/**
 * @brief 计算角点期望邻角点位置
 *
 * @param corner_ptr 角点智能指针
 *
 * @return vector<Point2f> 邻角点位置容器
 */
vector<Point2f> Engineer_Locator::calcExpectedNearbyPositions(const Ptr<Engineer_Corner> &corner_ptr)
{
    vector<Point2f> result;
    Point2f center = corner_ptr->center;
    double angle_deg = fabs(corner_ptr->angle);
    double angle_rad = getAngle(*corner_ptr) / 180 * 3.14159;
//    cout << "angle_rad: " << angle_rad << endl;
//    cout << "cos: " << cos(angle_rad) << endl;
    Size2f size = corner_ptr->size;
//    cout << "size.width:" << size.width << endl;
//    cout << "size.height:" << size.height << endl;
    int sign1 = corner_ptr->corner_orientation > 2 ? 1 : -1;        //左负右正
    int sign2 = corner_ptr->corner_orientation % 2 == 0 ? 1 : -1;   //上负下正
    int sign3 = angle_deg < 45 ? 1 : -1;    //仰负俯正
    double cos_value = cos(angle_rad);
    double sin_value = sin(angle_rad);
    if (corner_ptr->corner_type == THREE_CORNER_RECT)
    {
        result.emplace_back(center.x + 4*size.width*cos_value*sign1, center.y + 4*size.width*sin_value*sign1*sign3);
        result.emplace_back(center.x - 4*size.height*sin_value*sign2*sign3, center.y + 4*size.height*cos_value*sign2);
        result.emplace_back(center.x + 4*(size.width*cos_value*sign1 - size.height*sin_value*sign2*sign3),
                            center.y + 4*(size.width*sin_value*sign1*sign3 + size.height*cos_value*sign2));
    } else if (corner_ptr->corner_type == COMPLETE_RECT)
    {
        //从左向开始顺时针方向旋转共九个
        result.emplace_back(center.x - 4*size.width*cos_value, center.y - 4*size.width*sin_value*sign3);
        result.emplace_back(center.x - 4*(size.width*cos_value - size.height*sin_value*sign3),
                            center.y - 4*(size.width*sin_value*sign3 + size.height*cos_value));
        result.emplace_back(center.x + 4*size.height*sin_value*sign3, center.y - 4*size.height*cos_value);
        result.emplace_back(center.x + 4*(size.width*cos_value + size.height*sin_value*sign3),
                            center.y + 4*(size.width*sin_value*sign3 - size.height*cos_value));
        result.emplace_back(center.x + 4*size.width*cos_value, center.y + 4*size.width*sin_value*sign3);
        result.emplace_back(center.x + 4*(size.width*cos_value - size.height*sin_value*sign3),
                            center.y + 4*(size.width*sin_value*sign3 + size.height*cos_value));
        result.emplace_back(center.x - 4*size.height*sin_value*sign3, center.y + 4*size.height*cos_value);
        result.emplace_back(center.x - 4*(size.width*cos_value + size.height*sin_value*sign3),
                            center.y - 4*(size.width*sin_value*sign3 - size.height*cos_value));
        result.emplace_back(center.x - 4*size.width*cos_value, center.y - 4*size.width*sin_value*sign3);
    }
//    for (size_t i = 0; i < result.size(); i++)
//    {
//        circle(color_img, result[i], 5, Scalar(0 ,255, 255), 1);
//        line(color_img, corner_ptr->center, result[i], Scalar(0, 255, 255), 1);
//    }
    return result;
}

/**
 * @brief 计算附近的各种角点个数
 *
 * @param m_corner_ptrs_vect
 * @param corner_ptr_index
 * @param expected_nearby_positions
 * @param three_corner_count
 * @param complete_corner_count
 */
void Engineer_Locator::calcNearbyCornersCount(vector<Ptr<Engineer_Corner>> &m_corner_ptrs_vect, size_t corner_ptr_index,
                                              vector<Point2f> expected_nearby_positions, int &three_corner_count,
                                              int &complete_corner_count)
{
    Engineer_Corner m_corner = *m_corner_ptrs_vect[corner_ptr_index];
    if (m_corner.corner_type == THREE_CORNER_RECT)
    {
        for (size_t i_1 = 0; i_1 < expected_nearby_positions.size(); i_1++)
        {
            for (size_t i_2 = 0; i_2 < m_corner_ptrs_vect.size() - corner_ptr_index || i_2 <= corner_ptr_index; i_2++)
            {
                bool within_distance_1 = false;
                bool fit_two_1 = false;
                bool within_distance_2 = false;
                bool fit_two_2 = false;
                if (i_2 < m_corner_ptrs_vect.size() - corner_ptr_index)
                {
                    double distance1 = getDistance(expected_nearby_positions[i_1], m_corner_ptrs_vect[corner_ptr_index + i_2]->center);
                    within_distance_1 = distance1 < 1.5 * MAX(m_corner.size.width, m_corner.size.height);
                    fit_two_1 = fitSizeTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                           *m_corner_ptrs_vect[corner_ptr_index + i_2],
                                           mineral_block_param.max_size_ratio) &&
                                fitAngleTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                            *m_corner_ptrs_vect[corner_ptr_index + i_2],
                                            mineral_block_param.max_delta_angle);
                }
                if (i_2 <= corner_ptr_index && i_2)
                {
                    double distance2 = getDistance(expected_nearby_positions[i_1],
                                                   m_corner_ptrs_vect[corner_ptr_index - i_2]->center);
                    within_distance_2 = distance2 < 1.5 * MAX(m_corner.size.width, m_corner.size.height);
                    fit_two_2 = fitSizeTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                           *m_corner_ptrs_vect[corner_ptr_index - i_2],
                                           mineral_block_param.max_size_ratio) &&
                                fitAngleTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                            *m_corner_ptrs_vect[corner_ptr_index - i_2],
                                            mineral_block_param.max_delta_angle);
                }
                if (within_distance_1 && fit_two_1)
                {
                    if (m_corner_ptrs_vect[corner_ptr_index + i_2]->corner_type == THREE_CORNER_RECT)
                    {
                        three_corner_count++;
                    } else
                    {
                        complete_corner_count++;
                    }
                    break;
                } else if (within_distance_2 && fit_two_2)
                {
                    if (m_corner_ptrs_vect[corner_ptr_index - i_2]->corner_type == THREE_CORNER_RECT)
                    {
                        three_corner_count++;
                    } else
                    {
                        complete_corner_count++;
                    }
                    break;
                }
            }
        }
    } else if (m_corner.corner_type == COMPLETE_RECT)
    {
        for (size_t i_0 = 0; i_0 < 7; i_0+=2)
        {
            int temp_three_corner_count = 0, temp_complete_corner_count = 0;
            for (size_t i_1 = i_0; i_1 < i_0 + 3; i_1++)
            {
                for (size_t i_2 = 0; i_2 < m_corner_ptrs_vect.size() - corner_ptr_index || i_2 <= corner_ptr_index; i_2++)
                {
                    bool within_distance_1 = false;
                    bool fit_two_1 = false;
                    bool within_distance_2 = false;
                    bool fit_two_2 = false;
                    if (i_2 < m_corner_ptrs_vect.size() - corner_ptr_index)
                    {
                        double distance1 = getDistance(expected_nearby_positions[i_1], m_corner_ptrs_vect[corner_ptr_index + i_2]->center);
                        within_distance_1 = distance1 < 1.5 * MAX(m_corner.size.width, m_corner.size.height);
                        fit_two_1 = fitSizeTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                               *m_corner_ptrs_vect[corner_ptr_index + i_2],
                                               mineral_block_param.max_size_ratio) &&
                                    fitAngleTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                                *m_corner_ptrs_vect[corner_ptr_index + i_2],
                                                mineral_block_param.max_delta_angle);
                    }
                    if (i_2 <= corner_ptr_index && i_2)
                    {
                        double distance2 = getDistance(expected_nearby_positions[i_1],
                                                       m_corner_ptrs_vect[corner_ptr_index - i_2]->center);
                        within_distance_2 = distance2 < 1.5 * MAX(m_corner.size.width, m_corner.size.height);
                        fit_two_2 = fitSizeTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                               *m_corner_ptrs_vect[corner_ptr_index - i_2],
                                               mineral_block_param.max_size_ratio) &&
                                    fitAngleTwo(*m_corner_ptrs_vect[corner_ptr_index],
                                                *m_corner_ptrs_vect[corner_ptr_index - i_2],
                                                mineral_block_param.max_delta_angle);
                    }
                    if (within_distance_1 && fit_two_1)
                    {
                        if (m_corner_ptrs_vect[corner_ptr_index + i_2]->corner_type == THREE_CORNER_RECT)
                        {
                            temp_three_corner_count++;
                        } else
                        {
                            temp_complete_corner_count++;
                        }
                        break;
                    } else if (within_distance_2 && fit_two_2)
                    {
                        if (m_corner_ptrs_vect[corner_ptr_index - i_2]->corner_type == THREE_CORNER_RECT)
                        {
                            temp_three_corner_count++;
                        } else
                        {
                            temp_complete_corner_count++;
                        }
                        break;
                    }
                }
            }
            three_corner_count = three_corner_count < temp_three_corner_count ? temp_three_corner_count : three_corner_count;
            complete_corner_count = complete_corner_count < temp_complete_corner_count ? temp_complete_corner_count : complete_corner_count;
        }
    }
}

/**
 * @brief 计算两点距离
 *
 * @param point1
 * @param point2
 *
 * @return double distance
 */
double Engineer_Locator::getDistance(const Point2f &point1, const Point2f &point2)
{
    double delta_x = point1.x - point2.x;
    double delta_y = point1.y - point2.y;
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

/**
 * @brief 判断一个旋转矩形与另外一个是否符合大小
 *
 * @param rect1
 * @param rect2
 * @param ratio
 *
 * @return bool fitSize
 */
bool Engineer_Locator::fitSizeTwo(const RotatedRect &rect1, const RotatedRect &rect2, double ratio)
{
    bool fit_width = MAX(MIN(rect1.size.width, rect1.size.height) / MIN(rect2.size.width, rect2.size.height),
                         MIN(rect2.size.width, rect2.size.height) / MIN(rect1.size.width, rect1.size.height)) >
                     ratio ? false : true;
    bool fit_height = MAX(MAX(rect1.size.width, rect1.size.height) / MAX(rect2.size.width, rect2.size.height),
                          MAX(rect2.size.width, rect2.size.height) / MAX(rect1.size.width, rect1.size.height)) >
                      ratio ? false : true;
    return fit_width && fit_height;
}

/**
 * @brief 判断一个旋转矩形与另外一个是否符合角度
 *
 * @param rect1
 * @param rect2
 * @param m_delta_angle
 *
 * @return bool fitAngle
 */
bool Engineer_Locator::fitAngleTwo(const RotatedRect &rect1, const RotatedRect &rect2, double m_delta_angle)
{
    bool fit_angle = fabs(getAngle(rect1) - getAngle(rect2)) <= m_delta_angle;
    return fit_angle;
}

/**
 * @brief 绘制角点分数数据
 *
 * @param src_img
 */
void Engineer_Locator::drawScore(Mat &src_img, const Ptr<Engineer_Corner> &m_corner)
{
    char score_str[10];
    sprintf(score_str, "%.0f", m_corner->score);
    putText(src_img, (string)score_str, m_corner->center,
            FONT_HERSHEY_SIMPLEX, img_cols>640?1.2:0.6, Scalar(0, 255, 255), 2);
}

/**
 * @brief 判断四边形畸变程度是否符合阈值
 * @see https://www.ixueshu.com/document/0cba1e4e97209739318947a18e7f9386.html
 *
 * @param point1-4 四边形四顶点 左上-左下-右上-右下
 *
 * @return
 */
bool Engineer_Locator::fitDistortionCoeff(const Point2f &point1, const Point2f &point2, const Point2f &point3, const Point2f &point4)
{
//    float e1, f1;
    float e2, e3, e4, f2, f3, f4;
    float Ar, Alpha, Tx, Ty;

//    e1 = point1.x + point2.x + point3.x + point4.x;
    e2 = - point1.x - point2.x + point3.x + point4.x;
    e3 = - point1.x + point2.x - point3.x + point4.x;
    e4 = point1.x - point2.x - point3.x + point4.x;

//    f1 = point1.y + point2.y + point3.y + point4.y;
    f2 = - point1.y - point2.y + point3.y + point4.y;
    f3 = - point1.y + point2.y - point3.y + point4.y;
    f4 = point1.y - point2.y - point3.y + point4.y;

    if (e2 == 0 || f3 == 0)
    {
        return false;
    }

    Ar = MAX(fabs(e2 / f3), fabs(f3 / e2));                 //Aspect ratio
    Alpha = fabs(atan(e3 / f3) + atan(f2 / e2)) * 57.2958;  //Skew angle
    Tx = fabs(e4 / e2);         //X-axis taper
    Ty = fabs(f4 / f3);         //Y-axis taper

    cout << "Ar: " << Ar << endl;
    cout << "Alpha: " << Alpha << endl;
    cout << "Tx: " << Tx << endl;
    cout << "Ty: " << Ty << endl;

    return Ar < 1.5 && Alpha < 10 && Tx < 0.08 && Ty < 0.08;
}

/**
 * @brief 绘制pnp结算数据
 *
 * @param src_img
 */
void Engineer_Locator::drawPnpData(Mat &src_img)
{
    char angle_str[20];
//    string Angle_str("YPR(");
    string Angle_str("(");
    sprintf(angle_str, "%.1f", euler_angles[0]);
    Angle_str += angle_str;
    Angle_str += ",";
    sprintf(angle_str, "%.1f", euler_angles[1]);
    Angle_str += angle_str;
    Angle_str += ",";
    sprintf(angle_str, "%.1f", euler_angles[2]);
    Angle_str += angle_str;
    Angle_str += ")";

    char cam_coord_str[20];
//    string Cam_coord_str("XYZ(");
    string Cam_coord_str("(");
    sprintf(cam_coord_str, "%.1f", camera_coordinate.x);
    Cam_coord_str += cam_coord_str;
    Cam_coord_str += ",";
    sprintf(cam_coord_str, "%.1f", camera_coordinate.y);
    Cam_coord_str += cam_coord_str;
    Cam_coord_str += ",";
    sprintf(cam_coord_str, "%.1f", camera_coordinate.z);
    Cam_coord_str += cam_coord_str;
    Cam_coord_str += ")";

    char mineral_coord_str[20];
//    string Mineral_coord_str("XYZ(");
    string Mineral_coord_str("(");
    sprintf(mineral_coord_str, "%.1f", mineral_block_coordinate.x);
    Mineral_coord_str += mineral_coord_str;
    Mineral_coord_str += ",";
    sprintf(mineral_coord_str, "%.1f", mineral_block_coordinate.y);
    Mineral_coord_str += mineral_coord_str;
    Mineral_coord_str += ",";
    sprintf(mineral_coord_str, "%.1f", mineral_block_coordinate.z);
    Mineral_coord_str += mineral_coord_str;
    Mineral_coord_str += ")";

    putText(src_img, Angle_str, Point(0*src_img.cols/640, 470*src_img.rows/480),
            FONT_HERSHEY_SIMPLEX, img_cols>640?1.2:0.6, Scalar(255, 155, 100), 2);
    putText(src_img, Cam_coord_str, Point(185*src_img.cols/640, 470*src_img.rows/480),
            FONT_HERSHEY_SIMPLEX, img_cols>640?1.2:0.6, Scalar(255, 255, 0), 2);
    putText(src_img, Mineral_coord_str, Point(430*src_img.cols/640, 470*src_img.rows/480),
            FONT_HERSHEY_SIMPLEX, img_cols>640?1.2:0.6, Scalar(155, 255, 100), 2);

    if (this->block_is_found)
    {
        vector<Point3f> obj_points;
        obj_points.emplace_back(-100, -100, 0); //左上前
        obj_points.emplace_back(100, -100, 0); //右上前
        obj_points.emplace_back(-100, 100, 0); //左下前
        obj_points.emplace_back(100, 100, 0); //右下前
        obj_points.emplace_back(-100, -100, 200); //左上后
        obj_points.emplace_back(100, -100, 200); //右上后
        obj_points.emplace_back(-100, 100, 200); //左下后
        obj_points.emplace_back(100, 100, 200); //右下后
        vector<Point2f> img_points;
        projectPoints(obj_points, this->rvec, this->tvec, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, img_points);
        line(src_img, img_points[0], img_points[1], Scalar(0, 0, 255), 2);
        line(src_img, img_points[0], img_points[2], Scalar(0, 0, 255), 2);
        line(src_img, img_points[1], img_points[3], Scalar(0, 0, 255), 2);
        line(src_img, img_points[2], img_points[3], Scalar(0, 0, 255), 2);
        line(src_img, img_points[0], img_points[4], Scalar(0, 0, 255), 2);
        line(src_img, img_points[1], img_points[5], Scalar(0, 0, 255), 2);
        line(src_img, img_points[2], img_points[6], Scalar(0, 0, 255), 2);
        line(src_img, img_points[3], img_points[7], Scalar(0, 0, 255), 2);
        line(src_img, img_points[4], img_points[5], Scalar(0, 0, 255), 2);
        line(src_img, img_points[4], img_points[6], Scalar(0, 0, 255), 2);
        line(src_img, img_points[5], img_points[7], Scalar(0, 0, 255), 2);
        line(src_img, img_points[6], img_points[7], Scalar(0, 0, 255), 2);
    }
}

/**
 * @brief 绘制安全区边界
 *
 * @param src_img
 */
void Engineer_Locator::drawSafetyZoneEdge(Mat &src_img, double up_ratio, double down_ratio, double left_ratio, double right_ratio)
{
    bool illegal = (up_ratio < down_ratio) && (left_ratio < right_ratio) && (down_ratio <= 1 && right_ratio <= 1)
                   && (up_ratio >= 0 && left_ratio >= 0);
    if (!illegal)
    {
        cout << "Safety zone ratio error!" << endl;
        return;
    }

    Point2f p1((float)src_img.cols * left_ratio - 2, (float)src_img.rows * up_ratio - 2);
    Point2f p2((float)src_img.cols * left_ratio - 2, (float)src_img.rows * down_ratio + 2);
    Point2f p3((float)src_img.cols * right_ratio + 2, (float)src_img.rows * up_ratio - 2);
    Point2f p4((float)src_img.cols * right_ratio + 2, (float)src_img.rows * down_ratio + 2);

    if (left_ratio > 0)
        line(src_img, p1, p2, Scalar(255, 255, 255), src_img.cols / 640);
    if (up_ratio > 0)
        line(src_img, p1, p3, Scalar(255, 255, 255), src_img.cols / 640);
    if (down_ratio < 1)
        line(src_img, p2, p4, Scalar(255, 255, 255), src_img.cols / 640);
    if (right_ratio < 1)
        line(src_img, p3, p4, Scalar(255, 255, 255), src_img.cols / 640);
}

/**
 * @brief 绘制操作手辅助框
 *
 * @param src_img
 */
void Engineer_Locator::drawAuxiliaryBox(Mat &src_img)
{
    int cols = src_img.cols;
    int rows = src_img.rows;
    Point2f point_1 = Point2f(1.8 / 5 * cols, 1.5 / 5 * rows);
    Point2f point_2 = Point2f(3.2 / 5 * cols, 1.5 / 5 * rows);
    Point2f point_3 = Point2f(1.8 / 5 * cols, 2.5 / 5 * rows);
    Point2f point_4 = Point2f(3.2 / 5 * cols, 2.5 / 5 * rows);
    Point2f point_5 = Point2f(1.8 / 5 * cols, 3.5 / 5 * rows);
    Point2f point_6 = Point2f(3.2 / 5 * cols, 3.5 / 5 * rows);
//    line(src_img, point_1, point_2, Scalar(0, 255, 255), 2);
    line(src_img, point_1, point_3, Scalar(0, 255, 255), 2);
    line(src_img, point_2, point_4, Scalar(0, 255, 255), 2);
//    line(src_img, point_3, point_4, Scalar(0, 255, 255), 2);
    line(src_img, point_3, point_5, Scalar(0, 255, 255), 2);
    line(src_img, point_4, point_6, Scalar(0, 255, 255), 2);
}

/**
 * @brief 计算data_sign值 (1-正转 2-停 3-反转), 并绘制辅助提示信息
 *
 * @param previous_data_sign 上一帧的data_sign值
 * @param m_block_surface
 * @param src_img
 * @param stop_at_every_surface 是否当每个面垂直时均停止 True->方案2 / False->方案1
 *
 * @return int data_sign
 */
int Engineer_Locator::getDataSign(int previous_data_sign, int m_block_surface, Mat &src_img,
                                  bool stop_at_every_surface)
{
    //reset after pre_sign is 2 for 5 times
    if (previous_data_sign == 2)
    {
        static int count = 0;
        count++;
        if (count >= 5)
        {
            count = 0;
            previous_data_sign = 1;
        }
    }

    //判断垂直
    if (stop_at_every_surface)
    {
        if (fabs(euler_angles[1]) < mineral_block_param.block_vertical_fault_tolerance_angle)
        {
            putText(src_img, "DONE!", Point(10, 60), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 0), img_cols / 320);
            cout << "\033[01;32m DONE!!!\033[0m" << endl;
            previous_data_sign = 2; //DONE (stop)
        } else
        {
            putText(src_img, "Overturning...", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
        }
    } else
    {
        switch (m_block_surface)
        {
            static bool change_direction_once = false;
            static bool change_direction_twice = false;

            case 0: // normal R
                //判断垂直
                if (fabs(euler_angles[1]) < mineral_block_param.block_vertical_fault_tolerance_angle)
                {
                    putText(src_img, "DONE!", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                            int(img_cols / 640), Scalar(0, 255, 0), img_cols / 320);
                    cout << "\033[01;32m DONE!!!\033[0m" << endl;
                    previous_data_sign = 2; //DONE (stop)
                    change_direction_twice = change_direction_once = false; //reset
                } else
                {
                    if (euler_angles[1] > 0)
                    {
                        previous_data_sign = 1;
                    }
                    else
                    {
                        if (previous_data_sign == 1)
                        {
                            if (change_direction_once)
                                change_direction_twice = true;
                            else
                                change_direction_once = true;
                        }
                        previous_data_sign = 3;
                    }

                    if (change_direction_twice)
                    { // stop also when change direction to opposite twice
                        putText(src_img, "DONE!", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                                int(img_cols / 640), Scalar(0, 255, 0), img_cols / 320);
                        cout << "\033[01;32m DONE, with switch direction to opposite twice.\033[0m" << endl;
                        previous_data_sign = 2; //DONE (stop)
                        change_direction_twice = change_direction_once = false; //reset
                    } else
                    {
                        putText(src_img, "Overturning... 0 left", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                                int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
                    }
                }
                break;
            case 1: // bar code
                putText(src_img, "Overturning... 1 left", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                        int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
                previous_data_sign = 1;
                break;
            case 2: // upside-down R
                putText(src_img, "Overturning... 2 left", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                        int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
                break;
            case 3: // blank
//                putText(src_img, "Overturning... 3 left", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                putText(src_img, "Overturning... 1 left", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                        int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
                previous_data_sign = 3;
                break;
            case 4: // block not found
                putText(src_img, "Overturning...", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                        int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
                break;
            case 5: // left R
            case 6: // right R
                putText(src_img, "Aborting...", Point(10*img_cols/640, 60*img_rows/480), FONT_HERSHEY_SIMPLEX,
                        int(img_cols / 640), Scalar(255, 0, 255), img_cols / 320);
                if (fabs(euler_angles[1]) < mineral_block_param.block_vertical_fault_tolerance_angle)
                {
                    previous_data_sign = 2;
                    change_direction_twice = change_direction_once = false; //reset
                }
                break;
            default:
                break;
        }
    }
    return previous_data_sign;
}

/**
 * @brief 计算data_surface值 (0-void/finding_block 1-R 2-QR_code 3-Upside_down_R 4-Blank 5-Left_R 6-Right_R), 并绘制辅助提示信息
 *
 * @param m_block_surface
 * @param src_img
 *
 * @return int data_surface
 */
int Engineer_Locator::getDataSurface(int m_block_surface, Mat &src_img)
{
    int m_data_surface_sign;
    switch (m_block_surface)
    {
        case 0:
            putText(src_img, "R", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 0), img_cols / 320);
            m_data_surface_sign = 1;
            break;
        case 1:
            putText(src_img, "Bar code", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
            m_data_surface_sign = 2;
            break;
        case 2:
            putText(src_img, "Upside down R", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
            m_data_surface_sign = 3;
            break;
        case 3:
            putText(src_img, "Blank", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(0, 255, 255), img_cols / 320);
            m_data_surface_sign = 4;
            break;
        case 4:
            putText(src_img, "Finding Block", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(255, 255, 0), img_cols / 320);
            m_data_surface_sign = 0;
            break;
        case 5:
            putText(src_img, "Left R", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(255, 0, 255), img_cols / 320);
            m_data_surface_sign = 0;
            break;
        case 6:
            putText(src_img, "Right R", Point(10*img_cols/640, 30*img_rows/480), FONT_HERSHEY_SIMPLEX,
                    int(img_cols / 640), Scalar(255, 0, 255), img_cols / 320);
            m_data_surface_sign = 0;
            break;
        default:
            m_data_surface_sign = 0;
            break;
    }
    return m_data_surface_sign;
}

/**
 * @brief 处理发送串口数据
 *
 * @param bool block_is_found
 */
void Engineer_Locator::process_data_send(bool is_found_m)
{
    this->data_struct_send.Flag = 30;
    if (is_found_m)
    {
        if (this->current_state == SMALL_ISLAND_AIMING)
        {
            this->data_struct_send.x_up = -mineral_block_coordinate.x;
            this->data_struct_send.y_up = mineral_block_coordinate.z;
            this->data_struct_send.x_down = 0;
            this->data_struct_send.y_down = 730;
        } else if (this->current_state == BIG_ISLAND_AIMING)
        {
            this->data_struct_send.x_down = -mineral_block_coordinate.x;
            this->data_struct_send.y_down = mineral_block_coordinate.z;
            this->data_struct_send.x_up = 0;
            this->data_struct_send.y_up = 630;
        } else
        {
            this->data_struct_send.x_up = 0;
            this->data_struct_send.y_up = 630;
            this->data_struct_send.x_down = 0;
            this->data_struct_send.y_down = 730;
        }

        this->data_struct_send.z_up = euler_angles[0];
        this->data_struct_send.z_down = euler_angles[0];
    } else
    {
        this->data_struct_send.x_up = 0;
        this->data_struct_send.y_up = 630;
        this->data_struct_send.z_up = 0;
        this->data_struct_send.x_down = 0;
        this->data_struct_send.y_down = 730;
        this->data_struct_send.z_down = 0;
    }
    this->data_struct_send.get_mineral_success_sign = 0;
    this->data_struct_send.sign = this->data_sign;
    this->data_struct_send.surface = this->data_surface;
    this->data_struct_send.End = 25;
}

/**
 * @brief 打印输出接收串口数据
 */
void Engineer_Locator::print_data_get() const
{
    cout << "\033[1;32m<------serial port data get------>\033[0m" << endl;
    if (!this->data_struct_get.empty())
    {
        cout << "switch_camera_sign:\t" << (unsigned)this->data_struct_get.back().switch_camera_sign << endl;
        cout << "current_status:\t\t" << (unsigned)this->data_struct_get.back().current_status << endl;
        cout << "program_control:\t" << (unsigned)this->data_struct_get.back().program_control << endl;
    } else
    {
        cout << "switch_camera_sign:\t" << "NAN" << endl;
        cout << "current_status:\t\t" << "NAN" << endl;
        cout << "program_control:\t" << "NAN" << endl;
    }
    cout << "\033[1;32m<-------------------------------->\033[0m" << endl;
}

/**
 * @brief 打印输出发送串口数据
 */
void Engineer_Locator::print_data_send() const
{
    cout << "\033[1;36m<------serial port data send------>\033[0m" << endl;
    cout << "flag:\t" << unsigned(this->data_struct_send.Flag) << endl;
    cout << "x_up:\t\t" << this->data_struct_send.x_up << endl;
    cout << "y_up:\t\t" << this->data_struct_send.y_up << endl;
    cout << "z_up:\t\t" << this->data_struct_send.z_up << endl;
    cout << "x_down:\t\t" << this->data_struct_send.x_down << endl;
    cout << "y_down:\t\t" << this->data_struct_send.y_down << endl;
    cout << "z_down:\t\t" << this->data_struct_send.z_down << endl;
    cout << "get_mineral_success_sign:\t" << unsigned(this->data_struct_send.get_mineral_success_sign) << endl;
    cout << "is_horizontal_sign:\t\t" << unsigned(this->data_struct_send.sign) << endl;
    cout << "surface_sign:\t\t\t" << unsigned(this->data_struct_send.surface) << endl;
    cout << "ww:\t\t" << unsigned(this->data_struct_send.move_w) << endl;
    cout << "aa:\t\t" << unsigned(this->data_struct_send.move_a) << endl;
    cout << "ss:\t\t" << unsigned(this->data_struct_send.move_s) << endl;
    cout << "dd:\t\t" << unsigned(this->data_struct_send.move_d) << endl;
    cout << "qq:\t\t" << unsigned(this->data_struct_send.move_q) << endl;
    cout << "ee:\t\t" << unsigned(this->data_struct_send.move_e) << endl;
    cout << "shift:\t" << unsigned(this->data_struct_send.move_shift) << endl;
    cout << "end:\t" << unsigned(this->data_struct_send.End) << endl;
    cout << "\033[1;36m<--------------------------------->\033[0m" << endl;
}

/**
 * @brief 判断旋转复位状态
 *
 * @return bool true 复位 false 不复位
 */
bool Engineer_Locator::get_rotate_reset_sign()
{
    static uint8_t status_array[2] = {0, 0}; //current, previous
    if (!this->data_struct_get.empty())
    {
        status_array[1] = status_array[0];
        status_array[0] = this->data_struct_get.back().current_status;
    }
    if (status_array[0] == 0 && status_array[1] == 1)
        return true;
    else
        return false;
}
