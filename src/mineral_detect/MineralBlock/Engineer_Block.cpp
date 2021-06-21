//
// Created by cui on 2021/4/23.
//

#include "Engineer_Block.h"

Engineer_Block::Engineer_Block(vector<Ptr<Engineer_Corner>> &corner_ptrs_vect)
{
    vector<Point2f> corner_imagePoints(4); //角点像素坐标

    for (int i = 0; i < 4; i++)
    {
        corner_imagePoints[i] = corner_ptrs_vect[i]->center;
    }

    solvePnP(block_four_corners, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);

    rvec.convertTo(Rvec, CV_32F);
    tvec.convertTo(Tvec, CV_32F);

    Rodrigues(Rvec, Rmat);

    //欧拉角
    this->calcEularAngles();

    //相对坐标
    this->calcCameraCoordinate();
    this->calcMineralBlockCoordinate();
}

Engineer_Block::Engineer_Block(vector<Ptr<Engineer_Corner>> &corner_ptrs_vect, Two_Corners_Position &position)
{
    vector<Point2f> corner_imagePoints; //角点像素坐标
    if (position <= 2) //上下
    {
        if (position == UP)
        {
            corner_imagePoints.push_back(corner_ptrs_vect[0]->center);
            corner_imagePoints.push_back(corner_ptrs_vect[1]->center);
        }
        corner_imagePoints.emplace_back(corner_ptrs_vect[0]->vertices_vect[0].x,
                                        corner_ptrs_vect[0]->vertices_vect[0].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[0]->vertices_vect[1].x,
                                        corner_ptrs_vect[0]->vertices_vect[1].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[1]->vertices_vect[2].x,
                                        corner_ptrs_vect[1]->vertices_vect[2].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[1]->vertices_vect[3].x,
                                        corner_ptrs_vect[1]->vertices_vect[3].y);
    } else //左右
    {
        corner_imagePoints.emplace_back(corner_ptrs_vect[0]->vertices_vect[0].x,
                                        corner_ptrs_vect[0]->vertices_vect[0].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[1]->vertices_vect[1].x,
                                        corner_ptrs_vect[1]->vertices_vect[1].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[0]->vertices_vect[2].x,
                                        corner_ptrs_vect[0]->vertices_vect[2].y);
        corner_imagePoints.emplace_back(corner_ptrs_vect[1]->vertices_vect[3].x,
                                        corner_ptrs_vect[1]->vertices_vect[3].y);
    }

    switch (position)
    {
        case UP:
            solvePnP(block_two_corners_up_six, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);
            break;
        case DOWN:
            solvePnP(block_two_corners_down, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);
            break;
        case LEFT:
            solvePnP(block_two_corners_left, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);
            break;
        case RIGHT:
            solvePnP(block_two_corners_right, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);
            break;
        default:
            break;
    }

    rvec.convertTo(Rvec, CV_32F);
    tvec.convertTo(Tvec, CV_32F);

    Rodrigues(Rvec, Rmat);

    //欧拉角
    this->calcEularAngles();

    //相对坐标
    this->calcCameraCoordinate();
    this->calcMineralBlockCoordinate(position == UP);
}

Engineer_Block::Engineer_Block(vector<Ptr<Engineer_Corner>> &corner_ptrs_vect, const Point2f &R_point1,
                               const Point2f &R_point2, bool m_found_R_1, bool m_found_R_2)
{
    vector<Point2f> corner_imagePoints; //角点像素坐标

    corner_imagePoints.push_back(corner_ptrs_vect[0]->center);
    corner_imagePoints.push_back(corner_ptrs_vect[1]->center);
    if (m_found_R_1)
        corner_imagePoints.push_back(R_point1);
    if (m_found_R_2)
        corner_imagePoints.push_back(R_point2);

    //FIXME three-point: CV_Assert !_rvec.empty() && !_tvec.empty() in function 'solvePnPGeneric'
//    if (m_found_R_1 && !m_found_R_2)
//    {
//        solvePnP(block_two_corners_up_with_R_point1, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_ITERATIVE);
//    } else if (!m_found_R_1 && m_found_R_2)
//    {
//        solvePnP(block_two_corners_up_with_R_point2, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_ITERATIVE);
//    } else
    if (m_found_R_1 && m_found_R_2)
    {
        solvePnP(block_two_corners_up_with_R_point1_2, corner_imagePoints, camera_param.cameraMatrix_4mm_1, camera_param.distCoeff_4mm_1, rvec, tvec, true, SOLVEPNP_EPNP);
    }

    rvec.convertTo(Rvec, CV_32F);
    tvec.convertTo(Tvec, CV_32F);

    Rodrigues(Rvec, Rmat);

    //欧拉角
    this->calcEularAngles();

    //相对坐标
    this->calcCameraCoordinate();
    this->calcMineralBlockCoordinate(true);
}

void Engineer_Block::calcEularAngles()
{
    angle_yaw = atan2(-Rmat.at<float>(Point(2, 0)), sqrt(Rmat.at<float>(Point(2, 0)) * Rmat.at<float>(Point(2, 0)) + Rmat.at<float>(Point(2, 2)) * Rmat.at<float>(Point(2, 2)))) * 57.2958;
    angle_pitch = atan2(Rmat.at<float>(Point(2, 1)), Rmat.at<float>(Point(2, 2))) * 57.2958;
    angle_row = atan2(Rmat.at<float>(Point(1, 0)), Rmat.at<float>(Point(0, 0))) * 57.2958;
}

void Engineer_Block::calcCameraCoordinate()
{
    Mat result = - Rmat.t() * Tvec;
    float temp_x = result.at<float>(0, 0);
    float temp_y = result.at<float>(1, 0);
    float temp_z = result.at<float>(2, 0);
    if (fabs(temp_x) < 5000 && fabs(temp_y) < 2000 && fabs(temp_z) < 5000) //过大保护
    {
        camera_coordinate.x = temp_x;
        camera_coordinate.y = temp_y;
        camera_coordinate.z = temp_z;
    } else
    {
        camera_coordinate.x = -9999;
        camera_coordinate.y = -9999;
        camera_coordinate.z = -9999;
    }
}

void Engineer_Block::calcMineralBlockCoordinate(bool use_up_cor)
{
    float temp_x, temp_y, temp_z;
    if (use_up_cor)
    { // Center of up edge
        Mat R_T_mat;
        hconcat(Rmat, Tvec, R_T_mat);
        Mat up_center_cor = Mat::zeros(4, 1, CV_32F);
        up_center_cor.at<float>(0, 0) = 0;
        up_center_cor.at<float>(1, 0) = -60;
        up_center_cor.at<float>(2, 0) = 0;
        up_center_cor.at<float>(3, 0) = 1;
        Mat result = R_T_mat * up_center_cor;

        temp_x = result.at<float>(0, 0);
        temp_y = result.at<float>(1, 0);
        temp_z = result.at<float>(2, 0);
    } else
    {
        temp_x = Tvec.at<float>(0, 0);
        temp_y = Tvec.at<float>(1, 0);
        temp_z = Tvec.at<float>(2, 0);
    }
    if (fabs(temp_x) < 5000 && fabs(temp_y) < 2000 && fabs(temp_z) < 5000) //过大保护
    {
        mineral_block_coordinate.x = temp_x;
        mineral_block_coordinate.y = temp_y;
        mineral_block_coordinate.z = temp_z;
    } else
    {
        mineral_block_coordinate.x = -9999;
        mineral_block_coordinate.y = -9999;
        mineral_block_coordinate.z = -9999;
    }
}
