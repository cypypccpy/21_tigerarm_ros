// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
                                // Include short list of convenience functions for rendering
#include <iostream>
#include <sstream>
#include <math.h>
#include <algorithm> // std::min, std::max
#include <tf/transform_broadcaster.h>
#include "cv-helpers.hpp"
#include "detect.h"
#include "PlaneExtract.h"
#include "ros/ros.h"
#include "mineral_detect/Mineral.h"
#include <geometry_msgs/PoseStamped.h>

#include "CmdlineParser.h"
#include "Parameter.h"
#include "SerialPort.h"
#include "Engineer_Locator.h"

using namespace cv;
using namespace rs2;
using namespace std;

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "mineral_detect");
  mineral_detect::Mineral mineral_msg;
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("mineral_detect", 1000);
  ros::Rate loop_rate(10);
  PlaneExtract plane;
  rs2::align align_to(RS2_STREAM_COLOR);
  Object yolo_out;
  detect yolo("/home/robotlab/Desktop/MineralDetection/yolov3-tiny.cfg", "/home/robotlab/Desktop/MineralDetection/yolov3-tiny_final.weights", "/home/robotlab/Desktop/MineralDetection/OpenYolo/config/camPara.yml");

  //---------------------------------崔桐欣代码test------------------------------------
  Ptr<CmdlineParser> cmdline_parser(new CmdlineParser(argc, argv));
  Ptr<Engineer_Locator> Engineer_locator(new Engineer_Locator(cmdline_parser, 640, 480));

  //cv::VideoWriter writer;
  //writer.open("/home/robotlab/Desktop/video.mp4", cv::VideoWriter::fourcc('M', 'P', '4', '2'), 5, cv::Size(640, 480));

  rs2::pointcloud pc;
  rs2::points points;

  rs2::pipeline pipe;
  pipe.start();

  while (ros::ok()) // Application still alive?
  {

    float FPS = getTickCount();

    // Wait for the next set of frames from the camera
    frameset frames = pipe.wait_for_frames();
    frameset aligned_set = align_to.process(frames);
    frame depth = aligned_set.get_depth_frame();
    // frame bw_depth = depth.apply_filter(colorize);
    Mat depth_mat = frame_to_mat(depth);

    // cvtColor(depth_mat, depth_mat, COLOR_BGR2GRAY);
    auto src_img = frame_to_mat(aligned_set.get_color_frame());
    stringstream ss;
    ss << FPS;
    string time_stamp;
    ss >> time_stamp;

    // if (waitKey(1) == 'r')
    // {
    //   imwrite("1" + time_stamp + ".png", src_img);
    // }
    
  //---------------------------------崔桐欣代码test------------------------------------
  
    Engineer_locator->blockLocator(src_img, 2);

/*    
    yolo_out = yolo.inference(src_img); //推理模型得到结果

    if (yolo_out.boxes.size() <= 0)
    {

      imshow("src", src_img);
      waitKey(1);
      continue;
    }

    sort(yolo_out.boxes.begin(), yolo_out.boxes.end(),
         [](const Rect2i box1, const Rect2i box2)
         {
           return box1.area() > box2.area();
         });
    if (yolo_out.boxes[0].x <= 0)
    {
      yolo_out.boxes[0].x = 1;
    }
    if (yolo_out.boxes[0].y <= 0)
    {
      yolo_out.boxes[0].y = 1;
    }
    if (yolo_out.boxes[0].br().x >= 639)
    {
      yolo_out.boxes[0].width = 638 - yolo_out.boxes[0].x;
    }
    if (yolo_out.boxes[0].br().y >= 479)
    {
      yolo_out.boxes[0].height = 478 - yolo_out.boxes[0].y;
    }

    Point2i center_point = (yolo_out.boxes[0].br() + yolo_out.boxes[0].tl()) / 2; //求中心点;

    Mat mask;
    Mat region = depth_mat(yolo_out.boxes[0]);
    plane.getPlane(region, mask, yolo.cameraMatrix);

    int label = plane.getMaxPlane(mask);

    Mat color_mask(mask.rows, mask.cols, CV_8UC3);  

    plane.colorMask(mask, color_mask);

    Point center_p = plane.getPlaneCentroid(mask);

    circle(src_img, yolo_out.boxes[0].tl() + center_p, 5, Scalar(255, 255, 255), 2);

    imshow("normal", mask);
    imshow("mask", color_mask);
*/

    imshow("edge", Engineer_locator->Edge_img);

    imshow("src", src_img);
    waitKey(1);
    FPS = getTickFrequency() / (getTickCount() - FPS);
    cout << "FPS: " << FPS << endl;
/*
    if(center_p.x < 0 || center_p.y < 0 || center_p.x > mask.cols || center_p.y > mask.rows)
    {
        continue;
    }

    //创建tf广播器
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
    transform = tf::Transform(quaternion, tf::Vector3(-0.22, 0.01, -0.59));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "realsense", "base_link"));
    
    geometry_msgs::PoseStamped mineral_pose;
    mineral_pose.header.frame_id = "realsense";
    mineral_pose.header.stamp = ros::Time();
    mineral_pose.pose.position.z = plane.mat3d.at<Vec3f>(center_p)[0];
    mineral_pose.pose.position.y = plane.mat3d.at<Vec3f>(center_p)[1];
    //mineral_pose.pose.position.y = 0;
    mineral_pose.pose.position.x = plane.mat3d.at<Vec3f>(center_p)[2];
    geometry_msgs::Quaternion q;

    float Yaw = abs(atan(plane.plane_normals.at<Vec4f>(label)[1] / plane.plane_normals.at<Vec4f>(label)[0]));
    float Pitch = abs(atan(Yaw / sqrt(pow(plane.plane_normals.at<Vec4f>(label)[1], 2) + pow(plane.plane_normals.at<Vec4f>(label)[0], 2))));
    q = tf::createQuaternionMsgFromRollPitchYaw(3.00,
                                                Pitch - 1.57,
                                                Yaw - 3.14); //取出方向存储于四元数

    cout << "Pitch: " << Pitch << endl;
    cout << "Yaw: " << Yaw << endl; 
    
    mineral_pose.pose.orientation = q;

    cout << "三维坐标： "  <<  plane.mat3d.at<Point3f>(center_p) << endl;
    cout << "平面方程: " << plane.plane_normals.at<Vec4f>(label) << endl;
    chatter_pub.publish(mineral_pose);
    ros::spinOnce();
    loop_rate.sleep();
    */
  }

  //writer.release();
  return EXIT_SUCCESS;
}
