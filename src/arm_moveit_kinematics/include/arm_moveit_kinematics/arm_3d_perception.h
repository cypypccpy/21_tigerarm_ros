#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <tf/transform_broadcaster.h>
#include "cv-helpers.hpp"
#include "detect.h"
#include "PlaneExtract.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class Arm3DPerception {
    public:
        explicit Arm3DPerception();

        //void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
        void DepthCB(const sensor_msgs::ImageConstPtr& input);

        void key_recv_callback(const std_msgs::Int32& msg);

        void detect_mineral(cv::Mat& src_img);

        ~Arm3DPerception() {};

    private:
        bool points_not_found = true;
        bool aim = false;

        ros::NodeHandle node_handle_;
        image_transport::Publisher ui_puber_;
        ros::Subscriber point_suber_;
        ros::Subscriber depth_suber_;
        ros::Subscriber keyboard_suber_;

        Object yolo_out;
        detect yolo;
};