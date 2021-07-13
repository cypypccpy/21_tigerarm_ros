#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <tf/transform_broadcaster.h>
#include "detect.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class Arm3DPerception {
    public:
        explicit Arm3DPerception();

        void CloudCB(const sensor_msgs::PointCloud2ConstPtr& point);

        void DepthCB(const sensor_msgs::ImageConstPtr& image_raw);

        void key_recv_callback(const std_msgs::Int32& msg);

        void detect_roi(cv::Mat& src_img);

        void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

        void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

        ~Arm3DPerception() {};

    private:
        bool points_not_found = true;
        bool aim = false;

        ros::NodeHandle node_handle_;
        image_transport::Publisher ui_puber_;
        ros::Subscriber point_suber_;
        ros::Subscriber depth_suber_;
        ros::Subscriber keyboard_suber_;
        ros::Publisher vis_cloud_puber;

        Object yolo_out;
        sensor_msgs::PointCloud2 vis_cloud;
        //detect yolo;
};