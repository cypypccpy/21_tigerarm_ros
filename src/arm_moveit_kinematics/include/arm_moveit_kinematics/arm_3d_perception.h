#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class Arm3DPerception {
    public:
        Arm3DPerception() {};

        void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
        void imageCB(const sensor_msgs::PointCloud2ConstPtr& input);

    private:
        bool points_not_found = true;

        ros::NodeHandle node_handle_;
        ros::Subscriber point_suber_;
        ros::Subscriber depth_suber_;
}