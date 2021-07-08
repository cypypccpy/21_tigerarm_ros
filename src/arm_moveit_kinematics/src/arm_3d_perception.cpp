#include "arm_moveit_kinematics/arm_3d_perception.h"

Arm3DPerception::Arm3DPerception() {
  point_suber = node_handle_.subscribe("/camera/depth_registered/points", 1, &Arm3DPerception::imageCB, this);
  depth_suber = node_handle_.subscribe("/camera/depth_registered/image_raw", 1, &Arm3DPerception::cloudCB, this);
}

void Arm3DPerception::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
    
    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
      return;
    }
    if (points_not_found)
    {
      points_not_found = false;
    }
  }
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "arm_3d_perception");

  ros::AsyncSpinner spinner(4); // four thread
  spinner.start();
  ros::Duration sleep_t(1.0);
  sleep_t.sleep();

  Arm3DPerception();

  ros::waitForShutdown();

}