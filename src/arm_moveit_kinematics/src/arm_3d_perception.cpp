#include "arm_moveit_kinematics/arm_3d_perception.h"

Arm3DPerception::Arm3DPerception() {

  //point_suber_ = node_handle_.subscribe("/camera/depth_registered/points", 1, &Arm3DPerception::CloudCB, this);
  depth_suber_ = node_handle_.subscribe("/camera/color/image_raw", 100, &Arm3DPerception::DepthCB, this);
  keyboard_suber_ = node_handle_.subscribe("arm_keys", 100, &Arm3DPerception::key_recv_callback, this);

  image_transport::ImageTransport it = image_transport::ImageTransport(node_handle_);
  ui_puber_ = it.advertise("camera/image", 100);
  vis_cloud_puber = node_handle_.advertise<sensor_msgs::PointCloud2>("/camera/vis_cloud", 100);

}

void Arm3DPerception::key_recv_callback(const std_msgs::Int32& msg) {
  if (msg.data == 113) {
    aim = true;
  }
  else
    aim = false;
}

void Arm3DPerception::DepthCB(const sensor_msgs::ImageConstPtr& image_raw) {
  cv::Mat src_img = cv_bridge::toCvShare(image_raw, "bgr8")->image;
  
  //辅助对准矿石
  if (aim == true) {
    Rect r(212, 241, 294, 174);
    rectangle(src_img, r, Scalar(0, 0, 255), 2);
  }

  //发布图片消息
  sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img).toImageMsg();
  ui_puber_.publish(msg1);
  
};
/*
void Arm3DPerception::detect_roi(cv::Mat& src_img) {
  yolo_out = yolo.inference(src_img); //推理模型得到结果

  if (yolo_out.boxes.size() <= 0)
  {
    return;
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
}
*/

void Arm3DPerception::CloudCB(const sensor_msgs::PointCloud2ConstPtr& point) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*point, *cloud);

  passThroughFilter(cloud);
  // Declare normals and call function to compute point normals.
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  computeNormals(cloud, cloud_normals);
}
//--------------------------tool----------------------------------

/** \brief Given a pointcloud extract the ROI defined by the user.
    @param cloud - Pointcloud whose ROI needs to be extracted. */
void Arm3DPerception::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // min and max values in z axis to keep
  pass.setFilterLimits(0.5, 1.0);
  pass.filter(*cloud);
}

/** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
    @param cloud - Pointcloud.
    @param cloud_normals - The point normals once computer will be stored in this. */
void Arm3DPerception::computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::toROSMsg(*cloud, vis_cloud);
  vis_cloud_puber.publish(vis_cloud);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "arm_3d_perception");
  ros::NodeHandle node_handle;

  Arm3DPerception arm_3d_perception;

  /* Multi threading */
  ros::AsyncSpinner spinner(2); // two thread
  spinner.start();

  ros::waitForShutdown();

}