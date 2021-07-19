#include "arm_moveit_kinematics/arm_3d_perception.h"

Arm3DPerception::Arm3DPerception():aim(false), overload(false) {

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
  if (msg.data == 255) {
    overload = true;
  }
}

void Arm3DPerception::DepthCB(const sensor_msgs::ImageConstPtr& image_raw) {
  cv::Mat src_img = cv_bridge::toCvShare(image_raw, "bgr8")->image;
  
  //辅助对准矿石
  if (aim == true) {
    Rect r(212, 241, 294, 174);
    rectangle(src_img, r, Scalar(0, 0, 255), 2);
  }

  if (overload) {
    circle(src_img, cv::Point2f(320, 240), 50, cv::Scalar(0, 0, 255), 2);
    overload = false;
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
  //计算法线
  computeNormals(cloud, cloud_normals);
  //计算内平面
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  removePlaneSurface(cloud, inliers_plane);

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
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  pcl::toROSMsg(*cloud, vis_cloud);
  vis_cloud_puber.publish(vis_cloud);
}

/** \brief Given the pointcloud and indices of the plane, remove the plannar region from the pointcloud.
    @param cloud - Pointcloud.
    @param inliers_plane - Indices representing the plane. */
void Arm3DPerception::removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
{
  // create a SAC segmenter without using normals
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  /* run at max 1000 iterations before giving up */
  segmentor.setMaxIterations(1000);
  /* tolerance for variation from model */
  segmentor.setDistanceThreshold(0.01);
  segmentor.setInputCloud(cloud);
  /* Create the segmentation object for the planar model and set all the parameters */
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);
  /* Extract the planar inliers from the input cloud */
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  /* Remove the planar inliers, extract the rest */
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}

/** \brief Given the point normals and point indices, extract the normals for the indices.
    @param cloud_normals - Point normals.
    @param inliers_plane - Indices whose normals need to be extracted. */
void Arm3DPerception::extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

/** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the plane from the
   pointcloud and store the plane parameters in coefficients_plane.
    @param cloud - Pointcloud whose plane is removed.
    @param coefficients_plane - plane parameters used to define an infinite plane will be stored here.
    @param cloud_normals - Point normals corresponding to the plane on which plane is kept */
void Arm3DPerception::extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_plane,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  // Create the segmentation object for plane segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(10000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(0.05);
  // min max values of radius in meters to consider
  segmentor.setRadiusLimits(0, 1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Obtain the plane inliers and coefficients
  segmentor.segment(*inliers_plane, *coefficients_plane);

  // Extract the plane inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);
  extract.filter(*cloud);
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