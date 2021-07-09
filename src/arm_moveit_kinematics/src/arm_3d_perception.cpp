#include "arm_moveit_kinematics/arm_3d_perception.h"

Arm3DPerception::Arm3DPerception():yolo("/home/robotlab/Desktop/MineralDetection/yolov3-tiny.cfg",
                                  "/home/robotlab/Desktop/MineralDetection/yolov3-tiny_final.weights",
                                  "/home/robotlab/Desktop/MineralDetection/OpenYolo/config/camPara.yml") {

  //point_suber = node_handle_.subscribe("/camera/depth_registered/points", 1, &Arm3DPerception::imageCB, this);
  depth_suber = node_handle_.subscribe("/camera/depth_registered/image_raw", 1, &Arm3DPerception::DepthCB, this);
}

void Arm3DPerception::DepthCB(const sensor_msgs::PointCloud2ConstPtr& input) {
  
  
};

void detect_mineral(const cv::Mat& src_img) {
  yolo_out = yolo.inference(src_img); //推理模型得到结果

  if (yolo_out.boxes.size() <= 0)
  {
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
}

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