#include "arm_moveit_kinematics/arm_3d_perception.h"

Arm3DPerception::Arm3DPerception():yolo("/home/robotlab/Desktop/MineralDetection/yolov3-tiny.cfg",
                                  "/home/robotlab/Desktop/MineralDetection/yolov3-tiny_final.weights",
                                  "/home/robotlab/Desktop/MineralDetection/OpenYolo/config/camPara.yml") {

  //point_suber = node_handle_.subscribe("/camera/depth_registered/points", 1, &Arm3DPerception::imageCB, this);
  depth_suber_ = node_handle_.subscribe("/camera/depth_registered/image_raw", 1, &Arm3DPerception::DepthCB, this);
  keyboard_suber_ = node_handle_.subscribe("arm_keys", 100, &Arm3DPerception::key_recv_callback, this);

  image_transport::ImageTransport it = image_transport::ImageTransport(node_handle_);
  ui_puber_ = it.advertise("camera/image", 100);
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

void Arm3DPerception::detect_mineral(cv::Mat& src_img) {
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