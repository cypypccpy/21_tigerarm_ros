#include <arm_moveit_kinematics/gpmp2_optimizer.h>

void GPMP2Planner::Plan(ros::NodeHandle nh, std::vector<double> arm_start_pose_, std::vector<double> arm_goal_pose_) {
  // robot state subscriber (used to initialize start state if not passed as param)
    arm_state_topic_ = "robot/arm_state_topic";
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &GPMP2Planner::armStateCallback, this);
    start_conf_, goal_conf_ = gtsam::Vector::Zero(6);
    arm_pos_time_ = ros::Time::now();

  ros::Duration(1.0).sleep();

  // get start from measurement if not passed as param
    start_conf_ = getVector(arm_start_pose_);
    goal_conf_ = getVector(arm_goal_pose_);

    /*
    if (problem_.robot.isThetaNeg())
      problem_.robot.negateTheta(problem_.start_conf);
  */
  // initialize trajectory
  initializeTrajectory(init_values_, start_conf_, goal_conf_);

  // solve with batch gpmp2
  ROS_INFO("Optimizing...");
  int DOF = 6;
  std::vector<double> orientation_ = (std::vector<double>){0, 0, 0, 1};
  std::vector<double> position_ = (std::vector<double>){0, 0, 0};

  // arm's base pose (relative to robot base if mobile_base_ is true)
  gtsam::Pose3  arm_base_ = gtsam::Pose3(gtsam::Rot3::Quaternion(orientation_[3], orientation_[0], orientation_[1], 
    orientation_[2]), gtsam::Point3(getVector(position_)));
  
  // spheres to approximate robot body: js - link id, rs - radius, [xs, ys, zs] - center
  std::vector<double> a_, alpha_, d_, theta_, js_, xs_, ys_, zs_, rs_;
  gpmp2::BodySphereVector spheres_data_;
  a_ = {0, 0.41, 0, 0, 0, 0};
  alpha_ = {1.5708, 3.1416, 1.5708, 1.0472, 1.0472, 3.1416};
  d_ = {0.2755, 0, -0.0098, -0.2501, -0.0856, -0.2228};
  theta_ = {0, -1.5708, 1.5708, 0, -3.1416, 1.5708};
  js_ = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,4,4,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};
      xs_ = {-0.01,-0.26,-0.26,-0.26,0.24,0.24,0.24,0.04,-0.2,-0.2,0.16,0.16,0.16,0.16,0.33,-0.01,
        -0.12,-0.22,-0.32,0.1,0.2,0.3,-0.01,-0.12,-0.22,-0.32,0.1,0.2,0.3,-0.32,-0.32,-0.32,0.32,
        0.32,0.32,0.12,0.14,0.14,0.19,0.14,0.14,0.175,0.175,0.175,0.175,0.175,0.27,0.37,0.37,0.37,
        0.37,0.37,0,0,0,0,0,-0.06,-0.12,-0.18,-0.24,-0.3,-0.36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0};
      ys_ = {-0.01,-0.01,0.15,-0.17,-0.01,0.15,-0.17,-0.01,-0.06,0.04,-0.07,0.05,-0.18,0.16,-0.01,
        -0.24,-0.24,-0.24,-0.24,-0.24,-0.24,-0.24,0.22,0.22,0.22,0.22,0.22,0.22,0.22,-0.01,0.1,-0.13,
        -0.01,0.1,-0.13,-0.01,-0.11,0.09,-0.01,-0.11,0.09,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,
        -0.01,-0.01,-0.01,-0.1,0.08,0,-0.08,-0.155,-0.23,0,0,0,0,0,0,0,-0.01,-0.01,0,0,0,0,0,-0.008,
        0.05,0.05,0.06,0.06,0.035,-0.05,-0.05,-0.06,-0.06,-0.035,0.015,0.025,0,-0.025,-0.015};
      zs_ = {0.22,0.08,0.08,0.08,0.08,0.08,0.08,0.6,0.45,0.45,0.41,0.41,0.41,0.41,0.29,0.31,0.31,0.31,
        0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.31,0.87,
        0.78,0.78,1.07,0.97,0.97,1.2,1.3,1.4,1.5,1.62,1.5,1.5,1.6,1.66,1.66,1.66,0,0,0,0,0,0.03,
        0.03,0.03,0.03,0.03,0.03,-0.05,-0.1,-0.15,-0.2,0,-0.045,0,-0.075,-0.01,0.01,-0.039,-0.067,
        -0.042,-0.01,0.01,-0.039,-0.067,-0.042,-0.055,-0.08,-0.08,-0.08,-0.055};
      rs_ = {0.21,0.08,0.08,0.08,0.08,0.08,0.08,0.18,0.1,0.1,0.06,0.06,0.06,0.06,0.05,0.05,0.05,0.05,
        0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.1,
        0.08,0.08,0.08,0.08,0.08,0.05,0.05,0.05,0.05,0.07,0.05,0.05,0.05,0.045,0.045,0.045,0.053,
        0.053,0.053,0.053,0.053,0.04,0.04,0.04,0.04,0.04,0.04,0.035,0.03,0.035,0.035,0.04,0.04,0.04,
        0.05,0.013,0.013,0.018,0.018,0.018,0.013,0.013,0.018,0.018,0.018,0.02,0.02,0.02,0.02,0.02};


  for (size_t i=0; i<js_.size(); i++)
    spheres_data_.push_back(gpmp2::BodySphere(js_[i], rs_[i], gtsam::Point3(xs_[i], ys_[i], zs_[i])));

    gpmp2::ArmModel arm = gpmp2::ArmModel(gpmp2::Arm(DOF, getVector(a_), getVector(alpha_), getVector(d_), arm_base_, getVector(theta_)), 
        spheres_data_);

    gpmp2::SignedDistanceField sdf;
    sdf.loadSDF("../empty_sdf.bin");

    gpmp2::TrajOptimizerSetting opt_setting;
  opt_setting = gpmp2::TrajOptimizerSetting(DOF);
  opt_setting.total_time = 10.0;
  opt_setting.total_step = 10-1;
  double delta_t = 10.0/(10-1);
  opt_setting.obs_check_inter = 5;
  opt_setting.cost_sigma = 0.005;
  opt_setting.epsilon = 0.1;
  opt_setting.Qc_model = gtsam::noiseModel::Gaussian::Covariance(1*gtsam::Matrix::Identity(DOF, DOF));
  opt_setting.conf_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, 0.0001);
  opt_setting.vel_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, 0.0001);
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::LM;
  if (true)
    batch_values_ = gpmp2::BatchTrajOptimize3DArm(arm, sdf, start_conf_, 
      gtsam::Vector::Zero(DOF), goal_conf_, gtsam::Vector::Zero(DOF), init_values_, opt_setting);

  ROS_INFO("Batch GPMP2 optimization complete.");
}

void GPMP2Planner::initializeTrajectory(gtsam::Values& init_values, gtsam::Vector start_, gtsam::Vector goal_) {
  ROS_INFO("Initializing trajectory.");
  gtsam::Vector conf, avg_vel;

  double total_time = 10.0;
  int total_step = 10;
  if (true)
  {
    avg_vel = (goal_ - start_conf_)/total_time;
    for (size_t i=0; i<total_step; i++)
    {
      double ratio = static_cast<double>(i)/static_cast<double>(total_step-1);
      conf = (1.0 - ratio)*start_ + ratio*goal_;
      init_values.insert(gtsam::Symbol('x',i), conf);
      init_values.insert(gtsam::Symbol('v',i), avg_vel);
    }
  }
}

/**
 *  Convert std double vector type to gtsam Vector
 *
 *  @param v std double vector
 **/
inline static const gtsam::Vector getVector(const std::vector<double>& v)
{
  gtsam::Vector send(v.size());
  for (size_t i=0; i<v.size(); i++)
    send[i] = v[i];
  return send;
}

