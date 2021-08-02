#include <map>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>

class GPMP2Planner
{
  private:
    gtsam::Values init_values_, batch_values_, exec_values_;

    std::string arm_state_topic_, base_state_topic_;
    ros::Subscriber arm_state_sub_, base_state_sub_;
    gtsam::Vector start_conf_, goal_conf_;
    gtsam::Pose2 base_pos_;
    ros::Time arm_pos_time_, base_pos_time_;

  public:
    /// Default constructor
    GPMP2Planner() {}

    /**
     *  batch gpmp2
     *
     *  @param nh node handle for namespace
     **/
    GPMP2Planner(ros::NodeHandle nh);

    /// Default destructor
    virtual ~GPMP2Planner() {}

    void initializeTrajectory(gtsam::Values& init_values, gtsam::Vector start_, gtsam::Vector goal_);

    /**
     *  Open-loop execution of GPMP2
     **/
    void Plan(ros::NodeHandle nh, std::vector<double> arm_start_pose_, std::vector<double> arm_goal_pose_);

    /**
     *  Call back to get current state of the arm
     *
     *  @param msg message from arm state subscriber
     **/
    void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     *  Call back to get current state of the base
     *
     *  @param msg message from base state subscriber
     **/
    void baseStateCallback(const geometry_msgs::Pose::ConstPtr& msg);
};