#include <arm_moveit_kinematics/arm_moveit_kinermatics_node.h>

void ArmJointsControllerNode::set_offline_pose() {

    geometry_msgs::PoseStamped g_pose_target_stamp;
    std::vector<std::string> pose_name;

    //抢矿
    pose_name = {"temp_point", "pick_island"};
    speed_scale = 0.71;
    set_target_pose(pose_name);
    g_pose_offline_pose.push_back(g_pose_muilt_target);

    arm_pose_updated = false;
    std::vector<geometry_msgs::Pose>().swap(g_pose_muilt_target); 

}

void ArmJointsControllerNode::compute_offline_trajectory() {

    /* Update target position */
    for (int i = 0; i < g_pose_offline_pose.size(); i++) {
        
        std::vector<double> joint_group_position = {0, -0.254034, 0.0116232, 0, -0.151452, -0.001};
        std::vector<double> joint_target_position = {0, -0.375363, 0.167894, 0, -0.11588, -0.001};

        moveit::core::RobotStatePtr offline_start_state(move_group_interface.getCurrentState());
        offline_start_state->setJointGroupPositions("arm", joint_group_position);
        move_group_interface.setStartState(*offline_start_state);

        moveit::planning_interface::MoveGroupInterface::Plan MuiltPlan;

        MuiltPlan.trajectory_ = compute_trajectory(g_pose_offline_pose[i], &move_group_interface);

        /* 人工赋予轨迹点 */
        for (int j=1;j<MuiltPlan.trajectory_.joint_trajectory.points.size() - 1;j++) {
            MuiltPlan.trajectory_.joint_trajectory.points[j].positions = joint_target_position;
        }

        /* Save trajectory */
        my_offline_plan.push_back(MuiltPlan);

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan>().swap(my_muilt_plan);  
    }
    ROS_INFO("offline planning completed!");

}

void ArmJointsControllerNode::offline_move_task(const int& index) {

    move_group_interface.setStartStateToCurrentState();

    auto tic = ros::Time::now();
    move_group_interface.execute(my_offline_plan[index]);
    auto toc = ros::Time::now() - tic;
    ROS_INFO("Execute time %.2fms", toc.toSec() * 1000);

    arm_states_.execute_finished = true;
    arm_puber_.publish(arm_states_);
    arm_states_.execute_finished = false;
}

/*
void ArmJointsControllerNode::offline_move_task(const int& index) {

    move_group_interface.setStartStateToCurrentState();

    auto tic = ros::Time::now();
    move_group_interface.execute(my_offline_plan[index]);
    auto toc = ros::Time::now() - tic;
    ROS_INFO("Execute time %.2fms", toc.toSec() * 1000);

    arm_states_.execute_finished = true;
    arm_puber_.publish(arm_states_);
    arm_states_.execute_finished = false;
}
*/