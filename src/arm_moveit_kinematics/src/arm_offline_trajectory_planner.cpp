#include <arm_moveit_kinematics/arm_moveit_kinermatics_node.h>

void ArmJointsControllerNode::set_offline_pose() {

    geometry_msgs::PoseStamped g_pose_target_stamp;
    std::vector<std::string> pose_name;

    //抢矿
    pose_name = {"temp_point", "pick_island"};
    set_target_pose(pose_name);
    offline_planning_.g_pose_offline_target_ = g_pose_muilt_target;
    offline_planning_.speed_scale_ = 0.71;
    offline_planning_.joint_begin_position_ = {0, -0.23575, 0.060500, 0, -0.083895, -0.001};
    offline_planning_queue.push_back(offline_planning_);
    std::vector<geometry_msgs::Pose>().swap(g_pose_muilt_target); 

    //抢完矿抬升
    pose_name = {"pick_up", "pre_place", "place"};
    set_target_pose(pose_name);
    offline_planning_.g_pose_offline_target_ = g_pose_muilt_target;
    offline_planning_.speed_scale_ = 0.85;
    offline_planning_.joint_begin_position_ = {0, -0.6729288, 0.6452, 0.0, 0.0636, -0.001};
    offline_planning_queue.push_back(offline_planning_);

    std::vector<geometry_msgs::Pose>().swap(g_pose_muilt_target); 

    //没抢到矿缩手
    pose_name = {"pre_pick_island2"};
    set_target_pose(pose_name);
    offline_planning_.g_pose_offline_target_ = g_pose_muilt_target;
    offline_planning_.speed_scale_ = 0.71;
    offline_planning_.joint_begin_position_ = {0, -0.6729288, 0.6452, 0.0, 0.0636, -0.001};
    offline_planning_queue.push_back(offline_planning_);

    std::vector<geometry_msgs::Pose>().swap(g_pose_muilt_target); 
    
    arm_pose_updated = false;
}

void ArmJointsControllerNode::compute_offline_trajectory() {

    /* Update target position */
    for (int i = 0; i < offline_planning_queue.size(); i++) {
        
        moveit::core::RobotStatePtr offline_start_state(move_group_interface.getCurrentState());
        offline_start_state->setJointGroupPositions("arm", offline_planning_queue[i].joint_begin_position_);
        move_group_interface.setStartState(*offline_start_state);

        moveit::planning_interface::MoveGroupInterface::Plan MuiltPlan;

        speed_scale = offline_planning_queue[i].speed_scale_;
        MuiltPlan.trajectory_ = compute_trajectory(offline_planning_queue[i].g_pose_offline_target_, &move_group_interface);


        if (i == 0) {
            std::vector<double> joint_target_position = {0, -0.400457, 0.274137, 0, -0.0352, -0.001};
            for (int j=1;j<MuiltPlan.trajectory_.joint_trajectory.points.size() - 1;j++) {
                MuiltPlan.trajectory_.joint_trajectory.points[j].positions = joint_target_position;
            }
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
    ROS_INFO("Offline Execute time %.2fms", toc.toSec() * 1000);

    if (air_pump_mode_change) {
        arm_states_.air_pump_close = true;
        arm_states_.execute_finished = true;

        arm_puber_.publish(arm_states_);

        arm_states_.air_pump_close = false;
        air_pump_mode_change = false;
    }
    else {
        arm_states_.execute_finished = true;
        arm_puber_.publish(arm_states_);
    }
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