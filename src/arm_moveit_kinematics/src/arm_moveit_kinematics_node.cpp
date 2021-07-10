#include <moveit/planning_pipeline/planning_pipeline.h>
#include "arm_moveit_kinematics/arm_moveit_kinermatics_node.h"

using namespace arm_keys;

ArmJointsControllerNode::ArmJointsControllerNode(): arm_pose_updated(false),
                                                    air_pump_mode_change(false),
                                                    g_total_obj_cnt(0),
                                                    move_group_interface(PLANNING_GROUP) {
    key_suber_ = node_handle_.subscribe("arm_keys", 100, &ArmJointsControllerNode::key_recv_callback, this);
    mineral_suber_ = node_handle_.subscribe("mineral_detect", 100, &ArmJointsControllerNode::mineral_pose_callback, this);
    arm_puber_ = node_handle_.advertise<arm_moveit_kinematics::arm_states>("arm_states", 100);

    /* Load poses and action group */
    std::string pose_yaml_path, stl_resource_path;
    ros::param::get("~poses_yaml", pose_yaml_path);
    ros::param::get("~stl_path", stl_resource_path);
    ROS_INFO_NAMED("arm_log", "Load poses list from yaml file %s", pose_yaml_path.c_str());
    ROS_INFO_NAMED("arm_log", "Load mesh shape from stl file %s", stl_resource_path.c_str());

    //mine_obj.setup(&planning_scene_interface, &move_group_interface);
    //island_obj.setup(&planning_scene_interface, &move_group_interface, stl_resource_path);

    pl.load(pose_yaml_path);
    
    // Please sleep a while before adding objects
    //mine_obj.add("base_link", object_type::MINERAL);
    //island_obj.add("base_link", object_type::ISLAND_BIG);
    
    //pipeline----------------很难搞啊
    /*
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");  //创建modelloader加载对象
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();        //将从ROS服务器中获取的模型，装载到robot_model内
 
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "planning_plugin", "request_adapters"));

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    */
    //离线计算trajectory
}


void ArmJointsControllerNode::mineral_pose_callback(const geometry_msgs::PoseStamped& pose) {
    mineral_pose = pose;
}

void ArmJointsControllerNode::key_recv_callback(const std_msgs::Int32& msg)
{
    ROS_INFO_NAMED("arm_log", "recieved keyboard msgs: %d", msg.data);

    tf::TransformListener listener;
    geometry_msgs::PoseStamped g_pose_target_stamp;
    std::vector<std::string> target_pose_name;

    /* clear last */
    g_pose_delta.clear();

    switch(msg.data){
        case kv::_w:
            g_pose_delta.dx = DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
            break;
        case kv::_s:
            g_pose_delta.dx = -DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
            break;
        case kv::_a:
            g_pose_delta.dy = DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
            break;
        case kv::_d:
            g_pose_delta.dy = -DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
        case kv::_W:
            g_pose_delta.dz = DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
            break;
        case kv::_S:
            g_pose_delta.dz = -DP;
            g_pose_target = g_pose_delta.add(g_pose_target);
            g_pose_muilt_target.push_back(g_pose_target);
            arm_pose_updated = true;
            break;
        case kv::_z: //抓到后放入矿仓一条龙
            target_pose_name = {"pick_up", "pre_place", "place"};

            air_pump_mode_change = true;
            if (std::abs(move_group_interface.getCurrentJointValues()[1] - (offline_planning_queue[1].joint_begin_position_[1])) < 0.02) {
                offline_move_task(1);
            }
            else {
                set_target_pose(target_pose_name);
            }
            
            break;
        case kv::_c: //准备抓取障碍块
            target_pose_name = {"pre_pick_block"};
            set_target_pose(target_pose_name);
            break;
        case kv::_space: //拿出矿仓中的矿石进入准备兑换模式
            target_pose_name = {"pre_place", "pick_mineral", "pre_place", "pre_exchange"};
            set_target_pose(target_pose_name);
            break;
        case kv::_x: //兑换矿石
            air_pump_mode_change = true;
            target_pose_name = {"exchange"};
            set_target_pose(target_pose_name);
            break;
        case kv::_v: //矿仓中已有矿石时抓取矿石但不入库
            target_pose_name = {"pick_up", "pre_exchange"};
            set_target_pose(target_pose_name);
            break;
        case kv::_t: //抓取障碍块并放于前端
            target_pose_name = {"pick_block", "pick_front2"};
            set_target_pose(target_pose_name);
            break;
        case kv::_r: //开车模式
            target_pose_name = {"drive"};
            set_target_pose(target_pose_name);
            break;
        case kv::_q: //准备抓取矿石姿态
            target_pose_name = {"pre_pick_island2"};

            if (std::abs(move_group_interface.getCurrentJointValues()[1] - (offline_planning_queue[1].joint_begin_position_[1])) < 0.02) {
                offline_move_task(2);
            }
            else {
                set_target_pose(target_pose_name);
            }

            break;
        case kv::_T: //视觉抓取矿石
            //------------------------test-------------------------------
            
            //监听两个坐标之间的变换关系
            listener.waitForTransform("base_link","realsense",ros::Time(0),ros::Duration(3));//ros::Time(0)表示使用缓冲中最新的tf数据
            listener.transformPose("base_link", mineral_pose, g_pose_target_stamp);//将realsense中的点变换到yaw_spindle中去

            
            get_pose_info("realsense origin----->", mineral_pose.pose);
            get_pose_info("realsense after converted----->", g_pose_target_stamp.pose);
            //不超过最大阈值
            if (g_pose_target_stamp.pose.position.z > 0.56) {
                g_pose_target_stamp.pose.position.z = 0.56;
            }

            if (g_pose_target_stamp.pose.position.x > 0.51) {
                g_pose_target_stamp.pose.position.x = 0.51;
            }

            g_pose_muilt_target.push_back(g_pose_target_stamp.pose);
            g_pose_target = g_pose_target_stamp.pose;         
            arm_pose_updated = true;
            ROS_INFO_NAMED("arm_log", "Select place poisition");
            break;
        case kv::_R: //手动抓取矿石
            
            target_pose_name = {"pick_island"};
            offline_move_task(0);
            
            //set_target_pose(target_pose_name);
            //----------------------准备修复上抬-------------------------
            
            break;
        case kv::_f: //准备抓取银矿石
            target_pose_name = {"silver"};
            set_target_pose(target_pose_name);
            break;
        case kv::_j: //矿石位姿不对转一转后兑换
            air_pump_mode_change = true;
            target_pose_name = {"pre_exchange2", "exchange2"};
            set_target_pose(target_pose_name);
            break;
        case kv::_k: //挥一拳
            target_pose_name = {"boxing_right"};
            set_target_pose(target_pose_name);
            break;
        case kv::_l: //放矿回矿仓后再次进入准备夹矿模式(防止碰到资源岛)
            target_pose_name = {"zero_point", "pre_pick_island2"};
            set_target_pose(target_pose_name);
            break;
        case kv::_e: //抓完银矿后的抬升
            target_pose_name = {"silver_up"};
            set_target_pose(target_pose_name);
            break;
        case kv::_u: //兑换完矿石后推入矿槽
            speed_scale = 0.8;
            target_pose_name = {"push"};
            set_target_pose(target_pose_name);
            break;
        default:
            g_pose_delta.clear();
            break;
    }
    std::vector<std::string>().swap(target_pose_name);  
}

moveit_msgs::RobotTrajectory ArmJointsControllerNode::compute_trajectory(const std::vector<geometry_msgs::Pose>& g_pose_muilt_target_,
                                                                        moveit::planning_interface::MoveGroupInterface* move_group_interface) {
    auto tic = ros::Time::now();
    
    //通过目标点计算轨迹
    for (int i = 0; i < g_pose_muilt_target_.size(); i++) {
        move_group_interface->setPoseTarget(g_pose_muilt_target_[i]);
        success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        my_muilt_plan.push_back(my_plan);
        
        /*
        std::cout << "-----------------轨迹点属性------------------------" << std::endl;        
        for (int j=0;j<my_plan.trajectory_.joint_trajectory.points.size();j++) {
        std::cout << "-----------------one point------------------------" << std::endl;        
            for (int i=0;i<my_plan.trajectory_.joint_trajectory.points[j].positions.size();i++)
                std::cout << my_plan.trajectory_.joint_trajectory.points[j].positions[i] <<std::endl;
        }
        */
    
        my_plan.start_state_.joint_state.position = my_plan.trajectory_.joint_trajectory.points.back().positions;
        move_group_interface->setStartState(my_plan.start_state_);

    }
    
    //连接轨迹
    trajectory.joint_trajectory.joint_names = my_muilt_plan[0].trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = my_muilt_plan[0].trajectory_.joint_trajectory.points;
    for (int i = 1; i < my_muilt_plan.size(); i++) {
        for (size_t j = 1; j < my_muilt_plan[i].trajectory_.joint_trajectory.points.size(); j++) {
            trajectory.joint_trajectory.points.push_back(my_muilt_plan[i].trajectory_.joint_trajectory.points[j]);
        }
    }
    //重规划
    trajectory_actul = trajectory;
    robot_trajectory::RobotTrajectory rt(move_group_interface->getCurrentState()->getRobotModel(), move_group_interface->getName());
    rt.setRobotTrajectoryMsg(*move_group_interface->getCurrentState(), trajectory_actul);
    //iptp算法
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, 1.0, 1.0);
    
    rt.getRobotTrajectoryMsg(trajectory_actul);

    //通过time_from_start 调整速度            
    trajectory_actul = ArmJointsControllerNode::finetune_time(trajectory_actul, speed_scale);
    speed_scale = 0.85;

    auto toc = ros::Time::now() - tic;
    ROS_INFO_NAMED("arm_log", "Plan %s in time %.2fms", success ? "SUCCEED" : "FAILED", toc.toSec() * 1000);

    return trajectory_actul;
}

void ArmJointsControllerNode::move_task() {
    
    const moveit::core::JointModelGroup* joint_model_group = \
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    ros::Rate task_rate(100); // 100Hz
    while(ros::ok()){
        g_pose_current = move_group_interface.getCurrentPose("yaw_spindle").pose;
        moveit::core::RobotStatePtr start_state(move_group_interface.getCurrentState());
        move_group_interface.setStartStateToCurrentState();

        /* Update target position */
        if(arm_pose_updated){
            arm_pose_updated = false;

            moveit::planning_interface::MoveGroupInterface::Plan MuiltPlan;

            MuiltPlan.trajectory_ = compute_trajectory(g_pose_muilt_target, &move_group_interface);

            get_pose_info("Target", g_pose_target);
            get_pose_info("Currect", g_pose_current);

            /* Do execution */
            if(success) {
                auto tic = ros::Time::now();
                move_group_interface.execute(MuiltPlan);
                auto toc = ros::Time::now() - tic;
                ROS_INFO("Execute time %.2fms", toc.toSec() * 1000);

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
                    
            else 
                g_pose_target = g_pose_current;

            std::vector<geometry_msgs::Pose>().swap(g_pose_muilt_target);  
            std::vector<moveit::planning_interface::MoveGroupInterface::Plan>().swap(my_muilt_plan);  

            /* Operation */
            switch(g_next_point.action_t){
                case PICKUP:
                    //mine_obj.pickup();
                    break;
                case PUTDOWN:
                    //mine_obj.putdown();
                    break;
                default:
                    break;
            }
        }
        else{
            g_pose_target = g_pose_current;
        }
        ros::spinOnce();
        task_rate.sleep();
    }
}

//--------------------------------tools-------------------------------------------

void ArmJointsControllerNode::set_target_pose(const std::vector<std::string>& target_pose_name) {
    for(int i = 0; i < target_pose_name.size(); i++) {
        g_next_point = pl[target_pose_name[i]];
        g_pose_muilt_target.push_back(g_next_point.pose);
    }
    g_pose_target = g_next_point.pose;
    arm_pose_updated = true;
    ROS_INFO_NAMED("arm_log", "Select rest poisition");
}

void ArmJointsControllerNode::get_pose_info(const std::string& obj, const geometry_msgs::Pose& g_pose) {
    double r,p,y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(g_pose.orientation, q);
    tf::Matrix3x3(q).getRPY(r, p, y);
    ROS_INFO_NAMED("arm_log", "%s xyz[%.2f,%.2f,%.2f] q[%.2f,%.2f,%.2f,%.2f] a[%.4f,%.4f,%.4f]",
        obj.c_str(),
        g_pose.position.x, g_pose.position.y, g_pose.position.z,
        g_pose.orientation.w, g_pose.orientation.x,
        g_pose.orientation.y, g_pose.orientation.z,
        r,p,y);
}

void ArmJointsControllerNode::delete_trajectory(moveit::planning_interface::MoveGroupInterface::Plan& plan, const unsigned& gap) {
    unsigned count = 0;
    auto i = plan.trajectory_.joint_trajectory.points.begin();
    while(i < plan.trajectory_.joint_trajectory.points.end())
    {
        //每gap个元素删除一次，保留首尾
        if(count % gap == 0 && count != 0)
        {
            //尾元素保留
            if(i == plan.trajectory_.joint_trajectory.points.end() - 1)
                break;
            i = plan.trajectory_.joint_trajectory.points.erase(i);
        }
        else
            i++;
        count++;
    }
}

moveit_msgs::RobotTrajectory ArmJointsControllerNode::finetune_time(moveit_msgs::RobotTrajectory trajectory_, const double& scale = 0.85) {
    //std::cout << "speed_scale: " << scale << std::endl;
    for (int j=0;j<trajectory_.joint_trajectory.points.size();j++) {
        for (int i=0;i<trajectory_.joint_trajectory.points[j].positions.size();i++) {
            trajectory_.joint_trajectory.points[j].time_from_start.operator*=(scale);
        }
    }
    return trajectory_;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "godzilla");
    ros::NodeHandle node_handle;
    ArmJointsControllerNode arm_joints_controller_node;

    /* Multi threading */
    ros::AsyncSpinner spinner(4); // four thread
    spinner.start();
    ros::Duration sleep_t(1.0);
    sleep_t.sleep();
    arm_joints_controller_node.set_offline_pose();
    arm_joints_controller_node.compute_offline_trajectory();
    arm_joints_controller_node.move_task();
    ros::waitForShutdown();
    return 0;
}