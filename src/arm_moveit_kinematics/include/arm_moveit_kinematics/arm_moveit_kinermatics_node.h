#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <arm_moveit_kinematics/arm_states.h>
#include <arm_moveit_kinematics/joint_plots.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/utils.h>


#include <std_msgs/Int32.h>

#include "arm_moveit_kinematics/deltas.h"
#include "arm_moveit_kinematics/keyboards.h"
#include "arm_moveit_kinematics/objects.h"
#include "arm_moveit_kinematics/action_group.h"
#include <tf/transform_listener.h>

using namespace arm_keys;

struct offline_planning
{
    double speed_scale_;
    std::vector<double> joint_begin_position_;
    std::vector<geometry_msgs::Pose> g_pose_offline_target_;
};

class ArmJointsControllerNode {
    public:
        explicit ArmJointsControllerNode();
        /**
         * @brief 接收深度相机发布的矿石位姿
         */
        void mineral_pose_callback(const geometry_msgs::PoseStamped& pose);
        /**
         * @brief 接收键盘按键信息
         */
        void key_recv_callback(const std_msgs::Int32& msg);
        /**
         * @brief 设置目标点，可一次性设多点
         */
        void set_target_pose(const std::vector<std::string>& target_pose_name);
        /**
         * @brief 打印位姿信息
         */
        void get_pose_info(const std::string& obj, const geometry_msgs::Pose& g_pose);
        /**
         * @brief 稀疏化轨迹点，每隔gap个点只保留首尾
         */
        void delete_trajectory(moveit::planning_interface::MoveGroupInterface::Plan& plan, const unsigned& gap);
        /**
         * @brief 通过目标点计算轨迹点
         */
        moveit_msgs::RobotTrajectory compute_trajectory(const std::vector<geometry_msgs::Pose>& g_pose_muilt_target_,
                                                       moveit::planning_interface::MoveGroupInterface *move_group_interface);
        
        /**
         * @brief 调整轨迹点经过的时间，实现间接性调整速度
         */
        moveit_msgs::RobotTrajectory finetune_time(moveit_msgs::RobotTrajectory trajectory_, const double& scale);
        /**
         * @brief 设置离线规划目标点
         */
        void set_offline_pose();
        /**
         * @brief 离线计算轨迹点
         */
        void compute_offline_trajectory();
        /**
         * @brief 离线规划执行函数
         */
        void offline_move_task(const int& index);
        /**
         * @brief 机械臂路径规划核心函数，执行规划任务
         */
        void move_task();

    private:
        const std::string PLANNING_GROUP = "arm";
        int g_total_obj_cnt;
        double speed_scale = 0.85;

        /* Global pointer to get delta pose */
        arm_pose_delta g_pose_delta;
        geometry_msgs::Pose g_pose_current, g_pose_target;
        geometry_msgs::PoseStamped mineral_pose;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        //muilt task
        std::vector<geometry_msgs::Pose> g_pose_muilt_target;
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> my_muilt_plan;

        //offline_muilt_task
        std::vector<moveit_msgs::RobotTrajectory> offline_trajectory;
        offline_planning offline_planning_;
        std::vector<offline_planning> offline_planning_queue;
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> my_offline_plan;

        moveit_msgs::RobotTrajectory trajectory;
        moveit_msgs::RobotTrajectory trajectory_actul;

        point_struct g_next_point;
        bool arm_pose_updated;
        bool air_pump_mode_change;
        bool success;

        //static mineObject mine_obj;
        //static islandObject island_obj;

        pointsLoader pl;

        ros::NodeHandle node_handle_;
        ros::Subscriber key_suber_;
        ros::Subscriber mineral_suber_;
        ros::Publisher arm_puber_;
        ros::Publisher key_msg_puber;

        arm_moveit_kinematics::arm_states arm_states_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface move_group_interface;

};