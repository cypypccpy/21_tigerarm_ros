======================================
如何对TigerArm进行优化
======================================

这一章，是我在TigerArm上做的工程量最大的优化，也是我在这个项目中遇到的最大的问题，直到现在，也不能说完全解决了，但我认为我还是给出了一个不错的方案，可以供大家参考。

他的需求来源很简单:当一个动作需要连续经过多个路径点时，简单地每段路plan一遍会造成在路径点附近反复做加减速的过程，运动就不连贯了。
我们首要做的就是避免这件事，使动作连贯起来。然后考虑执行速度的问题，最后则尝试离线规划，使运动之间真正做到没有停顿。

多路径点规划问题
========================

这个问题曾经困扰了我两三天，我尝试过各种不同的方法，翻阅过move_group(MoveIt!c++Interface)所有的接口，网上大量的博客，都没有找到有效的解决方案。

最后，我找到了一个笨笨的解决方法:先将每个路径点间的Trajectory给Plan出来，再将他们首尾相连，然后把这些路径点当作约束再进行一次Plan，就能做到没有停顿地执行动作

.. code-block:: cpp

    moveit_msgs::RobotTrajectory ArmJointsControllerNode::compute_trajectory(const std::vector<geometry_msgs::Pose>& g_pose_muilt_target_,
                                                                            moveit::planning_interface::MoveGroupInterface* move_group_interface) {

        //通过目标点计算轨迹
        for (int i = 0; i < g_pose_muilt_target_.size(); i++) {
            move_group_interface->setPoseTarget(g_pose_muilt_target_[i]);

            success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            my_muilt_plan.push_back(my_plan);
        
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
        ...
        return trajectory_actul;
    }

.. note:: 过程虽然曲折了点，但其实运算并不耗时，只相当于原本Plan的时间x2，而Plan的时间本身就很短

时间参数化
===========================

在上面代码的省略号中，有一个重要的东西:时间参数化  

MoveIt目前主要是一个kinematic运动规划框架-主要用来规划机器人各关节或末端执行器的位置，但没有涉及到机器人的速度和加速度限制。
因此，MoveIt可以利用后处理时间参数化运动轨迹的速度和加速度值。

在TigerArm中，我们也使用了iptp(Iterative Parabolic Time Parameterization)算法对最终轨迹进行后处理

.. code-block:: cpp

    //iptp算法
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, 1.0, 1.0);
    
    rt.getRobotTrajectoryMsg(trajectory_actul);

理论上我们因此计算出了速度与加速度后，可以发与下位机辅助插补，可以得到更好的运动控制，至于为什么没有，当然是因为一直没时间啦~

修改执行速度
=======================

对于不同的运动而言，需要不同的执行速度，这个问题曾经也困扰过我一段时间。理论上，应该是从算法调用的层面上进行优化，但MoveIt!与ompl中的规划算法是通过插件的形式整合在一起的，
我在ompl的配置文件上又没找到任何与规划的速度有关的参数。

在快要gg思密达~的时候，居然给我找到个办法:通过调整每个轨迹点的time_from_start属性，就能在Execute的时候加快或减慢运动，
于是我做了如下处理:

.. code-block:: cpp

    moveit_msgs::RobotTrajectory ArmJointsControllerNode::finetune_time(moveit_msgs::RobotTrajectory trajectory_, const double& scale = 0.85) {
        //std::cout << "speed_scale: " << scale << std::endl;
        for (int j=0;j<trajectory_.joint_trajectory.points.size();j++) {
            for (int i=0;i<trajectory_.joint_trajectory.points[j].positions.size();i++) {
                trajectory_.joint_trajectory.points[j].time_from_start.operator*=(scale);
            }
        }
        return trajectory_;
    }
    ...
    //通过time_from_start 调整速度            
    trajectory_actul = ArmJointsControllerNode::finetune_time(trajectory_actul, speed_scale);
    ...

然后在每条轨迹规划前给予一个速度scale，达到控制速度的作用

.. code-block:: cpp

    ...
    case kv::_u: //兑换完矿石后推入矿槽
        speed_scale = 0.8;
        target_pose_name = {"push"};
        set_target_pose(target_pose_name);
        break;
    ...

.. warning:: 在实机时speed_scale的设置应比较小心，当速度太快时会造成路径收缩，有危险

这么做虽然很蠢，但确实能用，能满足大部分对速度的需求。这也是可以说还未解决的地方，希望有志之士未来能补上这块空缺，用真正科学合理方法控制速度

离线规划
=========================

src/arm_moveit_kinematics/src/arm_offline_trajectory_planner.cpp

离线规划我单独在一个cpp文件里写着，虽然看上去和在线规划差别不大但其实比较复杂且需要具体理解MoveIt!的工作机制，整理起来比较麻烦，加上现在笔者写到这里有点累了，等下次有空在更新吧~


