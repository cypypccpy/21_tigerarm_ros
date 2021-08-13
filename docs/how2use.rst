======================================
如何在TigerArm中添加新的动作
======================================

在TigerArm中，一个按键对应于一个动作。当我们想要创建一个新动作的时候，需要先设定好他要经过的轨迹点，然后在MoveIt!中将其添加到动作组中，并与键盘上的一个按键对应起来

设置动作经过的轨迹点
===============================

我们是通过xyzrpy六个参数来表示一个点的位置，坐标原点位于机器人底盘中心

src/arm_moveit_kinematics/config/poses.yaml

.. code-block:: xml

    Total: 2 #总共读入的轨迹点数
    #EffectorLength: 0.05
    Points: #轨迹点序列
  -
    tag: pre_place #准备放入矿仓
    id: 0
    xyz:
      - -0.14
      - -0.01
      - 0.67
    rpy: 
      - 2.0601
      - -0.000
      - 1.6252
    action: 1
  -
    tag: pre_pick_island #准备抓取矿石(原姿态，现已废弃)
    id: 1
    xyz:
      - 0.52
      - -0.00
      - 0.38
    rpy:
      - 2.0089
      - -0.0044
      - -1.5773
    action: 1
    ...

注册新的键盘按键
===============================

打开src/arm_moveit_kinematics/include/arm_moveit_kinematics/keyboards.h，在enum中添加新的键盘对应数值进行按键注册

.. code-block:: cpp

    namespace arm_keys{
        enum kv{
            _w = 119,
            ...
            _o = 111
        };
    }

将动作加入Moveit!动作组中
===========================================

打开src/arm_moveit_kinematics/src/arm_moveit_kinematics_node.cpp，找到如下函数

.. code-block:: cpp

    void ArmJointsControllerNode::key_recv_callback(const std_msgs::Int32& msg) {
        
        ROS_INFO_NAMED("arm_log", "recieved keyboard msgs: %d", msg.data);

        tf::TransformListener listener;
        geometry_msgs::PoseStamped g_pose_target_stamp;

        ...

        std::vector<std::string>().swap(target_pose_name);  
    }

然后仿照如下格式加入到分支中，需要修改kv::后面的值与target_pose_name

.. code-block:: cpp

    case kv::_space: //拿出矿仓中的矿石进入准备兑换模式
        target_pose_name = {"pre_place", "pick_mineral", "pre_place", "pre_exchange"};
        set_target_pose(target_pose_name);
        break;

.. note:: 里面的target_pose_name即是在yaml中设定的轨迹点的tag，Moveit!将会规划出一条按给定顺序经过相应点的路径出来

然后重新编译，即可完成添加一个新的动作