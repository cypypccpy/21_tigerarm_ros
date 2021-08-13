======================================
在实际机器人上运行TigerArm
======================================

在真实的机器人上运行TigerArm需要Linux主机与单片机通过串口进行通讯，进行数据的传输。

上位机通过Moveit!计算出六个关节的实际角度值，再发与下位机进行运动的插补与控制。这是一个仿真驱动的过程，发送的关节角度应与Rviz中模拟的机械臂相同。

.. warning:: When you want to run tigerarm, you might need to source your ROS environment by running ``source devel/setup.bash`` first (replace bash with zsh if your shell is zsh)

启动Rviz与机器人环境
======================================

这一步与仿真类似

.. code-block:: bash

    roslaunch pixle_godzilla_moveit_config demo.launch

等待直到rviz中画面出现机械臂

打开TigerArm控制节点
========================================

检查并打开控制节点，在src/arm_moveit_kinematics/launch/demo.launch中取消注释下行:

.. code-block:: xml

    <node name="arm_joints_controller_node" pkg="arm_moveit_kinematics" type="arm_joints_controller_node" respawn="false" output="screen">
    </node>

.. note:: 注意查看输出信息中节点是否正常启动，串口能不能正常打开和写入

然后运行demo

.. code-block:: bash

    roslaunch arm_moveit_kinematics demo.launch

接着会出现一个新的终端，在新的终端中通过按键控制，注意保持鼠标光标停留在该终端

此时真实的机械臂应与仿真有着相同的动作

.. code-block:: bash

    rqt_plot

可以通过rqt查看6个关节的角度变化曲线，跟上位机的曲线对比
