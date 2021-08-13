======================================
在仿真环境中运行TigerArm
======================================

当你没有连接真实机器人时，可以选择在仿真中进行调试。说是仿真，其实只是使用Rviz进行虚拟机器人的可视化来达到模拟的作用，并没有使用到真实物理引擎如gazebo等，因此只能在运动学的层面上进行算法验证。

launch文件夹将会自动打开 ``roscore`` 和 ``rviz`` 。

.. warning:: When you want to run tigerarm, you might need to source your ROS environment by running ``source devel/setup.bash`` first (replace bash with zsh if your shell is zsh)

启动Rviz与机器人环境
=============================================

.. code-block:: bash

    roslaunch pixle_godzilla_moveit_config demo.launch


等待直到rviz中画面出现机械臂

打开TigerArm控制节点
============================================

首先关闭控制节点，在src/arm_moveit_kinematics/launch/demo.launch中注释下行:

.. code-block:: xml

    <!--node name="arm_joints_controller_node" pkg="arm_moveit_kinematics" type="arm_joints_controller_node" respawn="false" output="screen">
    </node-->

.. note:: 当节点检测不到串口时，控制节点会不断报错以致污染命令行。在实际调试时也可以通过关闭控制节点来暂时停止对机械臂的控制

然后运行demo

.. code-block:: bash

    roslaunch arm_moveit_kinematics demo.launch

接着会出现一个新的终端，在新的终端中通过按键控制，注意保持鼠标光标停留在该终端