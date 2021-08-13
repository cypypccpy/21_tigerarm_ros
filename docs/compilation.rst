
==========================
编译TigerArm
==========================

系统依赖
===================

TigerArm需要的依赖的ROS发行版本为如下三个版本，笔者已经在Melodic上进行过大量测试。

* Kinetic
* Melodic
* Noetic

.. _ROS distributions: http://wiki.ros.org/Distributions

安装
=======================

首先，创建工作空间，clone源码

.. code-block:: bash

    mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
    git clone https://git.scutbot.cn/ctypchen/21_tigerarm_ros.git
    cd ..

下一步，你需要安装所需依赖。可以使用rosdep直接全部安装，也可以手动安装。这里提供rosdep安装方法，如果已经安装有rosdep，可以忽略前两行

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

最后，通过catkin_make编译代码，由于使用了自定义的消息类型，需要先编译声明了该类型的package

.. code-block:: bash

    catkin_make --pkg arm_moveit_kinematics -j4
    catkin_make -j4


