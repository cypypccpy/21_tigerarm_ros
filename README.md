## 简介
- 基于 `ROS` 和 `Moveit` 实现的机械臂控制系统
- __Note__: 每次只取一个，取完不放直接回家；可以往回扔，然后再取下一个

## 环境
- `Ubuntu 18.04`
- `ROS Melodic`
- `OpenCV 4.5.1`
- `yaml-cpp`

## 配置
- 编译工作空间
```bash
cd ${YOUR_WORK_DIR}/arm_ros
catkin_make
source devel/setup.bash
# 或者将配置文件写入终端的配置文件中
# echo "source ${YOUR_WORK_DIR}/devel/setup.bash" >> ~/.bashrc
# 添加用户到tty所在的组
sudo gpasswd --add ${username} dialout
```

## 运行
- 设置轨迹
`src/arm_moveit_kinematics/config/poses.yaml`

```yaml
Total: 2 #总共读入的轨迹点数
#EffectorLength: 0.05
Points: #轨迹点序列
  -
    id: 0 #点的序号
    xyz: #位置
      - 0.47
      - 0
      - 0.1
    rpy: #末端姿态
      - 3.141593
      - 0
      - -1.570793
    action: 1
  -
    id: 1
    xyz: 
      - 0.3
      - 0
      - 0.57
    rpy:
      - 3.141593
      - 0
      - -1.570793
    action: 0
    ...
```

- 启动
```bash
roslaunch pixle_godzilla_moveit_config demo.launch
# 等待直到rviz中画面出现机械臂
roslaunch arm_moveit_kinematics demo.launch
# 注意查看输出信息中节点是否正常启动，串口能不能正常打开和写入
# 出现一个新的终端，在新的终端中通过按键控制
# 注意保持鼠标光标停留在该终端
rqt_plot
# 可以通过rqt查看6个关节的角度变化曲线，跟上位机的曲线对比
```

- 控制机械臂

| 键位 | 功能 |
|:-:|:-:|
|shift+r|手动夹取矿石|
|shift+t|视觉夹取矿石|
|c|准备抓取障碍块模式|
|f|准备抓取矿石模式|
|v|准备兑换矿石模式（通常与space连用）|
|space|抓取放于矿仓的矿石|
|z|抓到矿石后放入矿仓|
|x|扫码兑换矿石|
|t|抓取障碍块|
|r|障碍块放于前端|
|w|x轴前进1cm|
|s|x轴后退1cm|
|a|y轴前进1cm|
|d|y轴后退1cm|
|shift+w|z轴前进1cm|
|shift+s|z轴后退1cm|


## TODO List


- 如何降低plan的延迟

- [x] 【重要】模型继续简化
- [ ] 能否离线plan储存为轨迹点
- [ ] 【重要】使用增量式目标值时不用plan，直接用逆运动学得到joint值并进行连续性检测
- [ ] 【重要】move是阻塞的，但是运动是连贯的，在自动组操作中可以用pipeline并行降低延迟


- 如何提高plan的质量

- [x] 【重要】直接设定轨迹的Cartesian Paths
- [ ] 直接用request给运动空间设置Constrain，避免大量采样
- [ ] 设置合理的tolerance


- 如何提高运动速度

- [x] 适当调节速度因子
- [x] urdf中增大关节速度限制
- [x] 同时记得提高插值速度


- 运动稳定性

- [ ] 把一次插值改为多次插值
