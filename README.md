# Tigerarm_ros

![banner]()

![badge]()
![badge]()
[![license](https://img.shields.io/github/license/:user/:repo.svg)](LICENSE)
[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

超级机械哥斯拉

## 内容列表

- [背景](#背景)
- [安装](#安装)
- [用法](#用法)
- [如何贡献](#如何贡献)
- [使用许可](#使用许可)

## 背景

基于 `ROS` 和 `Moveit` 实现的机械臂控制系统

## 安装

- `Ubuntu 18.04`
- `ROS Melodic`
- `OpenCV 4.5.1`
- `yaml-cpp`

## 用法

```bash
cd ${YOUR_WORK_DIR}/arm_ros
catkin_make
source devel/setup.bash
# 或者将配置文件写入终端的配置文件中
# echo "source ${YOUR_WORK_DIR}/devel/setup.bash" >> ~/.bashrc
# 添加用户到tty所在的组
sudo gpasswd --add ${username} dialout
```

### 设置轨迹
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

### 启动
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

### 控制机械臂

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

## 如何贡献

See [the contributing file](CONTRIBUTING.md)!

## 使用许可

[MIT © Richard McRichface.](../LICENSE)

