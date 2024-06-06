# 0 项目说明
![](./img/demo.png)
# 1 安装配置
- 环境说明：`Ubuntu20.04`、`ROS1.0 Noetic` 
- 安装配置
```shell
mkdir -p ~/swarm_vins_ws/src
git clone https://github.com/yonggaogit/swarm_vins.git
cd ~/swarm_vins_ws/
catkin_make
```
# 2 模块说明
## 2.1 network_utils
- 用途：通过`ZeroMQ`将每个无人机自身由`VINS`估计的位姿话题通过`TCP`网络发送到其它无人机上，其它无人机在接收到后再通过话题的机制在本机发送出来
    - 由于无人机的初始位姿并不是相同的，而由`VINS`估计的单机位姿又是以自身运动起点为原点的位姿，因此在真实飞行时需要按照固定的间隔摆放无人机，在`network_utils/launch/all_start.launch`文件中通过`offset_multiplier`来指明这个摆放间隔。从y轴的方向放置。
- 通过`network_utils/launch/all_start.launch`来启动
    - `drone{i}_ip`表示的是三架无人机的ip地址，`user`和`password`则是登录主机的用户名和ip地址、`env_loader`是程序在机载电脑上的环境变量的位置。需要设置`ROS_MASTER`等来配置`ROS`的多机通信。`num_drones`指示总的无人机数量，`base_ip`指示的是第一架无人机的ip地址，期望是从1开始后面几架无人机逐步递增的。`base_port`指示通信端口。`offset_multiplier`指示摆放无人机时的固定间隔
    ```launch
    <arg name="drone1_ip" default="192.168.1.1"/>
    <arg name="drone2_ip" default="192.168.1.2"/>
    <arg name="drone3_ip" default="192.168.1.3"/>
    <arg name="user" default="user"/>
    <arg name="password" default="password"/>
    <arg name="env_loader" default="/path/to/env/setup.bash"/>
    <arg name="num_drones" default="3"/>
    <arg name="base_ip" default="192.168.1."/>
    <arg name="base_port" default="5555"/>
    <arg name="offset_multiplier" default="1.5"/>
    ```
## 2.2 ranging_fusion
- 订阅`VINS_FUSION`发布的位姿估计话题，同时接收其它无人机的位姿数据以及UWB测距数据，利用测距优化原始位姿。
- 通过`ranging_fusion/launch/ranging_fusion.launch`启动
    - `pose_graph_path_topic`、`self_odom_topic`、`linktrack_nodeframe_topic`及`imu_topic`为对应的路径、里程计、UWB测距及IMU话题，需要确保修改正确
    ```launch
    <param name="drone_id" value="1" />
    <param name="num_drones" value="4" />

    <param name="pose_graph_path_topic" value="/pose_graph/path_1"/>
    <param name="self_odom_topic" value="/vins_fusion/imu_propagate"/>
    <param name="linktrack_nodeframe_topic" value="/nlink_linktrack_nodeframe2"/>
    <param name="imu_topic" value="/mavros/imu/data_raw"/>
    ```
## 2.3 nlink_parser
- linktrack UWB的官方功能包，但进行了一定的修改，不要用官方自己的，主要修改在其自定义的msg中加入header头信息
- 通过`nlink_parser/launch/linktrack.launch`启动。

## 2.4 VINS_FUSION
- vins视觉里程计位姿估计模块，具体原理用法参考官方例程[VINS_FUSION](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- 通过`VINS-Fusion/vins_estimator/launch/fast_drone_250.launch`启动，主要需要修改的参数在`VINS-Fusion/config/fast_drone_250.yaml`中，包括相机和IMU的话题及机体系到相机系的位姿变换
# 3 使用说明
```shell
sudo chmod +x 
echo "source ~/JKW_PROJECT/swarm_vins_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```