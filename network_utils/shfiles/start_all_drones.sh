#!/bin/bash

# 配置无人机 IP 地址、用户、密码等信息
DRONE1_IP="192.168.7.104"
DRONE2_IP="192.168.7.202"
DRONE3_IP="192.168.1.3"
DRONE4_IP="192.168.1.4"
DRONE5_IP="192.168.1.5"
USER="coolas"
PASSWORD="504"
NUM_DRONES=2
BASE_PORT=5555
OFFSET_MULTIPLIER=1.0
ODOM_TOPIC="/vins_fusion/imu_propagate"
VINS_TOPIC="/vins_fusion/imu_propagate"

start_drone() {
    DRONE_ID=$1
    sshpass -p $PASSWORD ssh $USER@$2 "bash -c '/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/start_drone.sh $DRONE_ID $NUM_DRONES $DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP $BASE_PORT $OFFSET_MULTIPLIER $ODOM_TOPIC $VINS_TOPIC'" &
}

# 启动无人机
start_drone 1 $DRONE1_IP
start_drone 2 $DRONE2_IP
# start_drone 3 $DRONE3_IP
# start_drone 4 $DRONE4_IP
# start_drone 5 $DRONE5_IP

wait