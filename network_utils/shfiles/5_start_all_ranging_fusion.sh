#!/bin/bash

source ~/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg


DRONE_IPS=($DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP)

start_drone() {
    sshpass -p $PASSWORD ssh $USER@$2 "bash -c '/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/tmp/start_ranging_fusion.sh $1 $NUM_DRONES $UWB_VIO_CONFIG $POSE_GRAPH_PATH_TOPIC $VINS_TOPIC $LINKTRACK_NODEFRAME_TOPIC $IMU_TOPIC' " &
}

# 启动指定数量的无人机
for ((i=1; i<=NUM_DRONES; i++)); do
    start_drone $i ${DRONE_IPS[$i-1]}
done

wait