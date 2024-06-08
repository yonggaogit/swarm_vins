#!/bin/bash

source /home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg

# 无人机 IP 地址数组
DRONE_IPS=($DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP)

start_drone() {
    DRONE_ID=$1
    DRONE_IP=$2
    echo "Starting drone $DRONE_ID at IP $DRONE_IP"
    sshpass -p $PASSWORD ssh $USER@$DRONE_IP "bash -c '/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/start_drone.sh $DRONE_ID $NUM_DRONES $DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP $SERVER_TO_CLIENT_BASE_PORT $OFFSET_MULTIPLIER $ODOM_TOPIC $VINS_TOPIC'" &
}

# 启动指定数量的无人机
for ((i=1; i<=NUM_DRONES; i++)); do
    start_drone $i ${DRONE_IPS[$i-1]}
done

wait