#!/bin/bash

source ~/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg


DRONE_IPS=($DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP)




start_drone() {
    echo $2
    sshpass -p $PASSWORD ssh $USER@$2 "bash -c '~/JKW_PROJECT/Fast-Drone-250/shfiles/rspx4.sh' " &
}

# 启动指定数量的无人机
for ((i=1; i<=NUM_DRONES; i++)); do
    start_drone $i ${DRONE_IPS[$i-1]}
done

wait