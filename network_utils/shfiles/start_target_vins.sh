#!/bin/bash

# Usage: ./start_target_vins.sh <drone_id>

DRONE_ID=$1
source ~/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg

# 获取无人机 IP 地址
get_ip_by_id() {
    case $1 in
        1) echo $DRONE1_IP ;;
        2) echo $DRONE2_IP ;;
        3) echo $DRONE3_IP ;;
        4) echo $DRONE4_IP ;;
        5) echo $DRONE5_IP ;;
        *) echo "Invalid drone ID"; exit 1 ;;
    esac
}

DRONE_IP=$(get_ip_by_id $DRONE_ID)

sshpass -p $PASSWORD ssh $USER@$DRONE_IP "bash -c '/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/tmp/rspx4.sh' " &
