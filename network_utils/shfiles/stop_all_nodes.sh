#!/bin/bash

# Usage: ./stop_drone.sh <drone_id>

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

if [ -n "$DRONE_IP" ]; then
    echo "Stopping all ROS nodes on drone $DRONE_ID ($DRONE_IP)..."
    
    # SSH 连接到无人机，获取所有 ROS 节点并逐个关闭，然后重启飞控
    sshpass -p $PASSWORD ssh -o StrictHostKeyChecking=no $USER@$DRONE_IP << EOF
    python3 /home/$USER/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/tmp/stop_all_rosnode.py
    sleep 0.5
    echo "reboot" | /home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/tmp/mavlink_shell.py
EOF
    echo "All ROS nodes on drone $DRONE_ID have been stopped."
else
    echo "Invalid or missing IP for drone ID $DRONE_ID"
    exit 1
fi
