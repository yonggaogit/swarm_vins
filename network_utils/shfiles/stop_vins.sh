#!/bin/bash

# Usage: ./stop_drone.sh <drone_id>

DRONE_ID=$1
source ~/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg

# 固定的节点名列表
NODE_NAMES=(
    "/camera/realsense2_camera"
    "/camera/realsense2_camera_manager"
    "/mavros"
    "/vins_fusion"
)

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
    echo "Stopping ROS nodes on drone $DRONE_ID ($DRONE_IP)..."
    
    # SSH 连接到无人机，逐个关闭固定的节点
    sshpass -p $PASSWORD ssh -o StrictHostKeyChecking=no $USER@$DRONE_IP << EOF
    source /opt/ros/noetic/setup.bash

    # 逐个关闭固定的 ROS 节点
    # 逐个关闭固定的 ROS 节点
    for NODE in "${NODE_NAMES[@]}"
    do
        echo "Killing node \$NODE..."
        rosnode kill \$NODE
    done
EOF

    echo "Specified nodes on drone $DRONE_ID have been stopped."
else
    echo "Invalid or missing IP for drone ID $DRONE_ID"
    exit 1
fi

