#!/bin/bash

# Usage: ./start_drone.sh <drone_id> <num_drones> <drone1_ip> <drone2_ip> <drone3_ip> <drone4_ip> <drone5_ip> <base_port> <offset_multiplier> <odom_topic> <vins_topic>

DRONE_ID=$1
NUM_DRONES=$2
UWB_VIO_CONFIG=$3
POSE_GRAPH_PATH_TOPIC=$4
SELF_ODOM_TOPIC=$5
LINKTRACK_NODEFRAME_TOPIC=$6
IMU_TOPIC=$7

source /opt/ros/noetic/setup.bash;
source /home/coolas/JKW_PROJECT/swarm_vins_ws/devel/setup.bash;

echo $DRONE_ID
echo $NUM_DRONES
echo $UWB_VIO_CONFIG
echo $POSE_GRAPH_PATH_TOPIC
echo $SELF_ODOM_TOPIC
echo $LINKTRACK_NODEFRAME_TOPIC
echo $IMU_TOPIC

roslaunch ranging_fusion ranging_fusion.launch drone_id:=$DRONE_ID num_drones:=$NUM_DRONES uwb_vio_config:=$UWB_VIO_CONFIG pose_graph_path_topic:=$POSE_GRAPH_PATH_TOPIC self_odom_topic:=$SELF_ODOM_TOPIC linktrack_nodeframe_topic:=$LINKTRACK_NODEFRAME_TOPIC imu_topic:=$IMU_TOPIC
