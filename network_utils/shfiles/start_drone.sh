#!/bin/bash

# Usage: ./start_drone.sh <drone_id> <num_drones> <drone1_ip> <drone2_ip> <drone3_ip> <drone4_ip> <drone5_ip> <base_port> <offset_multiplier> <odom_topic> <vins_topic>

DRONE_ID=$1
NUM_DRONES=$2
DRONE1_IP=$3
DRONE2_IP=$4
DRONE3_IP=$5
DRONE4_IP=$6
DRONE5_IP=$7
BASE_PORT=$8
OFFSET_MULTIPLIER=$9
ODOM_TOPIC=${10}
VINS_TOPIC=${11}

roslaunch network_utils start_drone.launch \
    drone_id:=$DRONE_ID \
    num_drones:=$NUM_DRONES \
    drone1_ip:=$DRONE1_IP \
    drone2_ip:=$DRONE2_IP \
    drone3_ip:=$DRONE3_IP \
    drone4_ip:=$DRONE4_IP \
    drone5_ip:=$DRONE5_IP \
    base_port:=$BASE_PORT \
    offset_multiplier:=$OFFSET_MULTIPLIER \
    odom_topic:=$ODOM_TOPIC \
    vins_topic:=$VINS_TOPIC
