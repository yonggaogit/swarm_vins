#!/bin/bash


DRONE_ID=$1
SERVER_IP=$2
SERVER_PORT=$3
OFFSET_MULTIPLIER=$4

source /opt/ros/noetic/setup.bash;
source ~/JKW_PROJECT/swarm_vins_ws/devel/setup.bash;

roslaunch network_utils drone_node.launch drone_id:=$DRONE_ID server_ip:=$SERVER_IP server_port:=$SERVER_PORT offset_multiplier:=$OFFSET_MULTIPLIER;
