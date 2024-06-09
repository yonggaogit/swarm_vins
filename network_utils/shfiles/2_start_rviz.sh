#!/bin/bash

source ~/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg


source /opt/ros/noetic/setup.bash;
source ~/JKW_PROJECT/swarm_vins_ws/devel/setup.bash;

roslaunch network_utils server_node.launch server_ip:=$SERVER_IP server_port:=CLIENT_TO_SERVER_PORT rviz_config:=RVIZ_CONFIG;