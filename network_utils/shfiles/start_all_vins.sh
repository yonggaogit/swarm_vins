#!/bin/bash

# 配置无人机 IP 地址、用户、密码等信息
DRONE1_IP="192.168.7.104"
DRONE2_IP="192.168.7.202"
DRONE3_IP="192.168.1.3"
DRONE4_IP="192.168.1.4"
DRONE5_IP="192.168.1.5"
USER="coolas"
PASSWORD="504"



start_drone() {
    echo $2
    sshpass -p $PASSWORD ssh $USER@$2 "bash -c '~/JKW_PROJECT/Fast-Drone-250/shfiles/rspx4.sh' " &
}

# export -f start_drone
# export USER PASSWORD DRONE1_IP DRONE2_IP DRONE3_IP DRONE4_IP DRONE5
# 并行启动无人机
# parallel start_drone ::: 1 2 ::: $DRONE1_IP $DRONE2_IP

# 启动无人机
start_drone 1 $DRONE1_IP
start_drone 2 $DRONE2_IP
# start_drone 3 $DRONE3_IP
# start_drone 4 $DRONE4_IP
# start_drone 5 $DRONE5_IP

wait