#!/bin/bash
. network_utils/config/drone_config.cfg

DRONE_IPS=($DRONE1_IP $DRONE2_IP $DRONE3_IP $DRONE4_IP $DRONE5_IP)

# 生成补丁文件
git add .
git diff --cached > changes.diff
git reset

scp changes.diff $USER@$DRONE1_IP:/tmp

# 在目标机器上应用补丁
ssh $USER@$DRONE1_IP << 'EOF'
cd /home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins
# 确保工作目录干净
git clean -f -d
git reset --hard HEAD
# 应用补丁
git apply /tmp/changes.diff
cd ../..
. devel/setup.sh
catkin_make
EOF
