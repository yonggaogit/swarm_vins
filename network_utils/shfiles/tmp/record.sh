ROS_BAG_NAME=$1

source /opt/ros/noetic/setup.bash;
source ~/JKW_PROJECT/Fast-Drone-250/devel/setup.bash;

rosbag record --tcpnodelay \
/drone_0_ego_planner_node/goal_point \
/ego_planner_node/global_list \
/drone_0_ego_planner_node/optimal_list \
/ego_planner_node/a_star_list \
/drone_0_ego_planner_node/init_list \
/drone_0_odom_visualization/path \
/drone_0_ego_planner_node/grid_map/occupancy_inflate \
/drone_0_odom_visualization/robot \
/vins_fusion/path \
/vins_fusion/odometry \
/vins_fusion/camera_pose \
/vins_fusion/camera_pose_visual \
/vins_fusion/imu_propagate \
/camera/infra1/image_rect_raw \
/camera/infra2/image_rect_raw \
/nlink_linktrack_nodeframe2 \
/ranging_fusion/global_odometry \
/ranging_fusion/global_path \
/mavros/imu/data_raw \
/drone_1/vins_fusion/imu_propagate \
/drone_2/vins_fusion/imu_propagate \
/position_cmd -o /home/coolas/JKW_PROJECT/$ROS_BAG_NAME