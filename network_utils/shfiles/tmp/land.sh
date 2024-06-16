source /opt/ros/noetic/setup.bash;
source ~/JKW_PROJECT/Fast-Drone-250/devel/setup.bash;
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
