#!/usr/bin/env python3
import signal
from runall_vins import launch_process, stop_process, cleanup
import os
import subprocess
import atexit
import configparser

# 配置文件
config_file = '/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/config/drone_config.cfg'
# 手动添加一个默认的节标题
config_content = '[DEFAULT]\n' + open(config_file).read()
# 创建 ConfigParser 对象
config = configparser.ConfigParser()
# 读取配置文件
config.read_string(config_content)
# 获取各个变量的值
num_drones = config.getint('DEFAULT', 'NUM_DRONES')
server_ip = config.get('DEFAULT', 'SERVER_IP')
drone1_ip = config.get('DEFAULT', 'DRONE1_IP')
drone2_ip = config.get('DEFAULT', 'DRONE2_IP')
drone3_ip = config.get('DEFAULT', 'DRONE3_IP')
drone4_ip = config.get('DEFAULT', 'DRONE4_IP')
drone5_ip = config.get('DEFAULT', 'DRONE5_IP')

# 打印示例，你可以根据需要进一步处理这些值
print(f"NUM_DRONES: {num_drones}")
print(f"SERVER_IP: {server_ip}")
print(f"DRONE1_IP: {drone1_ip}")
print(f"DRONE2_IP: {drone2_ip}")
print(f"DRONE3_IP: {drone3_ip}")
print(f"DRONE4_IP: {drone4_ip}")
print(f"DRONE5_IP: {drone5_ip}")

def cleanup_local_remote():
    # 本地清理
    cleanup()
    # 远程清理
    for i in range(num_drones):
        print(f"Stopping all nodes on drone {i+1}")
        subprocess.run(f'bash stop_all_nodes.sh {i+1}', shell=True)

# 主菜单函数
def main_menu(log_dir):
    rviz_pid = None
    while True:
        print("\n=== 菜单 ===")
        print("1. 启动 1_start_all_vins  4_start_all_drones  5_start_all_ranging_fusion  6_start_all_run_control 9_start_all_ego_planner")
        print("2. 重启指定无人机vins（如果某架无人机漂了）")
        print("3. 所有无人机takeoff")
        print("4. 本架无人机takeoff")
        print("5. 所有无人机land")
        print("6. 开始记录rosbag")
        print("7. 启动可视化，轻微晃动查看是否漂移")
        print("8. 关闭可视化")
        print("0. 退出")

        choice = input("请输入选项：")

        if choice == '1':
            launch_process('bash 1_start_all_vins.sh', f'{log_dir}/1_start_all_vins.log')
            launch_process('bash 4_start_all_drones.sh', f'{log_dir}/4_start_all_drones.log')
            launch_process('bash 5_start_all_ranging_fusion.sh', f'{log_dir}/5_start_all_ranging_fusion.log')
            launch_process('bash 6_start_all_run_control.sh', f'{log_dir}/6_start_all_run_control.log')
            launch_process('bash 9_start_all_ego_planner.sh', f'{log_dir}/9_start_all_ego_planner.log')
        elif choice == '2':
            drone_id = input("请输入无人机编号：")
            launch_process(f'bash stop_vins.sh {drone_id}', f'{log_dir}/stop_vins_{drone_id}.log')
            launch_process(f'bash start_target_vins.sh {drone_id}', f'{log_dir}/start_vins_{drone_id}.log')
        elif choice == '3':
            subprocess.run('bash 7_all_takeoff.sh', shell=True)
        elif choice == '4':
            subprocess.run(f'bash tmp/takeoff.sh', shell=True)
        elif choice == '5':
            subprocess.run('bash A_all_land.sh', shell=True)
        elif choice == '6':
            launch_process('bash 8_start_record.sh', f'{log_dir}/8_start_record.log')
        elif choice == '7':
            # rviz_pid = launch_process('bash 2_start_rviz.sh', f'{log_dir}/2_start_rviz.log')
            rviz_pid = launch_process('roslaunch network_utils rviz_single_drone.launch', f'{log_dir}/network_utils_rviz_single_drone.log')
            # net_node_pid = launch_process('bash 3_start_all_drone_node.sh', f'{log_dir}/3_start_all_drone_node')
        elif choice == '8':
            if rviz_pid:
                stop_process(rviz_pid)
                # stop_process(net_node_pid)
        elif choice == '0':
            print("退出程序")
            cleanup_local_remote()
            break
        else:
            print("无效选项，请重新输入")

if __name__ == "__main__":
    # 创建一个日志目录
    log_dir = '/home/coolas/JKW_PROJECT/swarm_vins_ws/logs/'
    os.makedirs(log_dir, exist_ok=True)

    # 切换到工作目录并设置环境
    os.chdir('/home/coolas/JKW_PROJECT/swarm_vins_ws/src/swarm_vins/network_utils/shfiles/')
    subprocess.run('. /home/coolas/JKW_PROJECT/swarm_vins_ws/devel/setup.sh', shell=True)

    atexit.register(cleanup_local_remote)
    main_menu(log_dir)
