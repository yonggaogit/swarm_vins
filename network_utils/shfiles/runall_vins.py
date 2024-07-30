#!/usr/bin/env python3
import atexit
import os
import signal
import subprocess
import time
import psutil

# 定义一个pid_set，用于存储所有进程的pid
pid_set = set()

# 递归的杀死进程树
def _kill_process_tree(pid):
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        
        # Terminate children processes
        for child in children:
            child.terminate()

        # Wait for processes to terminate
        gone, alive = psutil.wait_procs(children, timeout=5)
        for proc in alive:
            proc.kill()

        # Finally, terminate the parent process
        parent.terminate()
        parent.wait(5)
        
        print(f"Process {pid} and its children terminated successfully.")
        
    except psutil.NoSuchProcess:
        print(f"Process {pid} not found.")


# 进程启动函数，返回进程的pid，同时将pid加入pid_set
def launch_process(cmd, log_file, wait_time=3):
    with open(log_file, 'w') as f:
        proc = subprocess.Popen(cmd, shell=True, stdout=f, stderr=subprocess.STDOUT)
        pid_set.add(proc.pid)
        print(f"启动进程 {cmd}，pid: {proc.pid}")
        time.sleep(wait_time)
        return proc.pid

# 停止进程函数，同时将pid从pid_set中移除
def stop_process(pid, wait_time=0):
    _kill_process_tree(pid)
    pid_set.remove(pid)
    time.sleep(wait_time)

# 清理函数
def cleanup():
    print("Cleaning up...")
    pid_set_copy = pid_set.copy()
    for pid in pid_set_copy:
        stop_process(pid)

# 主菜单函数
def main_menu(log_dir):
    rspx4_pid = None
    while True:
        print("\n=== 菜单 ===")
        print("1. 启动 rviz  single_run_in_exp  run_ctrl  rspx4")
        print("2. 重启 rspx4")
        print("3. 执行 takeoff")
        print("4. 执行 land")
        print("0. 退出")

        choice = input("请输入选项（0-4）：")

        if choice == '1':
            launch_process('roslaunch ego_planner rviz.launch', f'{log_dir}/ego_planner_rviz.log')
            launch_process('roslaunch ego_planner single_run_in_exp.launch', f'{log_dir}/ego_planner_single_run_in_exp.log')
            launch_process('roslaunch px4ctrl run_ctrl.launch', f'{log_dir}/px4ctrl_run_ctrl.log')
            rspx4_pid = launch_process('bash shfiles/rspx4.sh', f'{log_dir}/rspx4.log')
        elif choice == '2':
            if rspx4_pid:
                stop_process(rspx4_pid)
                rspx4_pid = launch_process('bash shfiles/rspx4.sh', f'{log_dir}/rspx4.log')
            else:
                print("rspx4 还未启动，请先选择选项 '1' 启动。")
        elif choice == '3':
            subprocess.run('bash shfiles/takeoff.sh', shell=True)
        elif choice == '4':
            subprocess.run('bash shfiles/land.sh', shell=True)
        elif choice == '0':
            print("退出程序")
            break
        else:
            print("无效选项，请重新输入")

if __name__ == "__main__":
    # 创建一个日志目录
    log_dir = '/home/coolas/JKW_PROJECT/Fast-Drone-250/logs/'
    os.makedirs(log_dir, exist_ok=True)

    # 切换到工作目录并设置环境
    os.chdir('/home/coolas/JKW_PROJECT/Fast-Drone-250')
    subprocess.run('. devel/setup.sh', shell=True)

    atexit.register(cleanup)
    main_menu(log_dir)
