import os
import subprocess
import signal

# 查找所有包含 name 的进程并杀死
def kill_all(name):
    try:
        # 使用 pgrep 查找所有包含 name 的进程 ID
        pgrep_process = subprocess.Popen(['pgrep', '-f', name], stdout=subprocess.PIPE)
        pgrep_output, _ = pgrep_process.communicate()
        pgrep_process.wait()

        if pgrep_output:
            # 拆分进程 ID 列表
            roslaunch_pids = pgrep_output.decode().split()
            
            # 逐个杀死进程
            for pid_str in roslaunch_pids:
                try:
                    pid = int(pid_str)
                    # 发送 SIGTERM 信号
                    os.kill(pid, signal.SIGTERM)
                except ValueError:
                    print(f"Invalid PID found: {pid_str}")
                except OSError as e:
                    print(f"Failed to kill PID {pid}: {e}")
        
        print(f"All {name} processes killed successfully.")

    except Exception as e:
        print(f"Error occurred while killing {name} processes: {e}")

# 调用函数杀死所有相关进程
kill_all('roslaunch')
kill_all('rosbag')
