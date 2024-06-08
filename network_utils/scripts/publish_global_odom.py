import rospy
from nav_msgs.msg import Odometry, Path
import socket
import threading

class DroneServer:
    def __init__(self, ip, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip, port))
        self.server_socket.listen(5)
        self.clients = []

        self.odom_publishers = {}
        self.path_publishers = {}

    def handle_client(self, client_socket):
        while True:
            try:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                self.process_data(data)
            except ConnectionResetError:
                break
        client_socket.close()

    def process_data(self, data):
        parts = data.split(',')
        if len(parts) != 11:
            return

        drone_id = int(parts[0].split(':')[1])
        time = float(parts[1].split(':')[1])
        position = [float(p) for p in parts[2].split(':')[1:]]
        orientation = [float(o) for o in parts[3].split(':')[1:]]

        odometry = Odometry()
        odometry.header.stamp = rospy.Time.from_sec(time)
        odometry.header.frame_id = "world"
        odometry.pose.pose.position.x = position[0]
        odometry.pose.pose.position.y = position[1]
        odometry.pose.pose.position.z = position[2]
        odometry.pose.pose.orientation.w = orientation[0]
        odometry.pose.pose.orientation.x = orientation[1]
        odometry.pose.pose.orientation.y = orientation[2]
        odometry.pose.pose.orientation.z = orientation[3]

        if drone_id not in self.odom_publishers:
            self.odom_publishers[drone_id] = rospy.Publisher(f'/drone_{drone_id}/global_odometry', Odometry, queue_size=10)
            self.path_publishers[drone_id] = rospy.Publisher(f'/drone_{drone_id}/global_path', Path, queue_size=10)
        
        self.odom_publishers[drone_id].publish(odometry)

    def start(self):
        while not rospy.is_shutdown():
            client_socket, addr = self.server_socket.accept()
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()
            self.clients.append(client_thread)

if __name__ == '__main__':
    rospy.init_node('publish_global_odom')
    server_ip = rospy.get_param('~server_ip', '0.0.0.0')
    server_port = rospy.get_param('~server_port', 12345)

    server = DroneServer(server_ip, server_port)
    server.start()
