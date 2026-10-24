
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

rclpy.init()
node = rclpy.create_node('scan_capture')
data = {}

data['robot_x']       = -12.25
data['robot_y']       = 12.1
data['robot_yaw']     = 3.1416
data['robot_yaw_deg'] = 180.0


def scan_cb(msg):
    data['ranges'] = list(msg.ranges)
    data['angle_min'] = msg.angle_min
    data['angle_max'] = msg.angle_max
    data['angle_increment'] = msg.angle_increment
    np.save('./test/data/scan_corner_apprach.npy', data)
    print(f"Saved — pos=({data['robot_x']:.3f}, {data['robot_y']:.3f}) yaw={math.degrees(data['robot_yaw']):.1f}deg")
    rclpy.shutdown()


node.create_subscription(LaserScan, '/scan', scan_cb, 10)
# node.create_subscription(Odometry, '/odometry/filtered', odom_cb, 10)
# node.create_subscription(ModelStates, '/gazebo/model_states', model_cb, 10)

rclpy.spin(node)