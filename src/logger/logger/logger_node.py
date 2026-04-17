import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import json
import os
from datetime import datetime

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        self.base_path = os.path.expanduser("~/robot_logs")
        os.makedirs(self.base_path, exist_ok=True)

        self.system_file = open(self._file("system_log.json"), "a")
        self.alert_file = open(self._file("alerts_log.json"), "a")
        self.state_file = open(self._file("state_log.json"), "a")
        self.control_file = open(self._file("control_log.json"), "a")
        self.nav_file = open(self._file("navigation_log.json"), "a")

        self.create_subscription(String, '/system_status', self.system_cb, 10)
        self.create_subscription(String, '/system_alerts', self.alert_cb, 10)

        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(NavSatFix, '/gps', self.gps_cb, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

    def _file(self, name):
        return os.path.join(self.base_path, name)

    def _time(self):
        return datetime.utcnow().isoformat()

    def _write(self, file, data):
        file.write(json.dumps(data) + "\n")
        file.flush()

    def system_cb(self, msg):
        data = {
            "time": self._time(),
            "data": msg.data
        }
        self._write(self.system_file, data)

    def alert_cb(self, msg):
        data = {
            "time": self._time(),
            "data": msg.data
        }
        self._write(self.alert_file, data)

    def imu_cb(self, msg):
        data = {
            "time": self._time(),
            "imu": {
                "orientation": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                "angular_velocity": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "linear_acceleration": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            }
        }
        self._write(self.state_file, data)

    def gps_cb(self, msg):
        data = {
            "time": self._time(),
            "gps": {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "alt": msg.altitude
            }
        }
        self._write(self.state_file, data)

    def odom_cb(self, msg):
        data = {
            "time": self._time(),
            "odom": {
                "position": [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y
                ],
                "orientation": [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ],
                "velocity": [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.angular.z
                ]
            }
        }
        self._write(self.state_file, data)

    def cmd_cb(self, msg):
        data = {
            "time": self._time(),
            "cmd_vel": {
                "linear": msg.linear.x,
                "angular": msg.angular.z
            }
        }
        self._write(self.control_file, data)

    def goal_cb(self, msg):
        data = {
            "time": self._time(),
            "goal": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y
            }
        }
        self._write(self.nav_file, data)


def main():
    rclpy.init()
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
