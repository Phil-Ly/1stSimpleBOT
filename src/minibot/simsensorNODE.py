import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class SensorEmulatorNode(Node):

    def __init__(self):
        super().__init__('sensor_emulator_node')

        # Publisher定义
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_noisy', 10)

        # 定时器（10Hz模拟传感器）
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.t = 0.0

        self.get_logger().info("Sensor Emulator Node Started")

    def timer_callback(self):

        self.t += 0.1

        self.publish_imu()
        self.publish_scan()
        self.publish_odom()

    # ---------------- IMU ----------------
    def publish_imu(self):
        msg = Imu()

        msg.angular_velocity.z = 0.2
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.imu_pub.publish(msg)

    # ---------------- LiDAR ----------------
    def publish_scan(self):
        msg = LaserScan()

        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.1

        # 模拟距离数据（正弦障碍物）
        ranges = []
        for i in range(30):
            d = 2.0 + 0.5 * np.sin(self.t + i * 0.1)
            ranges.append(float(d))

        msg.ranges = ranges

        self.scan_pub.publish(msg)

    # ---------------- ODOM ----------------
    def publish_odom(self):
        msg = Odometry()

        # 简单运动模型（直线 + 噪声）
        x = 0.5 * self.t
        noise = np.random.normal(0, 0.02)

        msg.pose.pose.position.x = x + noise
        msg.pose.pose.position.y = 0.0

        # 简化姿态
        q = Quaternion()
        q.w = 1.0
        msg.pose.pose.orientation = q

        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = SensorEmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()