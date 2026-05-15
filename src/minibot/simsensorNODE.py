import rclpy
from rclpy.node import Node

import copy
import numpy as np

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class _SimSource:
    def __init__(self):
        self.t = 0.0

    def step(self, dt: float) -> None:
        self.t += dt

    def make_imu(self) -> Imu:
        msg = Imu()
        msg.angular_velocity.z = 0.2
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        return msg

    def make_scan(self) -> LaserScan:
        msg = LaserScan()
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.1

        ranges = []
        for i in range(30):
            d = 2.0 + 0.5 * np.sin(self.t + i * 0.1)
            ranges.append(float(d))
        msg.ranges = ranges
        return msg

    def make_odom(self) -> Odometry:
        msg = Odometry()
        x = 0.5 * self.t
        noise = np.random.normal(0, 0.02)

        msg.pose.pose.position.x = x + noise
        msg.pose.pose.position.y = 0.0

        q = Quaternion()
        q.w = 1.0
        msg.pose.pose.orientation = q
        return msg


class SensorEmulatorNode(Node):

    def __init__(self):
        super().__init__('sensor_emulator_node')

        # mode: sim（内部仿真生成） / topics（订阅真实驱动topic并转发）
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('publish_rate_hz', 10.0)  # sim模式下生效

        # 输入topic（topics模式下生效）
        self.declare_parameter('imu_in_topic', '/imu/data_raw')
        self.declare_parameter('scan_in_topic', '/scan')
        self.declare_parameter('odom_in_topic', '/odom')

        # 输出topic（两种模式都生效）
        self.declare_parameter('imu_out_topic', '/imu/data')
        self.declare_parameter('scan_out_topic', '/scan')
        self.declare_parameter('odom_out_topic', '/odom_noisy')

        # header/frame 统一（可选）
        self.declare_parameter('stamp_with_now_if_zero', True)
        self.declare_parameter('imu_frame_id', '')
        self.declare_parameter('scan_frame_id', '')
        self.declare_parameter('odom_frame_id', '')

        # 可选：对里程计加噪声（真实或仿真都可用）
        self.declare_parameter('add_odom_noise', False)
        self.declare_parameter('odom_noise_std_xy', 0.02)

        self.mode = str(self.get_parameter('mode').value).strip().lower()

        imu_out = str(self.get_parameter('imu_out_topic').value)
        scan_out = str(self.get_parameter('scan_out_topic').value)
        odom_out = str(self.get_parameter('odom_out_topic').value)

        self.imu_pub = self.create_publisher(Imu, imu_out, 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_out, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_out, 10)

        self._sim = _SimSource()
        self._timer = None

        if self.mode == 'sim':
            rate_hz = float(self.get_parameter('publish_rate_hz').value)
            period = 1.0 / max(rate_hz, 1e-6)
            self._timer = self.create_timer(period, self._on_sim_timer)
            self.get_logger().info(f"Sensor node started in SIM mode. rate={rate_hz}Hz")
        elif self.mode == 'topics':
            imu_in = str(self.get_parameter('imu_in_topic').value)
            scan_in = str(self.get_parameter('scan_in_topic').value)
            odom_in = str(self.get_parameter('odom_in_topic').value)

            self.create_subscription(Imu, imu_in, self._on_imu, 10)
            self.create_subscription(LaserScan, scan_in, self._on_scan, 10)
            self.create_subscription(Odometry, odom_in, self._on_odom, 10)
            self.get_logger().info(
                "Sensor node started in TOPICS mode. "
                f"imu: {imu_in} -> {imu_out}, scan: {scan_in} -> {scan_out}, odom: {odom_in} -> {odom_out}"
            )
        else:
            self.get_logger().warn(f"Unknown mode '{self.mode}', fallback to 'sim'.")
            self.mode = 'sim'
            rate_hz = float(self.get_parameter('publish_rate_hz').value)
            period = 1.0 / max(rate_hz, 1e-6)
            self._timer = self.create_timer(period, self._on_sim_timer)

    def _maybe_fix_header(self, msg, frame_id_param_name: str):
        if hasattr(msg, 'header'):
            frame_id = str(self.get_parameter(frame_id_param_name).value)
            if frame_id:
                msg.header.frame_id = frame_id

            if bool(self.get_parameter('stamp_with_now_if_zero').value):
                if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                    msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _maybe_add_odom_noise(self, msg: Odometry) -> Odometry:
        if not bool(self.get_parameter('add_odom_noise').value):
            return msg
        std_xy = float(self.get_parameter('odom_noise_std_xy').value)
        msg.pose.pose.position.x += float(np.random.normal(0.0, std_xy))
        msg.pose.pose.position.y += float(np.random.normal(0.0, std_xy))
        return msg

    def _on_sim_timer(self):
        dt = float(self._timer.timer_period_ns) / 1e9 if self._timer is not None else 0.1
        self._sim.step(dt)

        imu = self._sim.make_imu()
        scan = self._sim.make_scan()
        odom = self._sim.make_odom()

        imu = self._maybe_fix_header(imu, 'imu_frame_id')
        scan = self._maybe_fix_header(scan, 'scan_frame_id')
        odom = self._maybe_fix_header(odom, 'odom_frame_id')
        odom = self._maybe_add_odom_noise(odom)

        self.imu_pub.publish(imu)
        self.scan_pub.publish(scan)
        self.odom_pub.publish(odom)

    def _on_imu(self, msg: Imu):
        out = copy.deepcopy(msg)
        out = self._maybe_fix_header(out, 'imu_frame_id')
        self.imu_pub.publish(out)

    def _on_scan(self, msg: LaserScan):
        out = copy.deepcopy(msg)
        out = self._maybe_fix_header(out, 'scan_frame_id')
        self.scan_pub.publish(out)

    def _on_odom(self, msg: Odometry):
        out = copy.deepcopy(msg)
        out = self._maybe_fix_header(out, 'odom_frame_id')
        out = self._maybe_add_odom_noise(out)
        self.odom_pub.publish(out)


def main():
    rclpy.init()
    node = SensorEmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()