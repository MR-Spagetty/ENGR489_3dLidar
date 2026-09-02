#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - non-RPi development machine
    GPIO = None


class Lidar3DCloudNode(Node):
    """Convert a 2D LaserScan into a 3D point cloud using the relative pitch
    between two IMUs.

    Coordinate convention:
      +x : forward
      +y : left
      +z : up

    Sensor A is mounted on the moving platform.
    Sensor B is fixed to the robot body and acts as the reference frame.
    """

    def __init__(self):
        super().__init__('lidar_3d_pointcloud_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_a_topic', '/imu_a')
        self.declare_parameter('imu_b_topic', '/imu_b')
        self.declare_parameter('output_topic', '/point_cloud_3d')
        self.declare_parameter('pose_topic', '/lidar_pose')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('stepper_enabled', True)
        self.declare_parameter('stepper_step_pin', 17)
        self.declare_parameter('stepper_dir_pin', 27)
        self.declare_parameter('stepper_steps_per_rev', 200)
        self.declare_parameter('stepper_gear_ratio', 1.0)
        self.declare_parameter('stepper_microsteps', 1)
        self.declare_parameter('stepper_step_delay_s', 0.0005)
        self.declare_parameter('stepper_max_pitch_rad', 0.523599)  # 30 degrees
        self.declare_parameter('stepper_home_pitch_rad', 0.0)

        scan_topic = self.get_parameter('scan_topic').value
        imu_a_topic = self.get_parameter('imu_a_topic').value
        imu_b_topic = self.get_parameter('imu_b_topic').value
        output_topic = self.get_parameter('output_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.stepper_enabled = self.get_parameter('stepper_enabled').value
        self.stepper_step_pin = self.get_parameter('stepper_step_pin').value
        self.stepper_dir_pin = self.get_parameter('stepper_dir_pin').value
        self.stepper_steps_per_rev = self.get_parameter('stepper_steps_per_rev').value
        self.stepper_gear_ratio = self.get_parameter('stepper_gear_ratio').value
        self.stepper_microsteps = self.get_parameter('stepper_microsteps').value
        self.stepper_step_delay_s = self.get_parameter('stepper_step_delay_s').value
        self.stepper_max_pitch_rad = self.get_parameter('stepper_max_pitch_rad').value
        self.stepper_home_pitch_rad = self.get_parameter('stepper_home_pitch_rad').value

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.imu_a_sub = self.create_subscription(Imu, imu_a_topic, self.imu_a_callback, 10)
        self.imu_b_sub = self.create_subscription(Imu, imu_b_topic, self.imu_b_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        self.latest_scan = None
        self.imu_a = None
        self.imu_b = None
        self.stepper_target_pitch = self.stepper_home_pitch_rad
        self.stepper_current_pitch = self.stepper_home_pitch_rad

        if self.stepper_enabled:
            self._setup_stepper()

        self.get_logger().info(
            'Listening to %s, %s, %s; publishing %s and %s',
            scan_topic,
            imu_a_topic,
            imu_b_topic,
            output_topic,
            pose_topic,
        )

    def _setup_stepper(self):
        if GPIO is None:
            self.get_logger().warn('RPi.GPIO not available; stepper control disabled.')
            self.stepper_enabled = False
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.stepper_step_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.stepper_dir_pin, GPIO.OUT, initial=GPIO.LOW)
        self.get_logger().info(
            'Stepper enabled on BCM step=%d dir=%d',
            self.stepper_step_pin,
            self.stepper_dir_pin,
        )

    def _set_stepper_direction(self, direction):
        if not self.stepper_enabled or GPIO is None:
            return
        GPIO.output(self.stepper_dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)

    def _step_stepper(self, steps):
        if not self.stepper_enabled or GPIO is None or steps == 0:
            return

        self._set_stepper_direction(1 if steps > 0 else -1)
        for _ in range(abs(steps)):
            GPIO.output(self.stepper_step_pin, GPIO.HIGH)
            time.sleep(self.stepper_step_delay_s)
            GPIO.output(self.stepper_step_pin, GPIO.LOW)
            time.sleep(self.stepper_step_delay_s)

    def _command_stepper_to_pitch(self, desired_pitch_rad):
        if not self.stepper_enabled:
            return

        desired_pitch_rad = max(-self.stepper_max_pitch_rad, min(self.stepper_max_pitch_rad, desired_pitch_rad))
        pitch_error = desired_pitch_rad - self.stepper_current_pitch
        if abs(pitch_error) < 1e-6:
            return

        total_steps_per_rad = (self.stepper_steps_per_rev * self.stepper_gear_ratio * self.stepper_microsteps) / (2.0 * math.pi)
        steps = int(round(pitch_error * total_steps_per_rad))

        if steps != 0:
            self._step_stepper(steps)
            self.stepper_current_pitch += steps / total_steps_per_rad

    def imu_a_callback(self, msg):
        self.imu_a = msg

    def imu_b_callback(self, msg):
        self.imu_b = msg

    def scan_callback(self, msg):
        if self.imu_a is None or self.imu_b is None:
            self.get_logger().debug('Waiting for both IMU readings before generating point cloud.')
            return

        if not self.latest_scan or msg.header.stamp >= self.latest_scan.header.stamp:
            self.latest_scan = msg

        self.publish_3d_cloud(msg)

    def imu_pitch_rad(self, imu_msg):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        # Standard pitch for an IMU with +x forward, +y left, +z up.
        return math.atan2(-ax, math.sqrt(ay * ay + az * az))

    def relative_pitch_rad(self):
        if self.imu_a is None or self.imu_b is None:
            return None
        return self.imu_pitch_rad(self.imu_a) - self.imu_pitch_rad(self.imu_b)

    def build_cloud_from_scan(self, scan_msg):
        rel_pitch = self.relative_pitch_rad()
        if rel_pitch is None:
            return []

        # For a nose-up platform, the scan needs to be rotated upward in the x-z plane.
        tilt = -rel_pitch
        c = math.cos(tilt)
        s = math.sin(tilt)

        points = []
        for i, rng in enumerate(scan_msg.ranges):
            if not math.isfinite(rng):
                continue
            if rng < scan_msg.range_min or rng > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = rng * math.cos(angle)
            y = rng * math.sin(angle)
            z = 0.0

            x_rot = x * c + z * s
            z_rot = -x * s + z * c
            points.append((x_rot, y, z_rot))

        return points

    def publish_pose(self, scan_msg):
        rel_pitch = self.relative_pitch_rad()
        if rel_pitch is None:
            return

        pose = PoseStamped()
        pose.header.stamp = scan_msg.header.stamp
        pose.header.frame_id = self.reference_frame

        # Laser frame is pitched relative to the base frame.
        roll = 0.0
        pitch = -rel_pitch
        yaw = 0.0

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        pose.pose.orientation.z = sy * cr * cp - cy * sr * sp

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        self.pose_pub.publish(pose)

    def publish_3d_cloud(self, scan_msg):
        rel_pitch = self.relative_pitch_rad()
        if rel_pitch is not None:
            self.stepper_target_pitch = rel_pitch
            self._command_stepper_to_pitch(self.stepper_target_pitch)

        points = self.build_cloud_from_scan(scan_msg)
        if not points:
            return

        header = Header()
        header.stamp = scan_msg.header.stamp
        header.frame_id = self.frame_id

        point_cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(point_cloud)
        self.publish_pose(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
