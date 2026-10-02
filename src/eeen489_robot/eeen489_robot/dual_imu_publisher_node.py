#!/usr/bin/env python3
"""ROS publisher for the robot's dual LSM6DSOX IMU array.

This node reads acceleration and gyroscope data from two IMUs on the same I2C
bus and republishes each stream as a standard ROS Imu message. The design keeps
sensor A on the moving platform and sensor B fixed to the robot body so that the
relative pitch can be estimated by downstream perception nodes.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

try:
    from eeen489_robot.locked_i2c_bus import LockedI2CBus
except (ImportError, ModuleNotFoundError):
    try:
        from .locked_i2c_bus import LockedI2CBus
    except (ImportError, ModuleNotFoundError):
        from locked_i2c_bus import LockedI2CBus

try:
    import adafruit_lsm6ds.lsm6dsox as lsm6dsox
except ImportError as exc:  # pragma: no cover - runtime dependency is hardware-specific
    raise RuntimeError(
        'The adafruit_lsm6ds package is required for the LSM6DSOX IMU sensors.'
    ) from exc


class DualImuPublisherNode(Node):
    """Publish IMU data for the two LSM6DSOX sensors on the robot.

    Sensor A is mounted on the moving platform while sensor B stays fixed to the
    robot body. Downstream logic can compute the relative tilt between the two
    sensors by comparing the published IMU streams.
    """

    def __init__(self):
        """Initialize the ROS node and the two IMU devices on the configured bus."""
        super().__init__('dual_imu_publisher_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('sensor_a_address', 0x6A)
        self.declare_parameter('sensor_b_address', 0x6B)
        self.declare_parameter('imu_a_topic', '/imu_a')
        self.declare_parameter('imu_b_topic', '/imu_b')
        self.declare_parameter('frame_id_a', 'imu_a_link')
        self.declare_parameter('frame_id_b', 'imu_b_link')
        self.declare_parameter('publish_rate_hz', 100.0)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.sensor_a_address = self.get_parameter('sensor_a_address').value
        self.sensor_b_address = self.get_parameter('sensor_b_address').value
        imu_a_topic = self.get_parameter('imu_a_topic').value
        imu_b_topic = self.get_parameter('imu_b_topic').value
        self.frame_id_a = self.get_parameter('frame_id_a').value
        self.frame_id_b = self.get_parameter('frame_id_b').value
        rate = self.get_parameter('publish_rate_hz').value

        self.get_logger().info(
            f'Initializing dual LSM6DSOX IMU stream on I2C bus {self.bus_id}'
        )

        # The two sensors share the same bus, but each chip has a unique I2C slave
        # address. They are intentionally treated as independent streams so the
        # relative motion between the sensor platform and the robot body can be
        # computed later in the perception system.
        self.i2c = LockedI2CBus(self.bus_id)
        self.sensor_a = lsm6dsox.LSM6DSOX(self.i2c, address=self.sensor_a_address)
        self.sensor_b = lsm6dsox.LSM6DSOX(self.i2c, address=self.sensor_b_address)

        self.pub_a = self.create_publisher(Imu, imu_a_topic, 10)
        self.pub_b = self.create_publisher(Imu, imu_b_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'Publishing sensor A on {imu_a_topic} and sensor B on {imu_b_topic}'
        )

    def make_imu_msg(self, sensor, frame_id):
        """Convert one sensor readout into a ROS Imu message.

        The function preserves the sensor's acceleration and angular velocity,
        leaves orientation as identity, and marks the covariance fields as
        unknown because the robot does not estimate a full orientation solution in
        this node.
        """
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        try:
            ax, ay, az = sensor.acceleration
            gx, gy, gz = sensor.gyro
        except Exception as exc:  # pragma: no cover - hardware access failure
            self.get_logger().warn(f'Failed to read IMU data: {exc}')
            ax, ay, az = (0.0, 0.0, 0.0)
            gx, gy, gz = (0.0, 0.0, 0.0)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        # Pose is not estimated here; keep the orientation identity.
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # ROS convention uses -1.0 in the first covariance slot to indicate that
        # a quantity is not yet estimated or is intentionally unknown.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        return msg

    def timer_callback(self):
        """Publish the latest data from both IMUs at the configured rate."""
        imu_a_msg = self.make_imu_msg(self.sensor_a, self.frame_id_a)
        imu_b_msg = self.make_imu_msg(self.sensor_b, self.frame_id_b)

        self.pub_a.publish(imu_a_msg)
        self.pub_b.publish(imu_b_msg)


def main(args=None):
    """Launch the node and block until ROS shutdown is requested."""
    rclpy.init(args=args)
    node = DualImuPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
