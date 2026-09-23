#!/usr/bin/env python3

import atexit
import math
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
try:
    from sensor_msgs import point_cloud2
except ImportError:
    # Python 3.12 compatibility: use sensor_msgs_py if available
    from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - non-RPi development machine
    GPIO = None

# For Raspberry Pi 5 support, try gpiozero (newer GPIO library)
try:
    from gpiozero import OutputDevice
except ImportError:
    OutputDevice = None


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

        numeric_param_descriptor = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_a_topic', '/imu_a')
        self.declare_parameter('imu_b_topic', '/imu_b')
        self.declare_parameter('output_topic', '/point_cloud_3d')
        self.declare_parameter('output_flat_scan_topic', '/scan_flat')
        self.declare_parameter('pose_topic', '/lidar_pose')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('flat_scan_enabled', True)
        self.declare_parameter('flat_scan_slice_min_z_m', 0.005, descriptor=numeric_param_descriptor)
        self.declare_parameter('flat_scan_slice_max_z_m', 0.390, descriptor=numeric_param_descriptor)
        self.declare_parameter('flat_scan_ground_offset_z_m', 0.308, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_enabled', True)
        self.declare_parameter('stepper_step_pin', 17)
        self.declare_parameter('stepper_dir_pin', 27)
        self.declare_parameter('stepper_enable_pin', -1)
        self.declare_parameter('stepper_enable_active_low', True)
        self.declare_parameter('stepper_dir_invert', False)
        self.declare_parameter('stepper_test_steps', 0)
        self.declare_parameter('stepper_steps_per_rev', 32)
        self.declare_parameter('stepper_gear_ratio', 64.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_microsteps', 1)
        self.declare_parameter('stepper_step_delay_s', 0.0005, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_max_rate_hz', 500.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_min_move_rad', 0.002, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_enabled', True)
        self.declare_parameter('stepper_scan_period_s', 6.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_min_pitch_deg', -45.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_max_pitch_deg', 45.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_home_pitch_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_use_live_zero', True)
        self.declare_parameter('stepper_home_kp', 1.2, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_home_ki', 0.15, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_home_max_step_deg', 2.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_limit_tolerance_deg', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_step_deg', 0.75, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_ease_zone_deg', 12.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_min_speed_scale', 0.25, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_pi_kp', 0.9, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_pi_ki', 0.08, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_pi_max_step_deg', 1.5, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_pi_integral_limit_deg', 10.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_escape_step_deg', 2.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('relative_pitch_filter_alpha', 0.25, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_drift_k', 0.04, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_drift_limit_deg', 8.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_drift_max_step_deg', 0.06, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_endpoint_correction_enabled', True)
        self.declare_parameter('stepper_scan_endpoint_tolerance_deg', 0.8, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_endpoint_window_deg', 3.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_endpoint_max_step_deg', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_endpoint_recover_enabled', True)
        self.declare_parameter('stepper_scan_endpoint_recover_window_deg', 0.6, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_endpoint_recover_step_deg', 0.5, descriptor=numeric_param_descriptor)
        self.declare_parameter('stepper_scan_cloud_feedback_blend', 0.35, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_drift_comp_enabled', True)
        self.declare_parameter('output_cloud_drift_comp_k', 0.12, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_drift_comp_limit_deg', 12.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pitch_sign', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_pitch_sign', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pose_pitch_sign', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_sign_x', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_sign_y', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_sign_z', 1.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pose_roll_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pose_pitch_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pose_yaw_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_roll_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_pitch_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_cloud_yaw_offset_deg', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_mount_offset_x_m', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_mount_offset_y_m', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_mount_offset_z_m', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pivot_offset_x_m', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pivot_offset_y_m', 0.0, descriptor=numeric_param_descriptor)
        self.declare_parameter('output_pivot_offset_z_m', 0.0, descriptor=numeric_param_descriptor)

        scan_topic = self.get_parameter('scan_topic').value
        imu_a_topic = self.get_parameter('imu_a_topic').value
        imu_b_topic = self.get_parameter('imu_b_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_flat_scan_topic = self.get_parameter('output_flat_scan_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.flat_scan_enabled = self.get_parameter('flat_scan_enabled').value
        self.flat_scan_slice_min_z_m = self._get_float_parameter('flat_scan_slice_min_z_m', 0.005)
        self.flat_scan_slice_max_z_m = self._get_float_parameter('flat_scan_slice_max_z_m', 0.390)
        self.flat_scan_ground_offset_z_m = self._get_float_parameter('flat_scan_ground_offset_z_m', 0.308)
        if self.flat_scan_slice_min_z_m > self.flat_scan_slice_max_z_m:
            self.get_logger().warn(
                'flat_scan_slice_min_z_m is greater than flat_scan_slice_max_z_m; swapping values.'
            )
            self.flat_scan_slice_min_z_m, self.flat_scan_slice_max_z_m = (
                self.flat_scan_slice_max_z_m,
                self.flat_scan_slice_min_z_m,
            )
        self.stepper_enabled = self.get_parameter('stepper_enabled').value
        self.stepper_step_pin = self.get_parameter('stepper_step_pin').value
        self.stepper_dir_pin = self.get_parameter('stepper_dir_pin').value
        self.stepper_enable_pin = self.get_parameter('stepper_enable_pin').value
        self.stepper_enable_active_low = self.get_parameter('stepper_enable_active_low').value
        self.stepper_dir_invert = self.get_parameter('stepper_dir_invert').value
        self.stepper_test_steps = self.get_parameter('stepper_test_steps').value
        self.stepper_steps_per_rev = self.get_parameter('stepper_steps_per_rev').value
        self.stepper_gear_ratio = self._get_float_parameter('stepper_gear_ratio', 64.0)
        if self.stepper_gear_ratio <= 0.0:
            self.get_logger().warn('stepper_gear_ratio must be > 0; forcing to 1.0.')
            self.stepper_gear_ratio = 1.0
        self.stepper_microsteps = self.get_parameter('stepper_microsteps').value
        self.stepper_step_delay_s = self._get_float_parameter('stepper_step_delay_s', 0.0005)
        self.stepper_max_rate_hz = self._get_float_parameter('stepper_max_rate_hz', 500.0)
        self.stepper_min_move_rad = self._get_float_parameter('stepper_min_move_rad', 0.002)
        self.stepper_scan_enabled = self.get_parameter('stepper_scan_enabled').value
        self.stepper_scan_period_s = self._get_float_parameter('stepper_scan_period_s', 6.0)
        self.stepper_min_pitch_deg = self._get_float_parameter('stepper_min_pitch_deg', -45.0)
        self.stepper_max_pitch_deg = self._get_float_parameter('stepper_max_pitch_deg', 45.0)
        self.stepper_home_pitch_deg = self._get_float_parameter('stepper_home_pitch_deg', 0.0)
        self.stepper_use_live_zero = self.get_parameter('stepper_use_live_zero').value
        self.stepper_home_kp = self._get_float_parameter('stepper_home_kp', 1.2)
        self.stepper_home_ki = self._get_float_parameter('stepper_home_ki', 0.15)
        self.stepper_home_max_step_deg = self._get_float_parameter('stepper_home_max_step_deg', 2.0)
        self.stepper_scan_limit_tolerance_deg = self._get_float_parameter(
            'stepper_scan_limit_tolerance_deg', 1.0
        )
        self.stepper_scan_step_deg = self._get_float_parameter('stepper_scan_step_deg', 0.75)
        self.stepper_scan_ease_zone_deg = self._get_float_parameter('stepper_scan_ease_zone_deg', 12.0)
        self.stepper_scan_min_speed_scale = self._get_float_parameter(
            'stepper_scan_min_speed_scale', 0.25
        )
        self.stepper_scan_pi_kp = self._get_float_parameter('stepper_scan_pi_kp', 0.9)
        self.stepper_scan_pi_ki = self._get_float_parameter('stepper_scan_pi_ki', 0.08)
        self.stepper_scan_pi_max_step_deg = self._get_float_parameter(
            'stepper_scan_pi_max_step_deg', 1.5
        )
        self.stepper_scan_pi_integral_limit_deg = self._get_float_parameter(
            'stepper_scan_pi_integral_limit_deg', 10.0
        )
        self.stepper_scan_escape_step_deg = self._get_float_parameter(
            'stepper_scan_escape_step_deg', 2.0
        )
        self.relative_pitch_filter_alpha = self._get_float_parameter('relative_pitch_filter_alpha', 0.25)
        self.relative_pitch_filter_alpha = max(0.0, min(1.0, self.relative_pitch_filter_alpha))
        self.stepper_scan_drift_k = self._get_float_parameter('stepper_scan_drift_k', 0.04)
        self.stepper_scan_drift_k = max(0.0, min(1.0, self.stepper_scan_drift_k))
        self.stepper_scan_drift_limit_deg = self._get_float_parameter('stepper_scan_drift_limit_deg', 8.0)
        self.stepper_scan_drift_max_step_deg = self._get_float_parameter(
            'stepper_scan_drift_max_step_deg', 0.06
        )
        self.stepper_scan_endpoint_correction_enabled = self.get_parameter(
            'stepper_scan_endpoint_correction_enabled'
        ).value
        self.stepper_scan_endpoint_tolerance_deg = self._get_float_parameter(
            'stepper_scan_endpoint_tolerance_deg', 0.8
        )
        self.stepper_scan_endpoint_window_deg = self._get_float_parameter(
            'stepper_scan_endpoint_window_deg', 3.0
        )
        self.stepper_scan_endpoint_max_step_deg = self._get_float_parameter(
            'stepper_scan_endpoint_max_step_deg', 1.0
        )
        self.stepper_scan_endpoint_recover_enabled = self.get_parameter(
            'stepper_scan_endpoint_recover_enabled'
        ).value
        self.stepper_scan_endpoint_recover_window_deg = self._get_float_parameter(
            'stepper_scan_endpoint_recover_window_deg', 0.6
        )
        self.stepper_scan_endpoint_recover_step_deg = self._get_float_parameter(
            'stepper_scan_endpoint_recover_step_deg', 0.5
        )
        self.stepper_scan_cloud_feedback_blend = self._get_float_parameter(
            'stepper_scan_cloud_feedback_blend', 0.35
        )
        self.stepper_scan_cloud_feedback_blend = max(0.0, min(1.0, self.stepper_scan_cloud_feedback_blend))
        self.output_cloud_drift_comp_enabled = self.get_parameter('output_cloud_drift_comp_enabled').value
        self.output_cloud_drift_comp_k = self._get_float_parameter('output_cloud_drift_comp_k', 0.12)
        self.output_cloud_drift_comp_k = max(0.0, min(1.0, self.output_cloud_drift_comp_k))
        self.output_cloud_drift_comp_limit_deg = self._get_float_parameter(
            'output_cloud_drift_comp_limit_deg', 12.0
        )
        legacy_output_pitch_sign = self._sign_from_value(
            self._get_float_parameter('output_pitch_sign', 1.0)
        )
        self.output_cloud_pitch_sign = self._sign_from_value(
            self._get_float_parameter('output_cloud_pitch_sign', legacy_output_pitch_sign)
        )
        self.output_pose_pitch_sign = self._sign_from_value(
            self._get_float_parameter('output_pose_pitch_sign', legacy_output_pitch_sign)
        )
        self.output_cloud_sign_x = self._sign_from_value(
            self._get_float_parameter('output_cloud_sign_x', 1.0)
        )
        self.output_cloud_sign_y = self._sign_from_value(
            self._get_float_parameter('output_cloud_sign_y', 1.0)
        )
        self.output_cloud_sign_z = self._sign_from_value(
            self._get_float_parameter('output_cloud_sign_z', 1.0)
        )
        self.output_pose_roll_offset_deg = self._get_float_parameter('output_pose_roll_offset_deg', 0.0)
        self.output_pose_pitch_offset_deg = self._get_float_parameter('output_pose_pitch_offset_deg', 0.0)
        self.output_pose_yaw_offset_deg = self._get_float_parameter('output_pose_yaw_offset_deg', 0.0)
        self.output_cloud_roll_offset_deg = self._get_float_parameter('output_cloud_roll_offset_deg', 0.0)
        self.output_cloud_pitch_offset_deg = self._get_float_parameter('output_cloud_pitch_offset_deg', 0.0)
        self.output_cloud_yaw_offset_deg = self._get_float_parameter('output_cloud_yaw_offset_deg', 0.0)
        self.output_mount_offset_x_m = self._get_float_parameter('output_mount_offset_x_m', 0.0)
        self.output_mount_offset_y_m = self._get_float_parameter('output_mount_offset_y_m', 0.0)
        self.output_mount_offset_z_m = self._get_float_parameter('output_mount_offset_z_m', 0.0)
        self.output_pivot_offset_x_m = self._get_float_parameter('output_pivot_offset_x_m', 0.0)
        self.output_pivot_offset_y_m = self._get_float_parameter('output_pivot_offset_y_m', 0.0)
        self.output_pivot_offset_z_m = self._get_float_parameter('output_pivot_offset_z_m', 0.0)
        self.output_pose_roll_offset_rad = math.radians(self.output_pose_roll_offset_deg)
        self.output_pose_pitch_offset_rad = math.radians(self.output_pose_pitch_offset_deg)
        self.output_pose_yaw_offset_rad = math.radians(self.output_pose_yaw_offset_deg)
        self.output_cloud_roll_offset_rad = math.radians(self.output_cloud_roll_offset_deg)
        self.output_cloud_pitch_offset_rad = math.radians(self.output_cloud_pitch_offset_deg)
        self.output_cloud_yaw_offset_rad = math.radians(self.output_cloud_yaw_offset_deg)
        self.stepper_min_pitch_rad = math.radians(float(self.stepper_min_pitch_deg))
        self.stepper_max_pitch_rad = math.radians(float(self.stepper_max_pitch_deg))
        self.stepper_home_target_rel_pitch_rad = math.radians(float(self.stepper_home_pitch_deg))
        self.stepper_home_pitch_rad = self.stepper_home_target_rel_pitch_rad
        self.stepper_home_max_step_rad = math.radians(self.stepper_home_max_step_deg)
        self.stepper_scan_limit_tolerance_rad = math.radians(self.stepper_scan_limit_tolerance_deg)
        if self.stepper_scan_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_step_deg must be > 0; forcing to 0.75 deg.')
            self.stepper_scan_step_deg = 0.75
        self.stepper_scan_step_rad = math.radians(self.stepper_scan_step_deg)
        if self.stepper_scan_ease_zone_deg < 0.0:
            self.get_logger().warn('stepper_scan_ease_zone_deg must be >= 0; forcing to 0.0 deg.')
            self.stepper_scan_ease_zone_deg = 0.0
        self.stepper_scan_ease_zone_rad = math.radians(self.stepper_scan_ease_zone_deg)
        self.stepper_scan_min_speed_scale = max(0.05, min(1.0, self.stepper_scan_min_speed_scale))
        if self.stepper_scan_pi_max_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_pi_max_step_deg must be > 0; forcing to 1.5 deg.')
            self.stepper_scan_pi_max_step_deg = 1.5
        self.stepper_scan_pi_max_step_rad = math.radians(self.stepper_scan_pi_max_step_deg)
        if self.stepper_scan_pi_integral_limit_deg <= 0.0:
            self.get_logger().warn('stepper_scan_pi_integral_limit_deg must be > 0; forcing to 10.0 deg.')
            self.stepper_scan_pi_integral_limit_deg = 10.0
        self.stepper_scan_pi_integral_limit_rad = math.radians(self.stepper_scan_pi_integral_limit_deg)
        if self.stepper_scan_escape_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_escape_step_deg must be > 0; forcing to 2.0 deg.')
            self.stepper_scan_escape_step_deg = 2.0
        self.stepper_scan_escape_step_rad = math.radians(self.stepper_scan_escape_step_deg)
        if self.stepper_scan_drift_limit_deg <= 0.0:
            self.get_logger().warn('stepper_scan_drift_limit_deg must be > 0; forcing to 8.0 deg.')
            self.stepper_scan_drift_limit_deg = 8.0
        self.stepper_scan_drift_limit_rad = math.radians(self.stepper_scan_drift_limit_deg)
        if self.stepper_scan_drift_max_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_drift_max_step_deg must be > 0; forcing to 0.06 deg.')
            self.stepper_scan_drift_max_step_deg = 0.06
        self.stepper_scan_drift_max_step_rad = math.radians(self.stepper_scan_drift_max_step_deg)
        if self.stepper_scan_endpoint_tolerance_deg <= 0.0:
            self.get_logger().warn('stepper_scan_endpoint_tolerance_deg must be > 0; forcing to 0.8 deg.')
            self.stepper_scan_endpoint_tolerance_deg = 0.8
        self.stepper_scan_endpoint_tolerance_rad = math.radians(self.stepper_scan_endpoint_tolerance_deg)
        if self.stepper_scan_endpoint_window_deg <= 0.0:
            self.get_logger().warn('stepper_scan_endpoint_window_deg must be > 0; forcing to 3.0 deg.')
            self.stepper_scan_endpoint_window_deg = 3.0
        self.stepper_scan_endpoint_window_rad = math.radians(self.stepper_scan_endpoint_window_deg)
        if self.stepper_scan_endpoint_max_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_endpoint_max_step_deg must be > 0; forcing to 1.0 deg.')
            self.stepper_scan_endpoint_max_step_deg = 1.0
        self.stepper_scan_endpoint_max_step_rad = math.radians(self.stepper_scan_endpoint_max_step_deg)
        if self.stepper_scan_endpoint_recover_window_deg <= 0.0:
            self.get_logger().warn('stepper_scan_endpoint_recover_window_deg must be > 0; forcing to 0.6 deg.')
            self.stepper_scan_endpoint_recover_window_deg = 0.6
        self.stepper_scan_endpoint_recover_window_rad = math.radians(
            self.stepper_scan_endpoint_recover_window_deg
        )
        if self.stepper_scan_endpoint_recover_step_deg <= 0.0:
            self.get_logger().warn('stepper_scan_endpoint_recover_step_deg must be > 0; forcing to 0.5 deg.')
            self.stepper_scan_endpoint_recover_step_deg = 0.5
        self.stepper_scan_endpoint_recover_step_rad = math.radians(
            self.stepper_scan_endpoint_recover_step_deg
        )
        if self.output_cloud_drift_comp_limit_deg <= 0.0:
            self.get_logger().warn('output_cloud_drift_comp_limit_deg must be > 0; forcing to 12.0 deg.')
            self.output_cloud_drift_comp_limit_deg = 12.0
        self.output_cloud_drift_comp_limit_rad = math.radians(self.output_cloud_drift_comp_limit_deg)
        self.stepper_scan_start_time = time.monotonic()

        self.stepper_steps_per_rad = (
            self.stepper_steps_per_rev * self.stepper_gear_ratio * self.stepper_microsteps
        ) / (2.0 * math.pi)
        self.stepper_steps_per_deg = self.stepper_steps_per_rad * math.pi / 180.0

        # Keep pulse rate safe for the configured driver. For A4988 + this motor setup,
        # the user-reported limit is 500 Hz.
        if self.stepper_max_rate_hz <= 0.0:
            self.stepper_max_rate_hz = 500.0
        self.stepper_max_rate_hz = min(float(self.stepper_max_rate_hz), 500.0)
        min_half_period = 1.0 / (2.0 * self.stepper_max_rate_hz)
        self.stepper_pulse_half_period_s = max(float(self.stepper_step_delay_s), min_half_period)

        self.get_logger().info(
            'Stepper calibration: steps_per_rev=%s, gear_ratio=%.3f, microsteps=%s, '
            'effective_steps_per_deg=%.3f, limits=[%.2f, %.2f] deg. '
            'Using the 28BYJ-48 internal 1:64 gearbox and driver microstep setting.'
            % (
                self.stepper_steps_per_rev,
                self.stepper_gear_ratio,
                self.stepper_microsteps,
                self.stepper_steps_per_deg,
                self.stepper_min_pitch_deg,
                self.stepper_max_pitch_deg,
            )
        )

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.imu_a_sub = self.create_subscription(Imu, imu_a_topic, self.imu_a_callback, 10)
        self.imu_b_sub = self.create_subscription(Imu, imu_b_topic, self.imu_b_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.flat_scan_pub = self.create_publisher(LaserScan, output_flat_scan_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        self.latest_scan = None
        self.imu_a = None
        self.imu_b = None
        self.stepper_target_pitch = self.stepper_home_pitch_rad
        self.stepper_current_pitch = self.stepper_home_pitch_rad
        self.stepper_homed = False
        self.stepper_pending_home = False
        self.stepper_homing_in_progress = False
        self.stepper_homing_lock = threading.Lock()
        self.stepper_scan_direction = 1.0
        self.stepper_scan_target_rel = self.stepper_home_target_rel_pitch_rad
        self.stepper_scan_error_integral = 0.0
        self.latest_relative_pitch = None
        self.relative_pitch_filtered = None
        self.relative_pitch_last_time = None
        self.stepper_scan_prev_target_rel = self.stepper_scan_target_rel
        self.stepper_scan_prev_target_time = time.monotonic()
        self.stepper_scan_target_time = self.stepper_scan_prev_target_time
        self.stepper_scan_rel_bias = 0.0
        self.output_cloud_drift_comp_rad = 0.0
        self.scan_rel_pitch_prev = None
        self.scan_rel_pitch_prev_time = None
        self.scan_rel_pitch_now = None
        self.scan_rel_pitch_now_time = None
        self.use_gpiozero = False  # Will be set to True if gpiozero succeeds
        self.stepper_step = None  # Will be set by _setup_stepper if using gpiozero
        self.stepper_dir = None   # Will be set by _setup_stepper if using gpiozero
        self.stepper_enable = None

        if self.stepper_enabled:
            self._setup_stepper()
            if self.stepper_enabled and int(self.stepper_test_steps) > 0:
                self._run_stepper_self_test(int(self.stepper_test_steps))
            if self.stepper_use_live_zero:
                self.stepper_pending_home = True
                self.get_logger().info('Waiting for IMU data before starting stepper homing.')
                self._maybe_home_stepper()
            else:
                self.stepper_homed = True
                self.stepper_scan_target_rel = self.stepper_home_target_rel_pitch_rad
                self.stepper_scan_error_integral = 0.0
                self.get_logger().info(
                    'Live zero disabled. Using configured home pitch %.3f deg as the scan center.'
                    % math.degrees(self.stepper_home_pitch_rad)
                )
            if self.stepper_scan_enabled:
                self.create_timer(max(0.02, self.stepper_scan_period_s / 200.0), self._stepper_scan_loop)

        self.get_logger().info(
            f'Listening to {scan_topic}, {imu_a_topic}, {imu_b_topic}; '
            f'publishing {output_topic}, {output_flat_scan_topic}, and {pose_topic}'
        )
        self.get_logger().info(
            'Flat scan: enabled=%s, z slice=[%.3f, %.3f] m above ground, pivot_ground_z=%.3f m.'
            % (
                self.flat_scan_enabled,
                self.flat_scan_slice_min_z_m,
                self.flat_scan_slice_max_z_m,
                self.flat_scan_ground_offset_z_m,
            )
        )
        self.get_logger().info(
            'Output frame signs: cloud_pitch=%+.0f pose_pitch=%+.0f cloud_xyz=[%+.0f,%+.0f,%+.0f]. '
            'Legacy output_pitch_sign remains supported as a fallback default.'
            % (
                self.output_cloud_pitch_sign,
                self.output_pose_pitch_sign,
                self.output_cloud_sign_x,
                self.output_cloud_sign_y,
                self.output_cloud_sign_z,
            )
        )
        self.get_logger().info(
            'Output pose offsets (deg): roll=%.2f pitch=%.2f yaw=%.2f.'
            % (
                self.output_pose_roll_offset_deg,
                self.output_pose_pitch_offset_deg,
                self.output_pose_yaw_offset_deg,
            )
        )
        self.get_logger().info(
            'Cloud transform config: pitch_sign=%+.0f, axis_signs=[x=%+.0f, y=%+.0f, z=%+.0f], '
            'offsets(deg)=[roll=%.2f, pitch=%.2f, yaw=%.2f], mount_offset_m=[x=%.3f, y=%.3f, z=%.3f].'
            % (
                self.output_cloud_pitch_sign,
                self.output_cloud_sign_x,
                self.output_cloud_sign_y,
                self.output_cloud_sign_z,
                self.output_cloud_roll_offset_deg,
                self.output_cloud_pitch_offset_deg,
                self.output_cloud_yaw_offset_deg,
                self.output_mount_offset_x_m,
                self.output_mount_offset_y_m,
                self.output_mount_offset_z_m,
            )
        )
        self.get_logger().info(
            'Pose pivot offset in %s frame: [x=%.3f, y=%.3f, z=%.3f] m.'
            % (
                self.reference_frame,
                self.output_pivot_offset_x_m,
                self.output_pivot_offset_y_m,
                self.output_pivot_offset_z_m,
            )
        )

    def _get_float_parameter(self, name, default_value):
        raw_value = self.get_parameter(name).value
        try:
            return float(raw_value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Parameter {name} value {raw_value!r} is not numeric; using default {default_value}.'
            )
            return float(default_value)

    def _sign_from_value(self, value):
        return 1.0 if float(value) >= 0.0 else -1.0

    def _force_stepper_stable(self):
        """Force a stable LOW state on the step/direction pins so the stepper does
        not wander when no software control is active."""
        try:
            if self.use_gpiozero:
                if self.stepper_step is not None:
                    self.stepper_step.off()
                if self.stepper_dir is not None:
                    self.stepper_dir.off()
                if self.stepper_enable is not None:
                    self.stepper_enable.off() if self.stepper_enable_active_low else self.stepper_enable.on()
            elif GPIO is not None:
                GPIO.output(self.stepper_step_pin, GPIO.LOW)
                GPIO.output(self.stepper_dir_pin, GPIO.LOW)
                if self.stepper_enable_pin >= 0:
                    GPIO.output(
                        self.stepper_enable_pin,
                        GPIO.HIGH if self.stepper_enable_active_low else GPIO.LOW,
                    )
        except Exception as e:
            self.get_logger().warn(f'Failed to force the stepper to a stable state: {e}')

    def _setup_stepper(self):
        # Try gpiozero first (Pi 5 compatible), fall back to RPi.GPIO
        if OutputDevice is not None:
            try:
                self.stepper_step = OutputDevice(self.stepper_step_pin, initial_value=False)
                self.stepper_dir = OutputDevice(self.stepper_dir_pin, initial_value=False)
                if self.stepper_enable_pin >= 0:
                    self.stepper_enable = OutputDevice(
                        self.stepper_enable_pin,
                        active_high=not self.stepper_enable_active_low,
                        initial_value=False if self.stepper_enable_active_low else True,
                    )
                self.stepper_step.off()  # Initialize LOW
                self.stepper_dir.off()   # Initialize LOW
                if self.stepper_enable is not None:
                    self.stepper_enable.on()  # Enable driver
                self.use_gpiozero = True
                if self.stepper_enable_pin < 0:
                    self.get_logger().info('Stepper EN pin not used (board exposes STEP+DIR only).')
                self.get_logger().info(
                    f'Stepper enabled on BCM step={self.stepper_step_pin} dir={self.stepper_dir_pin} '
                    f'en={self.stepper_enable_pin} rate<={self.stepper_max_rate_hz:.1f}Hz (gpiozero)'
                )
                return
            except (RuntimeError, Exception) as e:
                self.get_logger().warn(f'gpiozero GPIO setup failed: {e}; trying RPi.GPIO...')
        
        if GPIO is None:
            self.get_logger().warn('RPi.GPIO not available; stepper control disabled.')
            self.stepper_enabled = False
            return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.stepper_step_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.stepper_dir_pin, GPIO.OUT, initial=GPIO.LOW)
            if self.stepper_enable_pin >= 0:
                GPIO.setup(self.stepper_enable_pin, GPIO.OUT)
                # Enable driver at startup (A4988 EN is active-low by default).
                GPIO.output(
                    self.stepper_enable_pin,
                    GPIO.LOW if self.stepper_enable_active_low else GPIO.HIGH,
                )
            self.use_gpiozero = False
            if self.stepper_enable_pin < 0:
                self.get_logger().info('Stepper EN pin not used (board exposes STEP+DIR only).')
            self.get_logger().info(
                f'Stepper enabled on BCM step={self.stepper_step_pin} dir={self.stepper_dir_pin} '
                f'en={self.stepper_enable_pin} rate<={self.stepper_max_rate_hz:.1f}Hz (RPi.GPIO)'
            )
        except (RuntimeError, Exception) as e:
            self.get_logger().warn(f'GPIO hardware not accessible: {e}; stepper control disabled.')
            self.stepper_enabled = False

    def _set_stepper_direction(self, direction):
        if not self.stepper_enabled:
            return

        # The hardware wiring may require the raw direction to be inverted.
        if self.stepper_dir_invert:
            direction *= -1

        if self.use_gpiozero:
            if direction > 0:
                self.stepper_dir.on()
            else:
                self.stepper_dir.off()
        else:
            if GPIO is not None:
                GPIO.output(self.stepper_dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)

    def _step_stepper(self, steps):
        if not self.stepper_enabled or steps == 0:
            return

        direction = 1 if steps > 0 else -1
        self._set_stepper_direction(direction)

        if self.use_gpiozero:
            for _ in range(abs(steps)):
                self.stepper_step.on()
                time.sleep(self.stepper_pulse_half_period_s)
                self.stepper_step.off()
                time.sleep(self.stepper_pulse_half_period_s)
        else:
            if GPIO is not None:
                for _ in range(abs(steps)):
                    GPIO.output(self.stepper_step_pin, GPIO.HIGH)
                    time.sleep(self.stepper_pulse_half_period_s)
                    GPIO.output(self.stepper_step_pin, GPIO.LOW)
                    time.sleep(self.stepper_pulse_half_period_s)

    def _run_stepper_self_test(self, test_steps):
        # A short forward/backward pulse test helps confirm wiring and GPIO control.
        self.get_logger().info(f'Running stepper self-test with {test_steps} steps forward/backward.')
        self._step_stepper(test_steps)
        time.sleep(0.1)
        self._step_stepper(-test_steps)

    def _home_stepper_to_zero(self):
        if not self.stepper_enabled:
            self.get_logger().warn('Stepper homing skipped because stepper is disabled.')
            return

        if self.imu_a is None or self.imu_b is None:
            self.get_logger().warn('Cannot home stepper because IMU A or B is not available.')
            self.stepper_pending_home = True
            return

        target_rel_pitch = self.stepper_home_target_rel_pitch_rad
        integral = 0.0

        self.get_logger().info(
            'Leveling the platform to %.3f deg relative pitch using the robot-body IMU as the reference.'
            % math.degrees(target_rel_pitch)
        )

        for attempt in range(80):
            rel_pitch = self.relative_pitch_rad()
            if rel_pitch is None:
                time.sleep(0.02)
                continue

            error = target_rel_pitch - rel_pitch
            integral += error
            integral = max(-0.25, min(0.25, integral))
            correction = (self.stepper_home_kp * error) + (self.stepper_home_ki * integral)
            correction = max(-self.stepper_home_max_step_rad, min(self.stepper_home_max_step_rad, correction))

            if abs(error) <= self.stepper_min_move_rad:
                self.get_logger().info(
                    f'Platform levelled at attempt {attempt + 1}: relative pitch={math.degrees(rel_pitch):.3f} deg.'
                )
                break

            steps = int(round(correction * self.stepper_steps_per_rad))
            if abs(steps) > 0:
                self._set_stepper_direction(1 if steps > 0 else -1)
                self._step_stepper(steps)
                self.stepper_current_pitch += steps / self.stepper_steps_per_rad

            self.get_logger().info(
                f'Leveling attempt {attempt + 1}: rel_pitch={math.degrees(rel_pitch):.3f} deg, '
                f'error={math.degrees(error):.3f} deg, correction={math.degrees(correction):.3f} deg, '
                f'steps={steps}.'
            )
            time.sleep(0.03)

        final_rel = self.relative_pitch_rad()
        if final_rel is None:
            self.get_logger().warn('Homing failed: could not read final relative pitch from IMUs.')
            self.stepper_pending_home = True
            self.stepper_homed = False
            return

        self.stepper_home_pitch_rad = self.stepper_current_pitch
        self.stepper_home_pitch_deg = math.degrees(self.stepper_home_pitch_rad)
        self.stepper_target_pitch = self.stepper_home_pitch_rad
        self.stepper_scan_target_rel = self.stepper_home_target_rel_pitch_rad
        self.stepper_scan_error_integral = 0.0

        self.get_logger().info(
            'Homing complete. Achieved relative pitch is %.3f deg (target %.3f deg); '
            'mechanical home is %.3f deg; scan limits are %.2f deg to %.2f deg relative to home.'
            % (
                math.degrees(final_rel),
                math.degrees(target_rel_pitch),
                self.stepper_home_pitch_deg,
                self.stepper_min_pitch_deg,
                self.stepper_max_pitch_deg,
            )
        )
        self.stepper_pending_home = False
        self.stepper_homed = True

    def _maybe_home_stepper(self):
        if not self.stepper_enabled:
            return
        if self.stepper_homed:
            return
        if not self.stepper_use_live_zero:
            return
        if not self.stepper_pending_home:
            return
        if self.imu_a is None or self.imu_b is None:
            return

        with self.stepper_homing_lock:
            if self.stepper_homing_in_progress:
                return
            self.stepper_homing_in_progress = True

        self.get_logger().info('IMUs are ready; starting deferred stepper homing worker.')
        threading.Thread(target=self._home_stepper_worker, daemon=True).start()

    def _home_stepper_worker(self):
        try:
            self._home_stepper_to_zero()
        finally:
            with self.stepper_homing_lock:
                self.stepper_homing_in_progress = False

    def _safe_stepper_shutdown(self):
        self.get_logger().info('Putting stepper in a safe shutdown state with stable LOW pins.')

        try:
            if self.stepper_enabled:
                self._command_stepper_to_pitch(self.stepper_home_pitch_rad)
                self.stepper_current_pitch = self.stepper_home_pitch_rad

            self._force_stepper_stable()
            self.stepper_enabled = False
        except Exception as e:
            self.get_logger().warn(f'Stepper safe shutdown failed: {e}')

    def _command_stepper_to_pitch(self, desired_pitch_rad):
        if not self.stepper_enabled:
            self.get_logger().warn('Pitch command ignored because stepper is disabled.')
            return

        lower_bound = self.stepper_home_pitch_rad + self.stepper_min_pitch_rad
        upper_bound = self.stepper_home_pitch_rad + self.stepper_max_pitch_rad
        clamped_target = max(lower_bound, min(upper_bound, desired_pitch_rad))
        pitch_error = clamped_target - self.stepper_current_pitch
        self.get_logger().debug(
            f'Pitch command: desired={math.degrees(desired_pitch_rad):.2f} deg, '
            f'clamped={math.degrees(clamped_target):.2f} deg, '
            f'current={math.degrees(self.stepper_current_pitch):.2f} deg, '
            f'home={math.degrees(self.stepper_home_pitch_rad):.2f} deg, '
            f'lower={math.degrees(lower_bound):.2f} deg, upper={math.degrees(upper_bound):.2f} deg, '
            f'error={math.degrees(pitch_error):.2f} deg.'
        )

        if abs(pitch_error) < self.stepper_min_move_rad:
            self.get_logger().debug(
                f'Pitch change below minimum move threshold ({self.stepper_min_move_rad:.6f} rad); not moving.'
            )
            return

        total_steps_per_rad = self.stepper_steps_per_rad
        steps = int(round(pitch_error * total_steps_per_rad))
        direction = 1 if steps > 0 else -1

        self.get_logger().info(
            f'Commanding stepper move: target={math.degrees(clamped_target):.2f} deg, '
            f'delta={math.degrees(pitch_error):.2f} deg, steps={steps}, direction={direction}, '
            f'gear_ratio={self.stepper_gear_ratio:.3f}, effective_steps_per_rad={total_steps_per_rad:.3f}.'
        )

        if steps != 0:
            self._set_stepper_direction(direction)
            self._step_stepper(steps)
            self.stepper_current_pitch += steps / total_steps_per_rad

        self.stepper_target_pitch = max(
            lower_bound,
            min(upper_bound, self.stepper_current_pitch)
        )
        self.get_logger().info(
            f'Stepper now at {math.degrees(self.stepper_current_pitch):.2f} deg '
            f'(home={math.degrees(self.stepper_home_pitch_rad):.2f} deg; target={math.degrees(self.stepper_target_pitch):.2f} deg).'
        )

    def _stepper_scan_loop(self):
        if (
            not self.stepper_enabled
            or not self.stepper_scan_enabled
            or not self.stepper_homed
            or self.stepper_homing_in_progress
        ):
            return

        now_s = time.monotonic()
        lower_bound = self.stepper_home_pitch_rad + self.stepper_min_pitch_rad
        upper_bound = self.stepper_home_pitch_rad + self.stepper_max_pitch_rad
        lower_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_min_pitch_rad
        upper_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_max_pitch_rad
        rel_pitch = self.relative_pitch_rad()

        # Relative pitch estimate derived from step counts, used as fallback when IMU data is stale.
        stepper_rel_pitch = (
            self.stepper_home_target_rel_pitch_rad
            + (self.stepper_current_pitch - self.stepper_home_pitch_rad)
        )
        rel_feedback = rel_pitch if rel_pitch is not None else stepper_rel_pitch

        # Advance the sweep target at a fixed rate, but reverse only when the measured
        # relative pitch reaches an endpoint band. This prevents premature bottom reversal.
        span_rel = upper_rel_bound - lower_rel_bound
        if span_rel <= 0.0:
            return
        limit_tol = max(self.stepper_min_move_rad, self.stepper_scan_limit_tolerance_rad)
        max_scan_step = max(self.stepper_min_move_rad, self.stepper_scan_step_rad)
        profile_rel = self.stepper_scan_target_rel

        if rel_feedback >= upper_rel_bound - limit_tol and self.stepper_scan_direction > 0.0:
            self.stepper_scan_direction = -1.0
            profile_rel = upper_rel_bound - self.stepper_scan_escape_step_rad
        elif rel_feedback <= lower_rel_bound + limit_tol and self.stepper_scan_direction < 0.0:
            self.stepper_scan_direction = 1.0
            profile_rel = lower_rel_bound + self.stepper_scan_escape_step_rad
        else:
            profile_rel = self.stepper_scan_target_rel + (self.stepper_scan_direction * max_scan_step)
        profile_rel = max(lower_rel_bound, min(upper_rel_bound, profile_rel))

        # Update long-term bias from low-lag IMU error, with stronger correction near limits.
        raw_rel_pitch = self.latest_relative_pitch if self.latest_relative_pitch is not None else rel_pitch
        drift_error = 0.0
        if raw_rel_pitch is not None:
            # Bias tracks measured-minus-stepper offset so positive bias means the real scan
            # is higher than the step-count estimate and the command must be shifted downward.
            drift_error = raw_rel_pitch - stepper_rel_pitch
            drift_deadband = math.radians(0.12)
            if abs(drift_error) > drift_deadband:
                dist_to_edge = min(rel_feedback - lower_rel_bound, upper_rel_bound - rel_feedback)
                edge_zone = max(self.stepper_min_move_rad * 4.0, 0.18 * span_rel)
                edge_gain = 2.2 if dist_to_edge <= edge_zone else 0.5
                effective_error = drift_error - math.copysign(drift_deadband, drift_error)
                bias_delta = edge_gain * self.stepper_scan_drift_k * effective_error
                bias_delta = max(
                    -self.stepper_scan_drift_max_step_rad,
                    min(self.stepper_scan_drift_max_step_rad, bias_delta),
                )
                self.stepper_scan_rel_bias += bias_delta
                self.stepper_scan_rel_bias = max(
                    -self.stepper_scan_drift_limit_rad,
                    min(self.stepper_scan_drift_limit_rad, self.stepper_scan_rel_bias),
                )

        self.stepper_scan_target_rel = profile_rel
        self.stepper_scan_target_rel = max(lower_rel_bound, min(upper_rel_bound, self.stepper_scan_target_rel))

        commanded_rel = self.stepper_scan_target_rel - self.stepper_scan_rel_bias
        commanded_rel = max(lower_rel_bound, min(upper_rel_bound, commanded_rel))
        endpoint_error = 0.0
        endpoint_target_rel = upper_rel_bound if self.stepper_scan_direction > 0.0 else lower_rel_bound
        endpoint_correction_active = False
        endpoint_step = 0.0
        endpoint_recovery_active = False
        endpoint_recovery_step = 0.0
        recovery_target = None

        if self.stepper_scan_endpoint_recover_enabled:
            lower_stall = (
                self.stepper_scan_direction < 0.0
                and rel_feedback > lower_rel_bound + limit_tol
                and stepper_rel_pitch <= lower_rel_bound + self.stepper_scan_endpoint_recover_window_rad
            )
            upper_stall = (
                self.stepper_scan_direction > 0.0
                and rel_feedback < upper_rel_bound - limit_tol
                and stepper_rel_pitch >= upper_rel_bound - self.stepper_scan_endpoint_recover_window_rad
            )
            if lower_stall:
                endpoint_recovery_active = True
                endpoint_recovery_step = self.stepper_scan_endpoint_recover_step_rad
                recovery_target = self.stepper_current_pitch - endpoint_recovery_step
            elif upper_stall:
                endpoint_recovery_active = True
                endpoint_recovery_step = self.stepper_scan_endpoint_recover_step_rad
                recovery_target = self.stepper_current_pitch + endpoint_recovery_step

        if recovery_target is not None:
            target = recovery_target
            recovery_margin = self.stepper_scan_endpoint_recover_step_rad
            target = max(lower_bound - recovery_margin, min(upper_bound + recovery_margin, target))
        else:
            target = self.stepper_home_pitch_rad + (commanded_rel - self.stepper_home_target_rel_pitch_rad)
            target = max(lower_bound, min(upper_bound, target))
        target_delta = target - self.stepper_current_pitch

        # Keep scan-point interpolation state fresh for per-point pitch estimation.
        self.stepper_scan_prev_target_rel = self.stepper_scan_target_rel
        self.stepper_scan_prev_target_time = self.stepper_scan_target_time
        self.stepper_scan_target_time = now_s
        self.scan_rel_pitch_prev = self.scan_rel_pitch_now
        self.scan_rel_pitch_prev_time = self.scan_rel_pitch_now_time
        scan_rel_estimate = stepper_rel_pitch + self.stepper_scan_rel_bias
        if rel_pitch is not None:
            blend = self.stepper_scan_cloud_feedback_blend
            scan_rel_estimate = ((1.0 - blend) * scan_rel_estimate) + (blend * rel_pitch)
        self.scan_rel_pitch_now = max(lower_rel_bound, min(upper_rel_bound, scan_rel_estimate))
        self.scan_rel_pitch_now_time = now_s

        self.get_logger().info(
            f'Stepper scan: direction={self.stepper_scan_direction:+.0f}, '
            f'current={math.degrees(self.stepper_current_pitch):.2f} deg, '
            f'rel_pitch={math.degrees(rel_pitch) if rel_pitch is not None else float("nan"):.2f} deg, '
            f'rel_feedback={math.degrees(rel_feedback):.2f} deg, '
            f'stepper_rel={math.degrees(stepper_rel_pitch):.2f} deg, '
            f'drift_error={math.degrees(drift_error):.2f} deg, '
            f'bias={math.degrees(self.stepper_scan_rel_bias):.2f} deg, '
            f'profile_rel={math.degrees(profile_rel):.2f} deg, '
            f'sweep_target_rel={math.degrees(self.stepper_scan_target_rel):.2f} deg, '
            f'endpoint_target={math.degrees(endpoint_target_rel) if endpoint_target_rel is not None else float("nan"):.2f} deg, '
            f'endpoint_error={math.degrees(endpoint_error):.2f} deg, '
            f'endpoint_step={math.degrees(endpoint_step):.2f} deg, '
            f'endpoint_correction={1 if endpoint_correction_active else 0}, '
            f'endpoint_recovery={1 if endpoint_recovery_active else 0}, '
            f'endpoint_recovery_step={math.degrees(endpoint_recovery_step):.2f} deg, '
            f'commanded_rel={math.degrees(commanded_rel):.2f} deg, '
            f'target_delta={math.degrees(target_delta):.2f} deg, '
            f'limit_tol={math.degrees(limit_tol):.2f} deg, '
            f'lower={math.degrees(lower_bound):.2f} deg, '
            f'upper={math.degrees(upper_bound):.2f} deg, '
            f'command={math.degrees(target):.2f} deg.'
        )
        self._command_stepper_to_pitch(target)

    def imu_a_callback(self, msg):
        self.imu_a = msg
        self._update_relative_pitch_estimate()
        self._maybe_home_stepper()

    def imu_b_callback(self, msg):
        self.imu_b = msg
        self._update_relative_pitch_estimate()
        self._maybe_home_stepper()

    def scan_callback(self, msg):
        if self.imu_a is None or self.imu_b is None:
            self.get_logger().debug('Waiting for both IMU readings before generating point cloud.')
            return

        self._maybe_home_stepper()

        # Always update latest scan - process all available scan data
        self.latest_scan = msg

        self.publish_3d_cloud(msg)

    def _update_relative_pitch_estimate(self):
        if self.imu_a is None or self.imu_b is None:
            return

        rel_pitch = self.imu_pitch_rad(self.imu_a) - self.imu_pitch_rad(self.imu_b)
        self.latest_relative_pitch = rel_pitch

        if self.relative_pitch_filtered is None:
            self.relative_pitch_filtered = rel_pitch
        else:
            alpha = self.relative_pitch_filter_alpha
            self.relative_pitch_filtered = (alpha * rel_pitch) + ((1.0 - alpha) * self.relative_pitch_filtered)
        self.relative_pitch_last_time = time.monotonic()

    def imu_pitch_rad(self, imu_msg):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        # Axis convention: +x is forward, +y is left, +z is up.
        # With this convention, the nose-up pose corresponds to +90 deg.
        # The current sensor mounting reports the opposite sign, so flip it here.
        return -math.atan2(-ax, math.sqrt(ay * ay + az * az))

    def average_relative_pitch_rad(self, samples=10, delay_s=0.02):
        # Non-blocking estimate based on latest IMU updates.
        # Keep function for compatibility with existing call sites.
        _ = samples
        _ = delay_s
        self._update_relative_pitch_estimate()
        return self.relative_pitch_filtered

    def relative_pitch_rad(self):
        if self.imu_a is None or self.imu_b is None:
            self.get_logger().debug('Relative pitch requested but IMU A or B not ready yet.')
            return None
        self._update_relative_pitch_estimate()
        rel_pitch = self.relative_pitch_filtered
        if rel_pitch is None:
            return None
        self.get_logger().debug(
            f'IMU A pitch={math.degrees(self.imu_pitch_rad(self.imu_a)):.2f} deg, '
            f'IMU B pitch={math.degrees(self.imu_pitch_rad(self.imu_b)):.2f} deg, '
            f'relative pitch={math.degrees(rel_pitch):.2f} deg.'
        )
        return rel_pitch

    def _scan_rel_pitch_window(self, scan_msg, rel_pitch_now):
        if rel_pitch_now is None:
            return (None, None)

        point_count = max(1, len(scan_msg.ranges))
        if scan_msg.time_increment > 0.0 and point_count > 1:
            scan_duration = scan_msg.time_increment * float(point_count - 1)
        elif scan_msg.scan_time > 0.0:
            scan_duration = float(scan_msg.scan_time)
        else:
            scan_duration = 0.0

        rel_pitch_start = rel_pitch_now
        rel_pitch_end = rel_pitch_now
        prev_pitch = self.scan_rel_pitch_prev
        prev_time = self.scan_rel_pitch_prev_time
        now_pitch = self.scan_rel_pitch_now
        now_time = self.scan_rel_pitch_now_time
        if (
            scan_duration > 0.0
            and prev_pitch is not None
            and prev_time is not None
            and now_pitch is not None
            and now_time is not None
        ):
            dt = now_time - prev_time
            if dt > 1e-4:
                rel_rate = (now_pitch - prev_pitch) / dt
                rel_pitch_end = rel_pitch_now
                rel_pitch_start = rel_pitch_end - (rel_rate * scan_duration)

        lower_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_min_pitch_rad
        upper_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_max_pitch_rad
        rel_pitch_start = max(lower_rel_bound, min(upper_rel_bound, rel_pitch_start))
        rel_pitch_end = max(lower_rel_bound, min(upper_rel_bound, rel_pitch_end))
        return (rel_pitch_start, rel_pitch_end)

    def _compensate_cloud_rel_pitch(self, rel_pitch_nominal, rel_pitch_measured):
        if rel_pitch_nominal is None:
            return rel_pitch_measured
        if rel_pitch_measured is None:
            return rel_pitch_nominal

        if self.output_cloud_drift_comp_enabled:
            drift_error = rel_pitch_nominal - rel_pitch_measured
            self.output_cloud_drift_comp_rad += self.output_cloud_drift_comp_k * drift_error
            self.output_cloud_drift_comp_rad = max(
                -self.output_cloud_drift_comp_limit_rad,
                min(self.output_cloud_drift_comp_limit_rad, self.output_cloud_drift_comp_rad),
            )
            corrected_rel_pitch = rel_pitch_nominal - self.output_cloud_drift_comp_rad
        else:
            corrected_rel_pitch = rel_pitch_nominal

        lower_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_min_pitch_rad
        upper_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_max_pitch_rad
        return max(lower_rel_bound, min(upper_rel_bound, corrected_rel_pitch))

    def build_cloud_from_scan(self, scan_msg, rel_pitch=None, rel_pitch_start=None, rel_pitch_end=None):
        if rel_pitch is None:
            rel_pitch = self.relative_pitch_rad()
        if rel_pitch is None:
            return []

        def rotation_for_pitch(local_rel_pitch):
            roll = self.output_cloud_roll_offset_rad
            pitch = (self.output_cloud_pitch_sign * local_rel_pitch) + self.output_cloud_pitch_offset_rad
            yaw = self.output_cloud_yaw_offset_rad

            cy = math.cos(yaw)
            sy = math.sin(yaw)
            cr = math.cos(roll)
            sr = math.sin(roll)
            cp = math.cos(pitch)
            sp = math.sin(pitch)

            return (
                cy * cp,
                cy * sp * sr - sy * cr,
                cy * sp * cr + sy * sr,
                sy * cp,
                sy * sp * sr + cy * cr,
                sy * sp * cr - cy * sr,
                -sp,
                cp * sr,
                cp * cr,
            )

        mount_offset = (
            self.output_mount_offset_x_m,
            self.output_mount_offset_y_m,
            self.output_mount_offset_z_m,
        )

        if rel_pitch_start is None:
            rel_pitch_start = rel_pitch
        if rel_pitch_end is None:
            rel_pitch_end = rel_pitch
        point_count = max(1, len(scan_msg.ranges))
        varying_pitch = point_count > 1 and abs(rel_pitch_end - rel_pitch_start) > 1e-6

        if not varying_pitch:
            rot = rotation_for_pitch(rel_pitch)

        points = []
        for i, rng in enumerate(scan_msg.ranges):
            if not math.isfinite(rng):
                continue
            if rng < scan_msg.range_min or rng > scan_msg.range_max:
                continue

            if varying_pitch:
                alpha = float(i) / float(point_count - 1)
                point_rel_pitch = rel_pitch_start + ((rel_pitch_end - rel_pitch_start) * alpha)
                rot = rotation_for_pitch(point_rel_pitch)
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = rng * math.cos(angle)
            y = rng * math.sin(angle)
            z = 0.0

            x += mount_offset[0]
            y += mount_offset[1]
            z += mount_offset[2]

            x_rot = r00 * x + r01 * y + r02 * z
            y_rot = r10 * x + r11 * y + r12 * z
            z_rot = r20 * x + r21 * y + r22 * z
            points.append((
                self.output_cloud_sign_x * x_rot,
                self.output_cloud_sign_y * y_rot,
                self.output_cloud_sign_z * z_rot,
            ))

        return points

    def publish_pose(self, scan_msg, rel_pitch=None):
        if rel_pitch is None:
            rel_pitch = self.relative_pitch_rad()
        if rel_pitch is None:
            return

        pose = PoseStamped()
        pose.header.stamp = scan_msg.header.stamp
        pose.header.frame_id = self.reference_frame

        # Laser frame is pitched relative to the base frame.
        roll = self.output_pose_roll_offset_rad
        pitch = (self.output_pose_pitch_sign * rel_pitch) + self.output_pose_pitch_offset_rad
        yaw = self.output_pose_yaw_offset_rad

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

        mount_offset_x = self.output_mount_offset_x_m
        mount_offset_y = self.output_mount_offset_y_m
        mount_offset_z = self.output_mount_offset_z_m

        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        r00 = cy * cp
        r01 = cy * sp * sr - sy * cr
        r02 = cy * sp * cr + sy * sr
        r10 = sy * cp
        r11 = sy * sp * sr + cy * cr
        r12 = sy * sp * cr - cy * sr
        r20 = -sp
        r21 = cp * sr
        r22 = cp * cr

        pose.pose.position.x = (
            self.output_pivot_offset_x_m
            + r00 * mount_offset_x
            + r01 * mount_offset_y
            + r02 * mount_offset_z
        )
        pose.pose.position.y = (
            self.output_pivot_offset_y_m
            + r10 * mount_offset_x
            + r11 * mount_offset_y
            + r12 * mount_offset_z
        )
        pose.pose.position.z = (
            self.output_pivot_offset_z_m
            + r20 * mount_offset_x
            + r21 * mount_offset_y
            + r22 * mount_offset_z
        )

        self.pose_pub.publish(pose)

    def publish_flat_scan(self, scan_msg, points):
        if not self.flat_scan_enabled:
            return

        flat_scan = LaserScan()
        flat_scan.header.stamp = scan_msg.header.stamp
        flat_scan.header.frame_id = self.frame_id
        flat_scan.angle_min = scan_msg.angle_min
        flat_scan.angle_max = scan_msg.angle_max
        flat_scan.angle_increment = scan_msg.angle_increment
        flat_scan.time_increment = scan_msg.time_increment
        flat_scan.scan_time = scan_msg.scan_time
        flat_scan.range_min = scan_msg.range_min
        flat_scan.range_max = scan_msg.range_max

        if flat_scan.angle_increment == 0.0 or flat_scan.angle_max <= flat_scan.angle_min:
            return

        beam_count = int(round((flat_scan.angle_max - flat_scan.angle_min) / flat_scan.angle_increment)) + 1
        if beam_count <= 0:
            return

        ranges = [float('inf')] * beam_count
        for x, y, z in points:
            z_above_ground = z + self.flat_scan_ground_offset_z_m
            if z_above_ground < self.flat_scan_slice_min_z_m or z_above_ground > self.flat_scan_slice_max_z_m:
                continue

            angle = math.atan2(y, x)
            if angle < flat_scan.angle_min or angle > flat_scan.angle_max:
                continue

            beam = int(round((angle - flat_scan.angle_min) / flat_scan.angle_increment))
            if beam < 0 or beam >= beam_count:
                continue

            rng = math.hypot(x, y)
            if rng < flat_scan.range_min or rng > flat_scan.range_max:
                continue

            if rng < ranges[beam]:
                ranges[beam] = rng

        flat_scan.ranges = ranges
        flat_scan.intensities = [0.0] * beam_count
        self.flat_scan_pub.publish(flat_scan)

    def publish_3d_cloud(self, scan_msg):
        rel_pitch_measured = self.relative_pitch_rad()
        rel_pitch_nominal = rel_pitch_measured
        if self.stepper_scan_enabled and self.scan_rel_pitch_now is not None:
            rel_pitch_nominal = self.scan_rel_pitch_now

        rel_pitch = self._compensate_cloud_rel_pitch(rel_pitch_nominal, rel_pitch_measured)
        rel_pitch_start, rel_pitch_end = self._scan_rel_pitch_window(scan_msg, rel_pitch_nominal)
        if rel_pitch_start is not None and rel_pitch_end is not None:
            rel_pitch_start -= self.output_cloud_drift_comp_rad
            rel_pitch_end -= self.output_cloud_drift_comp_rad
            lower_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_min_pitch_rad
            upper_rel_bound = self.stepper_home_target_rel_pitch_rad + self.stepper_max_pitch_rad
            rel_pitch_start = max(lower_rel_bound, min(upper_rel_bound, rel_pitch_start))
            rel_pitch_end = max(lower_rel_bound, min(upper_rel_bound, rel_pitch_end))
        if self.stepper_scan_enabled:
            self.get_logger().info(
                f'Continuous scan active: stepper pitch={math.degrees(self.stepper_current_pitch):.2f} deg; '
                f'relative measured={math.degrees(rel_pitch_measured) if rel_pitch_measured is not None else float("nan"):.2f} deg; '
                f'relative nominal={math.degrees(rel_pitch_nominal) if rel_pitch_nominal is not None else float("nan"):.2f} deg; '
                f'relative pitch={math.degrees(rel_pitch) if rel_pitch is not None else float("nan"):.2f} deg; '
                f'scan pitch window=[{math.degrees(rel_pitch_start) if rel_pitch_start is not None else float("nan"):.2f}, '
                f'{math.degrees(rel_pitch_end) if rel_pitch_end is not None else float("nan"):.2f}] deg; '
                f'cloud drift comp={math.degrees(self.output_cloud_drift_comp_rad):.2f} deg; '
                f'relative limits=[{self.stepper_min_pitch_deg:.2f}, {self.stepper_max_pitch_deg:.2f}] deg; '
                f'home offset={math.degrees(self.stepper_home_pitch_rad):.2f} deg.'
            )
        elif rel_pitch is not None:
            clamped_rel_pitch = max(self.stepper_min_pitch_rad, min(self.stepper_max_pitch_rad, rel_pitch))
            reference_pitch = self.stepper_home_pitch_rad
            if self.imu_b is not None:
                reference_pitch = self.imu_pitch_rad(self.imu_b)
            absolute_target = reference_pitch + clamped_rel_pitch
            self.get_logger().info(
                f'Publishing point cloud for scan; relative pitch={math.degrees(rel_pitch):.2f} deg, '
                f'clamped relative pitch={math.degrees(clamped_rel_pitch):.2f} deg, '
                f'absolute target={math.degrees(absolute_target):.2f} deg, '
                f'static imu={math.degrees(reference_pitch):.2f} deg.'
            )
            self.stepper_target_pitch = absolute_target
            self._command_stepper_to_pitch(self.stepper_target_pitch)
        else:
            self.get_logger().warn('Skipping point cloud publish because relative pitch is unavailable (IMUs not ready).')

        points = self.build_cloud_from_scan(
            scan_msg,
            rel_pitch=rel_pitch,
            rel_pitch_start=rel_pitch_start,
            rel_pitch_end=rel_pitch_end,
        )
        if not points:
            return

        header = Header()
        header.stamp = scan_msg.header.stamp
        header.frame_id = self.frame_id

        point_cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(point_cloud)
        self.publish_flat_scan(scan_msg, points)
        self.publish_pose(scan_msg, rel_pitch=rel_pitch)

    def __del__(self):
        try:
            self._safe_stepper_shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DCloudNode()

    def safe_exit_handler():
        try:
            node._safe_stepper_shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    atexit.register(safe_exit_handler)

    def handle_sigint(signum, frame):
        node.get_logger().warning('Received interrupt; forcing stepper to safe idle state.')
        safe_exit_handler()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        safe_exit_handler()


if __name__ == '__main__':
    main()
