#!/usr/bin/env python3

import atexit
import math
import signal
import time

import rclpy
from rclpy.node import Node

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
        self.declare_parameter('stepper_enable_pin', -1)
        self.declare_parameter('stepper_enable_active_low', True)
        self.declare_parameter('stepper_dir_invert', False)
        self.declare_parameter('stepper_test_steps', 0)
        self.declare_parameter('stepper_steps_per_rev', 200)
        self.declare_parameter('stepper_gear_ratio', 1.0)
        self.declare_parameter('stepper_microsteps', 1)
        self.declare_parameter('stepper_step_delay_s', 0.0005)
        self.declare_parameter('stepper_max_rate_hz', 500.0)
        self.declare_parameter('stepper_min_move_rad', 0.002)
        self.declare_parameter('stepper_scan_enabled', True)
        self.declare_parameter('stepper_scan_period_s', 6.0)
        self.declare_parameter('stepper_min_pitch_deg', -45.0)
        self.declare_parameter('stepper_max_pitch_deg', 45.0)
        self.declare_parameter('stepper_home_pitch_deg', 0.0)

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
        self.stepper_enable_pin = self.get_parameter('stepper_enable_pin').value
        self.stepper_enable_active_low = self.get_parameter('stepper_enable_active_low').value
        self.stepper_dir_invert = self.get_parameter('stepper_dir_invert').value
        self.stepper_test_steps = self.get_parameter('stepper_test_steps').value
        self.stepper_steps_per_rev = self.get_parameter('stepper_steps_per_rev').value
        self.stepper_gear_ratio = self.get_parameter('stepper_gear_ratio').value
        self.stepper_microsteps = self.get_parameter('stepper_microsteps').value
        self.stepper_step_delay_s = self.get_parameter('stepper_step_delay_s').value
        self.stepper_max_rate_hz = self.get_parameter('stepper_max_rate_hz').value
        self.stepper_min_move_rad = self.get_parameter('stepper_min_move_rad').value
        self.stepper_scan_enabled = self.get_parameter('stepper_scan_enabled').value
        self.stepper_scan_period_s = self.get_parameter('stepper_scan_period_s').value
        self.stepper_min_pitch_deg = self.get_parameter('stepper_min_pitch_deg').value
        self.stepper_max_pitch_deg = self.get_parameter('stepper_max_pitch_deg').value
        self.stepper_home_pitch_deg = self.get_parameter('stepper_home_pitch_deg').value
        self.stepper_min_pitch_rad = math.radians(float(self.stepper_min_pitch_deg))
        self.stepper_max_pitch_rad = math.radians(float(self.stepper_max_pitch_deg))
        self.stepper_home_pitch_rad = math.radians(float(self.stepper_home_pitch_deg))
        self.stepper_scan_start_time = time.monotonic()

        # Keep pulse rate safe for the configured driver. For A4988 + this motor setup,
        # the user-reported limit is 500 Hz.
        if self.stepper_max_rate_hz <= 0.0:
            self.stepper_max_rate_hz = 500.0
        self.stepper_max_rate_hz = min(float(self.stepper_max_rate_hz), 500.0)
        min_half_period = 1.0 / (2.0 * self.stepper_max_rate_hz)
        self.stepper_pulse_half_period_s = max(float(self.stepper_step_delay_s), min_half_period)

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
        self.stepper_homed = False
        self.use_gpiozero = False  # Will be set to True if gpiozero succeeds
        self.stepper_step = None  # Will be set by _setup_stepper if using gpiozero
        self.stepper_dir = None   # Will be set by _setup_stepper if using gpiozero
        self.stepper_enable = None

        if self.stepper_enabled:
            self._setup_stepper()
            if self.stepper_enabled and int(self.stepper_test_steps) > 0:
                self._run_stepper_self_test(int(self.stepper_test_steps))
            self._home_stepper_to_zero()
            if self.stepper_scan_enabled:
                self.create_timer(max(0.02, self.stepper_scan_period_s / 200.0), self._stepper_scan_loop)

        self.get_logger().info(
            f'Listening to {scan_topic}, {imu_a_topic}, {imu_b_topic}; '
            f'publishing {output_topic} and {pose_topic}'
        )

    def _setup_stepper(self):
        # Try gpiozero first (Pi 5 compatible), fall back to RPi.GPIO
        if OutputDevice is not None:
            try:
                self.stepper_step = OutputDevice(self.stepper_step_pin)
                self.stepper_dir = OutputDevice(self.stepper_dir_pin)
                if self.stepper_enable_pin >= 0:
                    self.stepper_enable = OutputDevice(
                        self.stepper_enable_pin,
                        active_high=not self.stepper_enable_active_low,
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

        self._set_stepper_direction(1 if steps > 0 else -1)
        
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

        self.stepper_target_pitch = self.stepper_home_pitch_rad
        self.get_logger().info(
            'Homing stepper to the configured physical zero offset before scan operation. '
            'Relative pitch zero is mapped to stepper home %.2f deg; scan limits are %.2f deg to %.2f deg.'
            % (self.stepper_home_pitch_deg, self.stepper_min_pitch_deg, self.stepper_max_pitch_deg)
        )

        home_attempt = 0
        max_home_attempts = 10
        while abs(self.stepper_current_pitch - self.stepper_home_pitch_rad) > self.stepper_min_move_rad:
            home_attempt += 1
            self.get_logger().info(
                f'Home attempt {home_attempt}/{max_home_attempts}: '
                f'current={math.degrees(self.stepper_current_pitch):.2f} deg, '
                f'target={math.degrees(self.stepper_home_pitch_rad):.2f} deg.'
            )
            self._command_stepper_to_pitch(self.stepper_home_pitch_rad)

            if abs(self.stepper_current_pitch - self.stepper_home_pitch_rad) <= self.stepper_min_move_rad:
                break

            if home_attempt >= max_home_attempts:
                self.get_logger().error(
                    'Stepper did not reach the configured home pitch after '
                    f'{max_home_attempts} attempts; current={self.stepper_current_pitch:.6f} rad, '
                    f'home={self.stepper_home_pitch_rad:.6f} rad.'
                )
                break

            time.sleep(0.1)

        self.stepper_homed = abs(self.stepper_current_pitch - self.stepper_home_pitch_rad) <= self.stepper_min_move_rad
        self.get_logger().info(
            f'Stepper homing complete? {self.stepper_homed}; current pitch={math.degrees(self.stepper_current_pitch):.2f} deg, '
            f'target pitch={math.degrees(self.stepper_home_pitch_rad):.2f} deg.'
        )

    def _safe_stepper_shutdown(self):
        if not self.stepper_enabled:
            return

        self.get_logger().info(
            f'Putting stepper in safe shutdown state at home pitch {self.stepper_home_pitch_rad:.4f} rad.'
        )

        try:
            self._command_stepper_to_pitch(self.stepper_home_pitch_rad)
            self.stepper_current_pitch = self.stepper_home_pitch_rad

            if self.use_gpiozero and self.stepper_step is not None:
                self.stepper_step.off()
            elif GPIO is not None:
                GPIO.output(self.stepper_step_pin, GPIO.LOW)
                GPIO.output(self.stepper_dir_pin, GPIO.LOW)

            if self.stepper_enable_pin >= 0:
                if self.use_gpiozero and self.stepper_enable is not None:
                    self.stepper_enable.off()
                elif GPIO is not None:
                    GPIO.output(
                        self.stepper_enable_pin,
                        GPIO.HIGH if self.stepper_enable_active_low else GPIO.LOW,
                    )

            self.stepper_enabled = False
        except Exception as e:
            self.get_logger().warn(f'Stepper safe shutdown failed: {e}')

    def _command_stepper_to_pitch(self, desired_pitch_rad):
        if not self.stepper_enabled:
            self.get_logger().warn('Pitch command ignored because stepper is disabled.')
            return

        clamped_target = max(self.stepper_min_pitch_rad, min(self.stepper_max_pitch_rad, desired_pitch_rad))
        pitch_error = clamped_target - self.stepper_current_pitch
        self.get_logger().debug(
            f'Pitch command: desired={math.degrees(desired_pitch_rad):.2f} deg, '
            f'clamped={math.degrees(clamped_target):.2f} deg, '
            f'current={math.degrees(self.stepper_current_pitch):.2f} deg, '
            f'error={math.degrees(pitch_error):.2f} deg.'
        )

        if abs(pitch_error) < self.stepper_min_move_rad:
            self.get_logger().debug(
                f'Pitch change below minimum move threshold ({self.stepper_min_move_rad:.6f} rad); not moving.'
            )
            return

        total_steps_per_rad = (self.stepper_steps_per_rev * self.stepper_gear_ratio * self.stepper_microsteps) / (2.0 * math.pi)
        steps = int(round(pitch_error * total_steps_per_rad))

        self.get_logger().info(
            f'Commanding stepper move: target={math.degrees(clamped_target):.2f} deg, '
            f'delta={math.degrees(pitch_error):.2f} deg, steps={steps}.'
        )

        if steps != 0:
            self._step_stepper(steps)
            self.stepper_current_pitch += steps / total_steps_per_rad

        self.stepper_target_pitch = max(self.stepper_min_pitch_rad, min(self.stepper_max_pitch_rad, self.stepper_current_pitch))
        self.get_logger().info(
            f'Stepper now at {math.degrees(self.stepper_current_pitch):.2f} deg '
            f'(target {math.degrees(self.stepper_target_pitch):.2f} deg).'
        )

    def _stepper_scan_loop(self):
        if not self.stepper_enabled or not self.stepper_scan_enabled:
            return

        elapsed = time.monotonic() - self.stepper_scan_start_time
        phase = (elapsed % self.stepper_scan_period_s) / self.stepper_scan_period_s
        if phase < 0.5:
            span = phase * 2.0
            target = self.stepper_min_pitch_rad + span * (self.stepper_max_pitch_rad - self.stepper_min_pitch_rad)
        else:
            span = (phase - 0.5) * 2.0
            target = self.stepper_max_pitch_rad - span * (self.stepper_max_pitch_rad - self.stepper_min_pitch_rad)

        self.get_logger().info(
            f'Stepper scan target={math.degrees(target):.2f} deg, '
            f'current={math.degrees(self.stepper_current_pitch):.2f} deg, '
            f'period={self.stepper_scan_period_s:.2f}s.'
        )
        self._command_stepper_to_pitch(target)

    def imu_a_callback(self, msg):
        self.imu_a = msg

    def imu_b_callback(self, msg):
        self.imu_b = msg

    def scan_callback(self, msg):
        if self.imu_a is None or self.imu_b is None:
            self.get_logger().debug('Waiting for both IMU readings before generating point cloud.')
            return

        # Always update latest scan - process all available scan data
        self.latest_scan = msg

        self.publish_3d_cloud(msg)

    def imu_pitch_rad(self, imu_msg):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        # Axis convention: +x is forward, +y is left, +z is up.
        # For a nose-up platform, gravity appears as a negative x acceleration,
        # so the pitch equation below returns positive pitch for nose-up motion.
        return math.atan2(-ax, math.sqrt(ay * ay + az * az))

    def relative_pitch_rad(self):
        if self.imu_a is None or self.imu_b is None:
            self.get_logger().debug('Relative pitch requested but IMU A or B not ready yet.')
            return None
        rel_pitch = self.imu_pitch_rad(self.imu_a) - self.imu_pitch_rad(self.imu_b)
        self.get_logger().debug(
            f'IMU A pitch={math.degrees(self.imu_pitch_rad(self.imu_a)):.2f} deg, '
            f'IMU B pitch={math.degrees(self.imu_pitch_rad(self.imu_b)):.2f} deg, '
            f'relative pitch={math.degrees(rel_pitch):.2f} deg.'
        )
        return rel_pitch

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
        if self.stepper_scan_enabled:
            self.get_logger().info(
                f'Continuous scan active: stepper pitch={math.degrees(self.stepper_current_pitch):.2f} deg; '
                f'limits=[{self.stepper_min_pitch_deg:.2f}, {self.stepper_max_pitch_deg:.2f}] deg.'
            )
        elif rel_pitch is not None:
            clamped_rel_pitch = max(self.stepper_min_pitch_rad, min(self.stepper_max_pitch_rad, rel_pitch))
            self.get_logger().info(
                f'Publishing point cloud for scan; relative pitch={math.degrees(rel_pitch):.2f} deg, '
                f'clamped pitch={math.degrees(clamped_rel_pitch):.2f} deg.'
            )
            self.stepper_target_pitch = clamped_rel_pitch
            self._command_stepper_to_pitch(self.stepper_target_pitch)
        else:
            self.get_logger().warn('Skipping point cloud publish because relative pitch is unavailable (IMUs not ready).')

        points = self.build_cloud_from_scan(scan_msg)
        if not points:
            return

        header = Header()
        header.stamp = scan_msg.header.stamp
        header.frame_id = self.frame_id

        point_cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(point_cloud)
        self.publish_pose(scan_msg)

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
