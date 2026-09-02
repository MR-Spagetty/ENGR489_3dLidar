import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# This code takes in rawscan and flips the axis to output to scan
class LidarSwapNode(Node):
    def __init__(self):
        super().__init__('lidar_swap_node')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'rawscan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        swapped_scan = LaserScan()
        swapped_scan.header = msg.header
        swapped_scan.angle_min = msg.angle_min
        swapped_scan.angle_max = msg.angle_max
        swapped_scan.angle_increment = msg.angle_increment
        swapped_scan.time_increment = msg.time_increment
        swapped_scan.scan_time = msg.scan_time
        swapped_scan.range_min = msg.range_min
        swapped_scan.range_max = msg.range_max
        
        # Swap x and y axes in ranges and intensities
        swapped_scan.ranges = [msg.ranges[i] for i in range(len(msg.ranges)-1, -1, -1)]
        swapped_scan.intensities = [msg.intensities[i] for i in range(len(msg.intensities)-1, -1, -1)]
        
        self.publisher_.publish(swapped_scan)

def main(args=None):
    rclpy.init(args=args)

    lidar_swap_node = LidarSwapNode()

    rclpy.spin(lidar_swap_node)

    lidar_swap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
