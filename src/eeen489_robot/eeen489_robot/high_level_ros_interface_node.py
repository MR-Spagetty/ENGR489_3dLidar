# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import transforms3d
import time
import math

class localOdometryPublisher(Node):
    """
    : Description: This node wants to convert the encoder information published on a twist message into a odometry message. It also wants to set the odom -> base_link transform which allows the robot to function with the navigation stack. Using NAV2 allows us to use SLAM and other advanced algorithms.
    """
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.subsciption = self.create_subscription(
            Twist,
            '/encoder',
            self.listener_callback,
            10
        )
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self) 
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.time_now = time.time()
        self.time_before = self.time_now
        

    def listener_callback(self,encoder_msg):
        self.get_logger().info('Encoder: Vx=%0.2f m.s^-1 Vy=%0.2f m.s^-1 Wz=%0.2f rad.s^-1'%(encoder_msg.linear.x,encoder_msg.linear.y,encoder_msg.angular.z))
        odom_msg = Odometry()
        ## An odometry message contains
		# Header header
		#Header header
        self.time_now = time.time()
		## Two-integer timestamp that is expressed as:
		# stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
			# stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
        odom_msg.header.stamp.sec = round(self.time_now) # I don't think we want decimals here
        odom_msg.header.stamp.nanosec = round((self.time_now%1) *1e9) # we just want the values up to one million
        #odom_msg.header.stamp = self.get_clock().now().to_msg() # clock values
			## Frame this data is associated with
			# string frame_id
        odom_msg.header.frame_id = 'base_link'
		# string child_frame_id
        odom_msg.child_frame_id = 'odom' # I don't think we want to call this map for now
            # The integral of velocity is position
        odom_msg.twist.twist.linear.x = encoder_msg.linear.x
        odom_msg.twist.twist.linear.y = encoder_msg.linear.y
        odom_msg.twist.twist.linear.z = encoder_msg.linear.z
        odom_msg.twist.twist.angular.x = encoder_msg.angular.x
        odom_msg.twist.twist.angular.y = encoder_msg.angular.y
        odom_msg.twist.twist.angular.z = encoder_msg.angular.z
        odom_msg.twist.covariance = [1.0,0.0,0.0,0.0,0.0,0.0, 
		    0.0,1.0,0.0,0.0,0.0,0.0, 
			0.0,0.0,1.0,0.0,0.0,0.0, 
			0.0,0.0,0.0,1.0,0.0,0.0, 
			0.0,0.0,0.0,0.0,1.0,0.0, 
			0.0,0.0,0.0,0.0,0.0,1.0]
            # set acceleration to zero as an approximation or do linear fit between odom values
        # Swap the values of linear.x and linear.y in the equations
        odom_msg.pose.pose.position.x = self.x + (odom_msg.twist.twist.linear.x *math.cos(self.yaw) -odom_msg.twist.twist.linear.y*math.sin(self.yaw)) * (self.time_now - self.time_before)
        odom_msg.pose.pose.position.y = self.y + (odom_msg.twist.twist.linear.y *math.cos(self.yaw) +odom_msg.twist.twist.linear.x*math.sin(self.yaw))  * (self.time_now - self.time_before)
        odom_msg.pose.pose.position.z = 0.0
        # As we rotate about the z-axis
        self.yaw = self.yaw + odom_msg.twist.twist.angular.z * (self.time_now - self.time_before) # Wz * dt where dt = this timestamp - previous timestamp
        # orientation in quaternion
        [w, x, y, z] = transforms3d.euler.euler2quat(0,0,self.yaw)
        odom_msg.pose.pose.orientation.x = x
        odom_msg.pose.pose.orientation.y = y
        odom_msg.pose.pose.orientation.z = z
        odom_msg.pose.pose.orientation.w = w           
        # if odom_msg.header.seq == 0:
		# 	# first message zero the position
        #     odom_msg.pose.pose.position.x = 0
        #     odom_msg.pose.pose.position.y = 0
        #     odom_msg.pose.pose.position.z = 0
        #     odom_msg.pose.pose.orientation.x = 0
        #     odom_msg.pose.pose.orientation.y = 0
        #     odom_msg.pose.pose.orientation.z = 0
        #     odom_msg.pose.pose.orientation.w = 1            
        # else:			
        #     odom_msg.pose.pose.position.x = self.prev_odom.pose.pose.position.x + odom_msg.twist.twist.linear.x * (odom_msg.header.stamp.nsecs - self.prev_odom.header.stamp.nsecs) # Vx * dt where dt = this timestamp - previous timestamp 
		# 			# float64 y
        #     odom_msg.pose.position.y = odom_msg.pose.position.y + odom_msg.twist.linear.y * (odom_msg.header.stamp.nsecs - self.prev_odom.header.stamp.nsecs) # Vy * dt where dt = this timestamp - previous timestamp
		# 			# float64 z
        #     # odom_msg.pose.position.z = odom_msg.pose.position.z + odom_msg.twist.angular.z * (odom_msg.header.stamp.nsecs - self.last_pose.header.stamp.nsecs) # Wz * dt where dt = this timestamp - previous timestamp
        #     self.yaw = self.yaw + odom_msg.twist.angular.z * (odom_msg.header.stamp.nsecs - self.prev_odom.header.stamp.nsecs) # Wz * dt where dt = this timestamp - previous timestamp
        #     		# Quaternion orientation   
        #     #odom_msg.pose.orientation = quaternion_from_euler(0,0,yaw) # roll and pitch are zero - we are just in 2D
        #     # Use transforms 3d instead
        #     [w, x, y, z] = transform3d.euler.euler2quat(0,0,self.yaw)
        #     odom_msg.pose.orientation.x = x
        #     odom_msg.pose.orientation.y = y
        #     odom_msg.pose.orientation.z = z
        #     odom_msg.pose.orientation.w = w
        odom_msg.pose.covariance = [1.0,0.0,0.0,0.0,0.0,0.0,
		    0.0,1.0,0.0,0.0,0.0,0.0, 
			0.0,0.0,1.0,0.0,0.0,0.0,
			0.0,0.0,0.0,1.0,0.0,0.0,
			0.0,0.0,0.0,0.0,1.0,0.0, 
			0.0,0.0,0.0,0.0,0.0,1.0]
        # Create the transform between odom -> base_link
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = odom_msg.pose.pose.position.x #swapped
        t.transform.translation.y = odom_msg.pose.pose.position.y #swapped
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # publish odom_raw
        self.publisher_.publish(odom_msg)
        # Update th = e variables
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.yaw = self.yaw % (2*math.pi)
        self.time_before = self.time_now

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = localOdometryPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
