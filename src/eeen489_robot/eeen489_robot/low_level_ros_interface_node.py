import math
import time
import os
import numpy as np
from geometry_msgs.msg import Twist
from roboclaw_3 import Roboclaw
import rclpy
from rclpy.node import Node

from motorClass import MotorClass
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class localTwistSubscriber(Node):

	def __init__(self):
		super().__init__('local_twist_subscriber')
		# # Roboclaw Parameters
		roboclaw_device_descriptor = ParameterDescriptor(description='The Roboclaw serial bus address.')
		self.declare_parameter('roboclaw.device', '/dev/ttyACM0', roboclaw_device_descriptor)
		self.roboclaw_device = self.get_parameter('roboclaw.device').get_parameter_value().string_value
		roboclaw_baud_descriptor = ParameterDescriptor(description='The Roboclaw serial bus baud rate.')
		self.declare_parameter('roboclaw.baud', 38400 , roboclaw_baud_descriptor)
		self.roboclaw_baud = self.get_parameter('roboclaw.baud').get_parameter_value().integer_value
		roboclaw_timeout_descriptor = ParameterDescriptor(description='The roboclaw serial command timeout in seconds.')
		self.declare_parameter('roboclaw.timeout', 30, roboclaw_timeout_descriptor)
		self.roboclaw_timeout = self.get_parameter('roboclaw.timeout').get_parameter_value().integer_value
		roboclaw_retries_descriptor = ParameterDescriptor(description='The number of times to retry a roboclaw serial command.')
		self.declare_parameter('roboclaw.retries', 3, roboclaw_retries_descriptor)
		self.roboclaw_retries = self.get_parameter('roboclaw.retries').get_parameter_value().integer_value
		
		# # Motor Parameters
		motor_ppr_descriptor = ParameterDescriptor(description='The number of encoders pulses per wheel revolution.')
		self.declare_parameter('motor.ppr', 537.7 , motor_ppr_descriptor)
		self.motor_ppr = self.get_parameter('motor.ppr').get_parameter_value().double_value

		# # Robot Platform Parameters
		robot_lengthX_descriptor = ParameterDescriptor(description='The distance in metres between the centre of the robot and the centre of the mecanum wheels in the x-direction.')
		self.declare_parameter('robot.lengthX', 0.1680, robot_lengthX_descriptor)
		self.robot_lengthX = self.get_parameter('robot.lengthX').get_parameter_value().double_value
		robot_lengthY_descriptor = ParameterDescriptor(description='The distance in metres between the centre of the robot and the centre of the mecanum wheels in the y-direction.')
		self.declare_parameter('robot.lengthY', 0.2075, robot_lengthY_descriptor)
		self.robot_lengthY = self.get_parameter('robot.lengthY').get_parameter_value().double_value
		robot_wheelDiameter_descriptor = ParameterDescriptor(description='The distance in metres of the wheel diameter.')
		self.declare_parameter('robot.wheelDiameter', 0.0960, robot_wheelDiameter_descriptor)
		self.robot_wheelDiameter = self.get_parameter('robot.wheelDiameter').get_parameter_value().double_value

		# # Enable the set parameter callback to change parameters
		# 
		
		#self.wheel_diameter = 96/1000;
		#self.wheel_width = 38/1000;
		#self.platform_width = 453/1000;
		#self.platform_length = 432/1000;
		# specify the robot width in metres
		#self.length_y = (self.platform_width - self.wheel_width)/2;
		# specify the robot length in metres
		#self.length_x = (self.platform_length - self.wheel_diameter)/2;
		self.mc = MotorClass()
		# self.mc = MotorClass(self.roboclaw_device, self.roboclaw_baud, self.roboclaw_timeout, self.roboclaw_retries, self.robot_lengthX, self.robot_lengthY , self.robot_wheelDiameter, self.motor_ppr)
		#self.roboclaw = Roboclarcw('/dev/ttyAMA0',38400)
		#self.roboclaw.Open()
		self.subscription = self.create_subscription(
			Twist,
			'/cmd_vel',
			self.listener_callback,
			10)
		self.subscription # prevent unused variable warning
		self.publisher_ = self.create_publisher(Twist,'/encoder', 10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.add_on_set_parameters_callback(self.parameter_callback)

	def parameter_callback(self, params):
		for param in params:
			if param.name == 'roboclaw.device' and param.type_ == Parameter.Type.STRING:
				# Test if the motordriver operates on new device.
				self.mc = MotorClass(param.value, self.roboclaw_baud, self.roboclaw_timeout, self.roboclaw_retries, self.robot_lengthX, self.robot_lengthY , self.robot_wheelDiameter, self.motor_ppr)
				self.roboclaw_device = param.value
			if param.name == 'roboclaw.baud' and param.type_ == Parameter.Type.INTEGER:
				self.roboclaw_baud = param.value
		return SetParametersResult(successful=True)
	def listener_callback(self,msg):
		# self.get_logger().info('I heard: [%.2f(Vx) %.2f(Vy) %.2f(Wz)] '%( msg.linear.x, msg.linear.y, msg.angular.z))
		self.mc.setVelocity(msg.linear.x,msg.linear.y,msg.angular.z)
		#[w1,w2,w3,w4] = self.forward_kinematics(msg.linear.x,msg.linear.y,msg.angular.z) has no attribute forward kinomatics
		[n,s1,s2,s3,s4]=self.mc.encoders()
		self.get_logger().info('Motor speeds: [%.2f(s1) %.2f(s2 %.2f(s3) %.2fs4]'%(s1,s2,s3,s4))
		#[s1,s2,s3,s4] = self.angularToSpeed(w1,w2,w3,w4)	
		#self.get_logger().info('{} {} {} {}'.format( s1, s2, s3,s4))
		# 	
		#self.motor_movement(s1,s2,s3,s4)
		#self.motor_movement(100,100,100,100) # test encoder routine
	def timer_callback(self):
		#my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
		# self.get_logger().info('Hello %s!' % my_param)
		# self.get_parameter('my_parameter').
		msg = Twist()
		# Get the speeds, returns 3 element tuple (e, s, d)
		# where e => error, 1 success 0 failure
		# s => speed, signed int value
		# d => direction, 0 = forward, 1 = backwards
		# [err,msg.linear.x, msg.linear.y, msg.angular.z] = self.rc.getVelocity()
		[err,msg.linear.x, msg.linear.y, msg.angular.z] = self.mc.getVelocity()
		if not err:
			# something went wrong
			self.get_logger().info('Could not obtain motor speeds.{}'.format(err))
			return
		self.publisher_.publish(msg)
		self.get_logger().info('Motor speeds: [%.2f(Vx) %.2f(Vy) %.2f(Wz)]'%(msg.linear.x,msg.linear.y,msg.angular.z))
		# encoderVelocity1 = self.roboclaw.ReadSpeedM1(0x80)
		# encoderVelocity2 = self.roboclaw.ReadSpeedM2(0x80)
		# encoderVelocity3 = self.roboclaw.ReadSpeedM1(0x81)
		# encoderVelocity4 = self.roboclaw.ReadSpeedM2(0x81)
		# if (encoderVelocity1[0] & encoderVelocity2[0] & encoderVelocity3[0] & encoderVelocity4[0]):
		#[e1, s1, d1] = self.roboclaw.ReadSpeedM1(0x80)
		#[e3, s3, d3] = self.roboclaw.ReadSpeedM2(0x80)
		#[e2, s2, d2] = self.roboclaw.ReadSpeedM1(0x81)
		#[e4, s4, d4] = self.roboclaw.ReadSpeedM2(0x81)
		#[w1,w2,w3,w4] = self.speedToAngular(s1,s2,s3,s4)
		# 	# Translate values
		#	# w = speedToAngular(s)
		#[msg.linear.x, msg.linear.y, msg.angular.z] = self.inverse_kinematics(w1,w2,w3,w4)
		# 	# output speed
		# 	self.publisher_.publish(msg)
		#self.get_logger().info('Motor speeds: [%.2f(Vx) %.2f(Vy) %.2f(Wz)]'%(msg.linear.x,msg.linear.y,msg.angular.z))
		#self.get_logger().info('Motor speeds: %d %d %d %d' %
		# self.roboclaw.ReadSpeedM1(0x80),self.roboclaw.ReadSpeedM2(0x80),
		# self.roboclaw.ReadSpeedM1(0x81),self.roboclaw.ReadSpeedM2(0x81))
def main(args=None):
	rclpy.init(args=args)
	time.sleep(1)
	local_twist_subscriber = localTwistSubscriber()
	# should we put one use functions here before we spin the node
	# we want to do one use functions after parameters have been loaded in 

	rclpy.spin(local_twist_subscriber)
	
	# Destroy the node explicitly
	local_twist_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

