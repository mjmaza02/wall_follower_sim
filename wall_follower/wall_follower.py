#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
	# TODO: Initialize your publishers and subscribers here
        self.create_subscription(Float32, '/distance_to_wall', self.listener_callback, 10)
        self.current_dist = 0
        self.prev_time = self.get_clock().now()
        self.prev_error = 0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        timer_callback = 0.05
        self.create_timer(timer_callback, self.timer_callback)

    # TODO: Write your callback functions here   

    def listener_callback(self, msg):
        self.current_dist = msg.data
        # self.get_logger().info(f'Current Distance: {msg.data}')

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = self.PD_control(self.current_dist)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = self.VELOCITY
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.drive_pub.publish(msg)

    def PD_control(self, current_dist):
        Kp = 3
        Kd = 1
        current_time = self.get_clock().now()
        dt = float((current_time-self.prev_time).nanoseconds * 10**-9) # proof of concept
        self.prev_time = current_time

        error = self.DESIRED_DISTANCE - current_dist

        de_dt = (error-self.prev_error)/dt

        corrected = Kp*error + Kd*de_dt
        self.prev_error = error

        # self.get_logger().info(f'P: {error}, D: {de_dt}')
        return corrected




def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
