#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

from wall_follower.visualization_tools import VisualizationTools


class ScanParse(Node):

    def __init__(self):
        super().__init__("scan_parse")
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
        self.scan_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.scan_sub
        self.line_pub = self.create_publisher(Marker, "/wall", 1)
        self.dist_pub = self.create_publisher(Float32, "/distance_to_wall", 10)

    # TODO: Write your callback functions here   
    def listener_callback(self, msg):
        range_len = len(msg.ranges)
        msg_ranges = msg.ranges
        msg_angles = np.linspace(msg.angle_min, msg.angle_max, range_len)

        side_ranges = []
        side_angles = []

        if self.SIDE == 1:
            side_ranges = msg_ranges[range_len//2:]
            side_angles = msg_angles[range_len//2:]
        elif self.SIDE == -1:
            side_ranges = msg_ranges[:range_len//2]
            side_angles = msg_angles[:range_len//2]

        xs = side_ranges * np.cos(side_angles)
        ys = side_ranges * np.sin(side_angles)

        slope, intercept = np.polyfit(xs, ys, 1)

        dist_msg = Float32()
        dist_msg.data = abs(intercept)/(slope**2+1)**(1/2)
        self.dist_pub.publish(dist_msg)

        x = [float(i) for i in range(5)]
        y = [(slope*xi +intercept) for xi in x]
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

def main():
    
    rclpy.init()
    scan_parse = ScanParse()
    rclpy.spin(scan_parse)
    scan_parse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
