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
        self.current_dist = 0
        self.prev_time = self.get_clock().now()
        self.prev_error = 0
        self.total_error = 0

        self.scan_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.scan_sub

        self.line_pub = self.create_publisher(Marker, "/wall", 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        timer_callback = 0.05
        self.create_timer(timer_callback, self.timer_callback)

    # TODO: Write your callback functions here   

    def listener_callback(self, msg:LaserScan):
        distance_filter = 5 * self.VELOCITY
        # angle_filter = (np.pi/4)//msg.angle_increment
        angle_filter = 0
        range_len = len(msg.ranges)
        msg_ranges = msg.ranges
        msg_angles = np.linspace(msg.angle_min, msg.angle_max, range_len)

        side_ranges = []
        side_angles = []

        if self.SIDE == 1:
            side_ranges = msg_ranges[int(range_len//2 - angle_filter):]
            side_angles = msg_angles[int(range_len//2 - angle_filter):]
        elif self.SIDE == -1:
            side_ranges = msg_ranges[:int(range_len//2 + angle_filter)]
            side_angles = msg_angles[:int(range_len//2 + angle_filter)]


        filtered_ranges, filtered_angles = self.filter_ranges_angles(side_ranges, side_angles, distance_filter*self.DESIRED_DISTANCE)
        # filtered_ranges, filtered_angles = side_ranges, side_angles
        xs = filtered_ranges * np.cos(filtered_angles)
        ys = filtered_ranges * np.sin(filtered_angles)
        # xs = side_ranges * np.cos(side_angles)
        # ys = side_ranges * np.sin(side_angles)

        if len(xs) > 0:
            slope, intercept = np.polyfit(xs, ys, 1)

            self.current_dist = abs(intercept)/(slope**2+1)**(1/2)

            x = [float(i) for i in range(round(distance_filter))]
            y = [(slope*xi +intercept) for xi in x]
            VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

        else:
            self.current_dist = self.DESIRED_DISTANCE

    def timer_callback(self):

        # DO NOT DELETE
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        msg = AckermannDriveStamped()
        steering_angle = self.PID_control(self.current_dist)
        msg.drive.steering_angle = -1 * self.SIDE * steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = self.VELOCITY * max(0.5, 1-0.1*steering_angle**2)
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.drive_pub.publish(msg)

    # Helpers
    def filter_ranges_angles(self, range_data, angle_data, range_filter):
        assert len(range_data) == len(angle_data)

        filtered_range = []
        filtered_angle = []

        for i in range(len(range_data)):
            if range_data[i] < range_filter:
                filtered_range.append(range_data[i])
                filtered_angle.append(angle_data[i])

        return filtered_range, filtered_angle

    def PID_control(self, current_dist):
        Kp = 0.3
        Ki = 0
        Kd = 1
        current_time = self.get_clock().now()
        dt = float((current_time-self.prev_time).nanoseconds * 10**-9) # proof of concept
        self.prev_time = current_time

        error = self.DESIRED_DISTANCE - current_dist
        integral_e = self.total_error + error*dt
        de_dt = (error-self.prev_error)/dt

        corrected = Kp*error + Ki*integral_e + Kd*de_dt
        self.prev_error = error
        self.total_error += error*dt
        self.get_logger().info(f'Current Distance: {current_dist}, Current Goal: {self.DESIRED_DISTANCE}, Current Output: {corrected}, Side: {self.SIDE}')

        return corrected




def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
