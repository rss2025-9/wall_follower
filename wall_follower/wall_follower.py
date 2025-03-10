#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools

import math 

# Helper functions 
def angle_to_index(angle, scan):
    # get an index of scan from a desired angle 
    clamped_angle = max(scan.angle_min, min(angle, scan.angle_max))
    idx = int(round((clamped_angle - scan.angle_min) / scan.angle_increment))
    # idx = max(0, min(idx, len(scan.ranges) - 1))
    return idx

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

# Is this ever used???
def ransac(points, num_iterations = 4, threshold = 0.1, min_inlier_ratio = 0.5): 
    iterations = 0 
    best_inlier_count = 0
    best_fit = None
    num_points = len(points)
    while iterations < num_iterations: 
        if num_points < 2:
            return None, 0
        indices = np.random.choice(num_points, 2, replace=False)
        p1, p2 = points[indices[0]], points[indices[1]]
        # Find line (x1, y1) and (x2, y2) that fits the data
        A = p2[1] - p1[1]
        B = p1[0] - p2[0]
        C = p2[0]*p1[1] - p1[0]*p2[1]
        # Compute distance of each point to the line
        distances = np.abs(A * points[:, 0] + B * points[:, 1] + C) / np.sqrt(A**2 + B**2)
        inliers = distances < threshold
        inlier_count = np.sum(inliers)

        if inlier_count > best_inlier_count and inlier_count > min_inlier_ratio * num_points:
            best_inlier_count = inlier_count
            best_fit = (A, B, C)
        iterations += 1
    return best_fit, best_inlier_count

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")

        # ROS parameters 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 0.5)
        self.declare_parameter('using_real_car', True)
        self.declare_parameter('min_turn_radius', 0.5) # min turn radius of car in meters

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        #self.SIDE = 1 #-1 right
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value 
        self.RADIUS = self.get_parameter('min_turn_radius').get_parameter_value().double_value 
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        # self.add_on_set_parameters_callback(self.parameters_callback)
  
        # Subscribers and Publishers
        self.can_subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 1)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)

        # publisher for the visualization of the wall follower
        self.view_publisher = self.create_publisher(LaserScan, "view_topic", 1)
        self.wall_publisher = self.create_publisher(Marker, "wall_topic", 1)
        self.steer_publisher = self.create_publisher(Marker, "steer_topic", 1)


        # variables to store previous results 
        self.prev_error = 0
        self.integral = 0
        self.prev_time = self.get_clock().now()
        self.prev_line = None

        # PID control variables
        self.Kp = 1
        self.Ki = 0.0
        self.Kd = 0.5

    # Callback function for the scan subscriber
    def scan_callback(self, scan):  

        # Print messages =================================================================
        # self.get_logger().info("scan min angle: " + str(scan.angle_min))
        # self.get_logger().info("scan max angle: " + str(scan.angle_max))
        # self.get_logger().info("scan angle increment: " + str(scan.angle_increment))

        # Publish what the robot is seeing ============================================================
        view_msg = LaserScan()
        view_msg.header = scan.header
        # view_msg.angle_min = scan.angle_min
        # view_msg.angle_max = scan.angle_max
        view_msg.angle_increment = scan.angle_increment
        view_msg.time_increment = scan.time_increment
        view_msg.scan_time = scan.scan_time
        view_msg.range_min = scan.range_min
        view_msg.range_max = scan.range_max - 1.0
        # view_msg.ranges = scan.ranges
        view_msg.intensities = scan.intensities

        # Slice up the scan data in half depending on side of wall we are following ==========================
        # side = 1 means we are following the left wall, -1 means we are following the right wall

        if self.SIDE == -1:
            start_angle = -np.pi*5/8
            end_angle = np.pi/8
        else:
            start_angle = -np.pi/8
            end_angle = np.pi*5/8
            

        start_idx = angle_to_index(start_angle, scan)
        end_idx = angle_to_index(end_angle, scan)

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        
        view_msg.angle_min = start_angle 
        view_msg.angle_max = end_angle
        view_msg.ranges = scan.ranges[start_idx:end_idx]
        self.view_publisher.publish(view_msg)

        ranges = np.array(scan.ranges[start_idx:end_idx])
        indices = np.arange(start_idx, end_idx)
        angles = scan.angle_min + indices * scan.angle_increment
        mask = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max) & (angles >= scan.angle_min) & (angles <= scan.angle_max)
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) == 0:
            distance = self.DESIRED_DISTANCE
            angle = 0
        else: 
            # distance = np.mean(valid_ranges)
            x, y = polar_to_cartesian(valid_ranges, valid_angles)
            slope, intercept = np.polyfit(x, y, 1, w = 1 + (1/valid_ranges))
            VisualizationTools.plot_line(x, slope * x + intercept, self.wall_publisher, frame="/laser")
            distance = abs(intercept)/math.sqrt(1 + slope**2)
            angle = np.arctan(slope)
            # angle = 0

        # checking if there's a wall upcoming at front 
        # mid_start = angle_to_index(-math.pi/15, scan)
        # mid_end = angle_to_index(math.pi/15, scan)
        # mid_ranges = np.array(scan.ranges[mid_start: mid_end])
        # valid_mid_ranges = np.isfinite(mid_ranges) 

        # turn_offset = 0
        # if len(valid_mid_ranges) != 0: 
        #     if self.VELOCITY <= 1 and np.mean(valid_mid_ranges) < 0.75: 
        #         turn_offset = 0.25
        #     elif self.VELOCITY <= 2 and np.mean(valid_mid_ranges) < 1.25: 
        #         turn_offset = 0.3
        #     elif self.VELOCITY <= 3 and np.mean(valid_mid_ranges) < 2: 
        #         turn_offset = 0.4 
        
        # Attempt to implementing RANSAC :( ==================================================
        # points = []
        # for r, theta in zip(valid_ranges, valid_angles):
        #     x, y = polar_to_cartesian(r, theta)
        #     points.append([x, y])
        # points = np.array(points)

        # line, num_inliers = ransac(points, num_iterations = 4, threshold = 0.1, min_inlier_ratio = 0.5)
        # if num_inliers == 0: 
        #     self.get_logger().info("No inliers found")
        #     if self.prev_line is not None: 
        #         A, B, C = self.prev_line
        #         distance = abs(C) / np.sqrt(A**2 + B**2)
        #     else: 
        #         distance = np.mean(valid_ranges) if len(valid_ranges) > 0 else self.DESIRED_DISTANCE
        # else: 
        #     A, B, C = line 
        #     distance = abs(C) / np.sqrt(A**2 + B**2)
        #     self.prev_line = line
        
        # if line is not None: 
        #     x_min = np.min(points[:, 0])
        #     x_max = np.max(points[:, 0])
        #     x = np.linspace(x_min, x_max, num=20) 
        #     y = (-A * x - C) / B 
        #     VisualizationTools.plot_line(x, y, self.wall_publisher, frame="base_link")

        error = (self.DESIRED_DISTANCE) - distance

        # Compute time difference ======================================================
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            dt = 1e-6  # avoid division by zero

        # PID ================================================================================
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative * (0.75 + 0.25*error/(error + self.RADIUS))
        steering_angle = -self.SIDE * control + angle


        VisualizationTools.plot_line([0.0, np.cos(steering_angle)], [0.0, np.sin(steering_angle)], self.steer_publisher, frame="/laser", color = (1.0, 1.0, 0.0))
        self.get_logger().info(f"Steering={steering_angle*180/np.pi}, Error = {error} m")



        # update for next iteration 
        self.prev_error = error 
        self.prev_time = current_time

        # Publish drive message ==================================================
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.VELOCITY
        self.drive_publisher.publish(drive_msg) 

        # print desired vs results 
        # self.get_logger().info(f"Side: {self.SIDE}")
        # self.get_logger().info(f"Desired distance: {self.DESIRED_DISTANCE}, Actual distance: {distance}, Error: {error}, Steering angle: {steering_angle}")
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()