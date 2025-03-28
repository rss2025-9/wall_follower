#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import os 
import csv

class RunStraight(Node):

    def __init__(self):
        super().__init__("run_straight")

        # ROS Parameters 
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("velocity", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        # self.add_on_set_parameters_callback(self.parameters_callback)

        #ADDED publish drive,  publishes msgs of type AckermannDriveStamped to topic DRIVE_TOPIC
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.publish_drive_command(0.0))
        self.drive_pub=self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)

    def publish_drive_command(self, steer_ang):
        msg=AckermannDriveStamped() #message of type AckermannDriveStamped()
        #set
        msg.drive.speed=self.VELOCITY
        msg.drive.steering_angle=steer_ang
        #publish msg (steering ang an speed) to /drive topic
        self.drive_pub.publish(msg)
    
    # def parameters_callback(self, params):
    #     """
    #     DO NOT MODIFY THIS CALLBACK FUNCTION!
        
    #     This is used by the test cases to modify the parameters during testing. 
    #     It's called whenever a parameter is set via 'ros2 param set'.
    #     """
    #     for param in params:
    #         if param.name == 'side':
    #             self.SIDE = param.value
    #             self.get_logger().info(f"Updated side to {self.SIDE}")
    #         elif param.name == 'velocity':
    #             self.VELOCITY = param.value
    #             self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
    #         elif param.name == 'desired_distance':
    #             self.DESIRED_DISTANCE = param.value
    #             self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
    #     return SetParametersResult(successful=True)


def main():
    rclpy.init()
    run_straight = RunStraight()
    rclpy.spin(run_straight)
    run_straight.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()