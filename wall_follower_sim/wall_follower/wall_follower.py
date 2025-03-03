#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")

        #ADDED gains
        self.kp=1.0 #3 (12)
        self.kd=0.5 #1.5 (4)
        #ki (10)


        self.prev_error = 0.0

        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/ackermann_cmd")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)


        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        #self.SIDE = 1 #-1 right
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        # TODO: Initialize your publishers and subscribers here

        # TODO: Write your callback functions here    

	    # TODO: Initialize your publishers and subscribers here
         #ADDED subscribe to lidar data (subscibe to SCN_TOPIC), listens for messages of type LaserScan, when msg arrives lidar_callback called
        self.subscription=self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 1)
        self.subscription 
        #ADDED publish drive,  publishes msgs of type AckermannDriveStamped to topic DRIVE_TOPIC
        self.drive_pub=self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, "wall", 1)



    # TODO: Write your callback functions here   
    #ADDED
    def lidar_callback(self, msg):
        if not msg.ranges: #no lidar data received
            self.get_logger().warn("No LIDAR data")
            return

        #to np array
        msg_arr=np.array(msg.ranges)
        angles=np.linspace(msg.angle_min, msg.angle_max, len(msg_arr))

        #angle range
        ang_min=-np.pi/8 if self.SIDE==1 else -np.pi*3/8
        ang_max=np.pi*3/8 if self.SIDE==1 else np.pi/8

        #indices of this range
        mask=(angles>=ang_min) & (angles<=ang_max)
        selected_angles=angles[mask]
        selected_ranges=msg_arr[mask]

        #get rid of bad readings
        #valid_mask=(selected_ranges>msg.range_min) & (selected_ranges<msg.range_max) 
        valid_mask=(selected_ranges>msg.range_min) & (selected_ranges<4.0) 
        selected_angles=selected_angles[valid_mask]
        selected_ranges=selected_ranges[valid_mask]

        if len(selected_ranges) < 2:
            self.get_logger().warn("not enough data")
            return

        #to cartesian
        x_vals=selected_ranges*np.cos(selected_angles)
        y_vals=selected_ranges*np.sin(selected_angles)

        #fit line
        slope, y_int=np.polyfit(x_vals, y_vals, 1)

        #show hypothesized wall
        x = np.linspace(-2., 2., num=20)
        y = slope*x +y_int
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")



        #perp dist from car to line
        dist_to_wall=abs(y_int)/np.sqrt(slope**2+1)

        #find error
        error=self.DESIRED_DISTANCE-dist_to_wall #FLIP?
    

        #compute derivative
        dt=max(msg.scan_time, 0.05)  #so no /0
        der=(error-self.prev_error)/dt

        #regular PD control
        if self.SIDE==-1: #if right side
            steer_ang=self.kp*error+self.kd*slope 
        else:
            steer_ang=-(self.kp*error+self.kd*-slope)


        self.publish_drive_command(steer_ang)
        self.prev_error=error




    #ADDED
    def publish_drive_command(self, steer_ang):
        #max_steer_ang=0.5  #something reasonable
        #steer_ang=np.clip(steer_ang, -max_steer_ang, max_steer_ang) #limits range of steering ang to [-max,max]

        

        msg=AckermannDriveStamped() #message of type AckermannDriveStamped()
        #set
        msg.drive.speed=self.VELOCITY
        msg.drive.steering_angle=steer_ang
        #publish msg (steering ang an speed) to /drive topic
        self.drive_pub.publish(msg)


 

    
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
    



