import rclpy
from rclpy.node import Node
import time
import os
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        # Setting Parameters (distance from wall, car length, velocity, error, queue size, Ts)
        self.QUEUE_SIZE = 10
        self.car_length = 1.5	# projection distance we project car forward. 
        self.vel = 1.5 		# used for pid_vel (not much use).
        self.error = 0.0
        self.dist_from_wall = 0.8

        # Call controller
        self.create_timer(self.Ts, self.controller)

        #TODO: set PID gains
        self.kp = -0.5
        self.kd = 0.000
        self.ki = 0

        # # TODO: store history
        self.integral = []
        self.prev_error = []
        self.error = []

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.laser_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, self.QUEUE_SIZE)
        self.laser_subscriber

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

        # Lidar info
        self.lidar_properties_set = False
        self.default_viewing_angle = 360
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270
        self.range_max = None
        self.range_min = None
        self.num_scans = None
        self.angle_increment = None
        self.angle_min_radians = None
        self.angle_max_radians = None
        self.scan_ranges = None

        self.declare_parameters(
            namespace='',
            parameters=[
                ('viewing_angle', self.default_viewing_angle),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle),
                ('live_laser_feed', 0),
                ('Ts', 0.05)
            ])
        
        # Update ROS parameters from config/launch file
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value
        self.live_laser_feed = self.get_parameter('live_laser_feed').value
        self.Ts = self.get_parameter('Ts').value # controller sample time

        # Print ROS parameters
        self.get_logger().info(
            f'\n viewing_angle: {self.viewing_angle}'
            f'\n front_degree_angle: {self.front_degree_angle}'
            f'\n right_degree_angle: {self.right_degree_angle}'
            f'\n left_degree_angle: {self.left_degree_angle}'
            f'\n live_laser_feed: {self.live_laser_feed}'
            f'\n Ts: {self.Ts}'
        )

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        radians_angle = np.deg2rad(angle)
        index = (radians_angle - range_data.angle_min) // range_data.angle_increment

        if np.isinf(range_data[index]) or np.isnan(range_data[index]):
            return 100.0
        
        return range_data[index]

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        dist_in_front = self.get_range(msg, self.front_degree_angle)
        theta = 30

        dist_in_left = self.get_range(msg, self.left_degree_angle)
        dist_in_right = self.get_range(msg, self.right_degree_angle)

        alpha_left = self.get_range(msg, self.left_degree_angle + theta)
        alpha_right = self.get_range(msg, self.right_degree_angle - theta)

        #Calculate future and present distance from right wall
        alpha_r = math.atan( (alpha_right * math.cos(theta) - dist_in_right)/ alpha_right * math.sin(theta) )
        curr_pos_r = dist_in_right * math.cos(alpha_r) # Present Position
        fut_pos_r = curr_pos_r + self.car_length * math.sin(alpha_r) # projection in Future Position

        #Calculate future and present distance from left wall
        alpha_l = math.atan( (alpha_left * math.cos(theta) - dist_in_left)/ alpha_left * math.sin(theta) )
        curr_pos_l = dist_in_left * math.cos(alpha_l)
        fut_pos_l = curr_pos_l + self.car_length * math.sin(alpha_l)

        #Calculate error
        error = - (fut_pos_r - fut_pos_l)

        #Publish drive message
        self.pid_control(error, self.vel)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()