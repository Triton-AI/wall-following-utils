import time
import os

import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

class ANGLE_TYPE(Enum):
    _0To360 = 0
    _n180To180 = 1

def angle_mod(angle_deg, type):
    output_angle = angle_deg
    if type == ANGLE_TYPE._0To360:
        if angle_deg > 360:
            output_angle = angle_deg - 360
        elif angle_deg < 0:
            output_angle = angle_deg + 360        
    elif type == ANGLE_TYPE._n180To180:
        if angle_deg > 180:
            output_angle = angle_deg - 360
        elif angle_deg < -180:
            output_angle = angle_deg + 360
    else:
        return None
    return output_angle

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('WallFollow')

        # declare params that can be passed by "wall_follower_config.yaml"
        self.declare_parameters(
            namespace='',
            parameters=[
                ('live_laser_feed', 0),
                ('frame_id', 'base_link'),
                ('laserscan_topic', '/scan'),
                ('drive_topic', '/drive'),
                ('max_vel', 1.0),
                ('kP', 0.5),
                ('kI', 0.0),
                ('kD', 0.0),
                ('laserscan_phase_deg', 0.0),
                ('scan_angle_type', 0), # 0 is 0_to_360, 1 is -180_to_180
                ('default_lidar_angle_unit','rad'),
                ('Ts', 0.02)
            ])
        
        # Update ROS parameters from config/launch file
        self.live_laser_feed = self.get_parameter('live_laser_feed').value
        self.Ts = self.get_parameter('Ts').value # controller sampling time

        # TODO: set PID gains
        self.kp = self.get_parameter('kP').value
        self.ki = self.get_parameter('kI').value
        self.kd = self.get_parameter('kD').value

        # TODO: store history
        self.integral_error = 0
        self.prev_error = 0
        self.error = 0

        # Print ROS parameters
        self.get_logger().info(
            f'\n live_laser_feed: {self.live_laser_feed}'
            f'\n PID: {self.kp, self.ki, self.kd}'
            f'\n Ts: {self.Ts}'
        )

        # Setting Parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.QUEUE_SIZE = 10
        # self.car_length = 1.5	# projection distance we project car forward. 
        self.max_vel = self.get_parameter('max_vel').value 	# used for pid_vel (not much use).

        self.dist_from_wall = 0.8

        lidarscan_topic =  self.get_parameter('laserscan_topic').value
        drive_topic =  self.get_parameter('drive_topic').value

        # TODO: create publishers and subscribers
        self.laser_subscriber = self.create_subscription(LaserScan,
                                                         lidarscan_topic, 
                                                         self.scan_callback, 
                                                         QoSProfile(depth=self.QUEUE_SIZE, 
                                                                    reliability=ReliabilityPolicy.RELIABLE)
                                                         )
        self.laser_subscriber

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 
                                                     drive_topic, 
                                                     self.QUEUE_SIZE)

        self.current_time = self.get_clock().now().to_msg()

        # Drive-related params
        self.drive_cmd = AckermannDriveStamped()
        self.drive_cmd.header.frame_id = self.frame_id
        self.drive_cmd.header.stamp = self.current_time
        self.prev_drive_cmd = self.drive_cmd

        # Lidar info
        self.lidar_properties_set = False
        self.default_viewing_angle = 360
        self.phase_deg = self.get_parameter('laserscan_phase_deg').value
        self.phase_rad = np.deg2rad(self.phase_deg)
        self.angle_type = self.get_parameter('scan_angle_type').value
        self.angle_unit = self.get_parameter('default_lidar_angle_unit').value

        self.front_deg = angle_mod(self.phase_deg, self.angle_type)
        self.left_deg = angle_mod(self.phase_deg + 90, self.angle_type)
        self.right_deg = angle_mod(self.phase_deg - 90, self.angle_type)
        self.rear_deg = angle_mod(self.phase_deg + 180, self.angle_type)

        self.num_scans = None
        self.scan = LaserScan()

        # TODO: store any necessary values you think you'll need


        # Start a timer
        self.timer = self.create_timer(self.Ts, self.periodic)

    def get_range(self, range_data, angle_deg):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        angle_deg = angle_mod(angle_deg, self.angle_type)
        angle_rad = np.deg2rad(angle_deg)
        index = (angle_rad - range_data.angle_min) // range_data.angle_increment

        if np.isinf(range_data[index]) or np.isnan(range_data[index]):
            return float('inf')
        
        return range_data[index]

    def pid_control(self):
        """
        Based on the calculated error, publish vehicle control

        Args:
            desired_vel: Desired Velocity

        Returns:
            None
        """
        angle = 0.0

        # TODO: Use kp, ki & kd to implement a PID controller
        if self.ki != 0.0:
            self.integral_error += self.error

        p_term = self.kp * self.error
        i_term = self.ki * self.integral_error
        d_term = self.kd * (self.error - self.prev_error) / self.Ts
        desired_steering = p_term + i_term + d_term

        if np.abs(desired_steering) < 10:
            desired_vel = self.max_vel
        elif np.abs(desired_steering) < 20:
            desired_vel = 0.66 * self.max_vel
        else:
            desired_vel = 0.33 *self.max_vel
     
        # TODO: fill in drive message and publish
        self.prev_drive_cmd = self.drive_cmd
        self.drive_cmd.header.stamp = self.current_time
        self.drive_cmd.drive.steering_angle = desired_steering
        self.drive_cmd.drive.speed = desired_vel

        # Publish the drive_cmd values to the drive topic
        self.drive_publisher.publish(self.drive_cmd)


    def laserscan_angle_unit_conv(self, msg):
        """
        Callback function for LaserScan messages. Convert the angle unit to radians.

        Args:
            msg: Incoming LaserScan message

        Returns:
            output_msg: in radians
        """
        output_msg = msg
        if self.angle_unit == 'deg':
            output_msg.angle_min = np.deg2rad(msg.angle_min)
            output_msg.angle_max = np.deg2rad(msg.angle_max)
            output_msg.angle_increment = np.deg2rad(msg.angle_increment)
        return output_msg

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # store laserscan_topic to self.scan
        self.scan = self.laserscan_angle_unit_conv(msg)
        self.lidar_properties_set = True
        self.num_scans = (self.scan.angle_max - self.scan.angle_min) // self.scan.angle_increment

        dist_in_front = self.get_range(self.scan, self.front_deg)
        theta_deg = 30 # deg
        theta_rad = np.deg2rad(theta_deg)

        dist_in_left = self.get_range(self.scan, self.left_deg)
        dist_in_right = self.get_range(self.scan, self.right_deg)

        dist_offset_left = self.get_range(self.scan, self.left_deg - theta_deg)
        dist_offset_right = self.get_range(self.scan, self.right_deg + theta_deg)

        # Calculate future and present distance from right wall
        alpha_r = np.arctan( (dist_offset_right * np.cos(theta_rad) - dist_in_right) / (dist_offset_right * np.sin(theta_rad)) )
        cur_pos_r = dist_in_right * np.cos(alpha_r) # Present Position
        fut_pos_r = cur_pos_r + self.car_length * np.sin(alpha_r) # projection in Future Position

        # Calculate future and present distance from left wall
        alpha_l = np.arctan( (dist_offset_left * np.cos(theta_rad) - dist_in_left) / (dist_offset_left * np.sin(theta_rad)) )
        cur_pos_l = dist_in_left * np.cos(alpha_l)
        fut_pos_l = cur_pos_l + self.car_length * np.sin(alpha_l)

        # Calculate error
        self.prev_error = self.error
        self.error = - (fut_pos_r - fut_pos_l)

    def periodic(self):
        self.current_time = self.get_clock().now().to_msg()
        self.pid_control()

def main(args=None):
    logger = rclpy.logging.get_logger('logger')

    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follower_node = WallFollow()
    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()