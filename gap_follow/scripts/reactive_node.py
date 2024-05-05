import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time
import os

import numpy as np
# from enum import Enum


from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

def angle_mod(angle_deg, type):
    output_angle = angle_deg
    if type == 0:
        if angle_deg > 360:
            output_angle = angle_deg - 360
        elif angle_deg < 0:
            output_angle = angle_deg + 360        
    elif type == 1:
        if angle_deg > 180:
            output_angle = angle_deg - 360
        elif angle_deg < -180:
            output_angle = angle_deg + 360
    else:
        return None
    return output_angle

class ReactiveFollowGap(Node):

    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scanner/scan'
        drive_topic = '/drive'

        self.frame_id = self.get_parameter('frame_id').value

        self.max_vel = self.get_parameter('max_vel').value 	# used for pid_vel (not much use).
        self.look_ahead_length = self.max_vel           	# projection distance we project car forward.

        laserscan_topic =  self.get_parameter('laserscan_topic').value
        drive_topic =  self.get_parameter('drive_topic').value

        self.laser_subscriber = self.create_subscription(LaserScan,
                                                         laserscan_topic, 
                                                         self.scan_callback, 
                                                         QoSProfile(depth=5, 
                                                                    reliability=ReliabilityPolicy.BEST_EFFORT)
                                                         )
        self.laser_subscriber

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 
                                                     drive_topic, 
                                                     10)

        self.current_time = self.get_clock().now().to_msg()


    def get_range(self, scan_data, angle_deg):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            scan_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        angle_deg = angle_mod(angle_deg, self.angle_type)
        angle_rad = np.deg2rad(angle_deg)
        index = int((angle_rad - scan_data.angle_min) // scan_data.angle_increment)        
        return scan_data.ranges[index]


    def extend_disparities(self, msg):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.scan=msg
        lidarderivative=np.diff(self.scan)
        halfcarwidth=0.2 #Meters
        i = 0
        for i in range (0,len(lidarderivative)):
            if lidarderivative(i)>1: #meters, sort of a magic number for now
                #calculate how wide an angle of points we want to exclude based on distance
                anglewidth=halfcarwidth/self.scan[i]
                gap=anglewidth/360
                self.scan[range(i-gap),(i+gap)] = 0
            #basically returning an array with the "no go," angles
        
        targetAngle=list.index(max(self.scan))

        return targetAngle


    def pid_control(self):
            '''
            Based on the calculated error, publish vehicle control

            Args:
                desired_vel: Desired Velocity

            Returns:
                None
        `
            angle = 0.0
            '''
        # TODO: Use kp, ki & kd to implement a PID controller
        
        if self.ki != 0.0:
            self.integral_error += self.error * self.Ts

        p_term = self.kp * self.error
        i_term = self.ki * self.integral_error
        d_term = self.kd * (self.error - self.prev_error) / self.Ts
        desired_steering = p_term + i_term + d_term

        if np.abs(desired_steering) < np.deg2rad(10):
            desired_vel = self.max_vel
        elif np.abs(desired_steering) < np.deg2rad(20):
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

        # Print ROS parameters
        self.get_logger().info(
            f'\n desired_steering_angle: {desired_steering}'
            f'\n desired_speed: {desired_vel}'
        )



def main():
    logger = rclpy.logging.get_logger('logger')

    rclpy.init()
    print("Obstacle Avoidance Initialized")
    ReactiveFollowGap = ReactiveFollowGap()
    try:
        rclpy.spin(ReactiveFollowGap)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ReactiveFollowGap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()