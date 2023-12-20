#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np
import std_msgs.msg

# Import ROS message information
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from hw09_crist.cfg import RobotCenteringDynCfgConfig


# /circ0_loc
# /circ1_loc

ACTIVE_WINDOWS = []

######################################
# RobotCenteringNode class definition
######################################
class RobotCenteringNode():
    def __init__(self):
        """Line following node"""

        # Initialize dynamic reconfigure
        self.enable = 0
        self.record_max = 0
        self.window = 0
        self.gain = 0
        self.position_err = 0
        self.max_err = 0
        self.old_max = 0
        

        # Define the subscribers
        self.sub_lidar = rospy.Subscriber('scan', LaserScan,
        				                  self.lidar_callback)
        
        # Define publishers
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)
        self.pub_lidar = rospy.Publisher('scan',
 	                                     LaserScan, queue_size=1)

        # Set up dynamic reconfigure
        self.srv = Server(RobotCenteringDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            if self.enable:
                twist_msg.linear.x = self.gain * self.position_err
                self.pub_twist.publish(twist_msg)
                if self.record_max:
                    if abs(self.position_err) > abs(self.max_err):
                        self.max_err = np.round(self.position_err, 2)
                        if self.max_err != self.old_max:
                            rospy.loginfo(f'Max Error: {self.max_err}')
                            self.old_max = self.max_err
                else:
                    self.max_err = 0
            else:
                twist_msg = Twist()
                self.pub_twist.publish(twist_msg)
            
            # Sleep for time step
            self.rate.sleep()
            
        return

    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.enable = config['enable']
        self.record_max = config['record_max']
        self.window = config['window']
        self.gain = config['gain']
        
        self.dyn_config = config
        
        return config

    ##################
    # Lidar callback
    ##################
    def lidar_callback(self, scan_msg):
        max_angle = 360
        min_angle = 0
        
        front_angle = min_angle
        rear_angle = max_angle//2
        
        front_window_min = max_angle - self.window
        front_window_max = front_angle + self.window
        rear_window_min = rear_angle - self.window
        rear_window_max = rear_angle + self.window
        
        front_range = list(scan_msg.ranges[front_window_min:max_angle])
        for x in scan_msg.ranges[front_angle:front_window_max+1]:
            front_range.append(x)
        np.array(front_range)
        front_min_dist = np.amin(front_range)
        
        rear_range = list(scan_msg.ranges[rear_window_min:rear_window_max + 1])
        np.array(rear_range)
        rear_min_dist = np.amin(rear_range)
        
        self.position_err = front_min_dist - rear_min_dist
        
        # right scan_msg.ranges[0]
        # left scan_msg.ranges[180]
        
        return

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('robot_centering_node')
    print("Robot Centering Node initialized")
    
    # Start node
    try:
        RobotCenteringNode()
    except rospy.ROSInterruptException:
        pass
