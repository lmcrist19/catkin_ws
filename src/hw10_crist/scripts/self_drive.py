#!/usr/bin/env python3

# Import base ROS
import rospy

# Import NumPy
import numpy as np
import std_msgs.msg

# Import ROS message information
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from hw10_crist.cfg import SelfDriveDynCfgConfig

"""Loop until vehicle is at the ending location
while( vehicle_location != ending_location )
• Set the vehicle speed
• Determine the heading angle between the vehicle location and the
ending location
• “path heading”
• Compute the error between path heading and the vehicle heading
• “heading error”
• Request a vehicle yaw rate proportional to the heading error

Vehicle Yaw Request = K * (Vehicle Bearing Angle – Path Bearing Angle)

Bearing angle

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def odom_callback(msg):
    orientation_quat = msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    bearing_angle_degrees = math.degrees(yaw)
    print("Bearing angle (degrees): ", bearing_angle_degrees)

rospy.init_node('odom_listener')
odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.spin()

"""


ACTIVE_WINDOWS = []

##################################
# SelfDriveNode class definition
##################################
class SelfDriveNode():
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
        self.sub_odom = rospy.Subscriber('odom', Odometry,
        				                  self.odom_callback)
        
        # Define publishers
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)

        # Set up dynamic reconfigure
        self.srv = Server(SelfDriveDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            """if self.enable:
                twist_msg.linear.x = self.gain * self.position_err
                self.pub_twist.publish(twist_msg)
            else:
                twist_msg = Twist()
                self.pub_twist.publish(twist_msg)"""
            
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

    ####################
    # Odometry callback
    ####################
    def odom_callback(self, msg):
        
        return

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('self_drive_node')
    print("Self Drive Node initialized")
    
    # Start node
    try:
        SelfDriveNode()
    except rospy.ROSInterruptException:
        pass
