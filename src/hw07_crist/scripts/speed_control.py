#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np

# Import ROS message information
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from hw07_crist.cfg import SpeedSettingsDynCfgConfig


####################################
# SpeedControlNode class definition
####################################
class SpeedControlNode():
    def __init__(self):
        """Speed control node"""

        # Initialize dynamic reconfigure
        self.enable = 0
        self.red_offset = 0.0
        self.red_gain = 0.0
        self.blue_offset = 0.0
        self.blue_gain = 0.0
        self.green_offset = 0.0
        self.green_gain = 0.0

        # Initialize speed
        self.speed = 0.0
      
        # Define the image subscriber
        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.camera_callback, queue_size=1)
        
        # Define 'cmd_vel' publisher
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)

        # Set up dynamic reconfigure
        self.srv = Server(SpeedSettingsDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message speed and yaw rate message and publish
            if self.enable:
                msg.linear.x = self.speed
                self.pub_twist.publish(msg)
            else:
                msg = Twist()
                self.pub_twist.publish(msg)

            # Sleep for time step
            self.rate.sleep()
            
        return


    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.enable = config['enable']
        self.red_offset = config['red_offset']
        self.red_gain = config['red_gain']
        self.blue_offset = config['blue_offset']
        self.blue_gain = config['blue_gain']
        self.green_offset = config['green_offset']
        self.green_gain = config['green_gain']
        return config
        

    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        
        # Get the Blue, Green and Red average values
        img_blue = img[:,:,0]   # Blue = 1st channel
        img_green = img[:,:,1]  # Green = 2nd channel
        img_red = img[:,:,2]    # Red = 3rd channel

        avg_blue = np.average(img_blue)
        avg_green = np.average(img_green)
        avg_red = np.average(img_red)

        # Compute the color base speed
        v_blue = self.blue_gain * avg_blue/255.0 + self.blue_offset
        v_green = self.green_gain * avg_green/255.0 + self.green_offset
        v_red = self.red_gain * avg_red/255.0 + self.red_offset

        self.speed = v_blue * v_green * v_red
        
        if self.speed < 0.05:
            self.speed = 0.0

        rospy.loginfo('R:%.2f  G:%.2f  B:%.2f  V:%.2f' %(v_red, v_green, v_blue, self.speed))

        return


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('speed_control_node')
    print("Speed Control node initialized")
    
    # Start node
    try:
        SpeedControlNode()
    except rospy.ROSInterruptException:
        pass
