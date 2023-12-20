#!/usr/bin/env python3

# Import base ROS
import rospy

# Import os
import os

# Import ROS message information
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

############################
# JoyReporter class definition
############################
class JoyReporterNode():
    def __init__(self):
        """Joystick reporter node"""

        # Setup joystick (defaults)
        self.speed_axis_id = 0
        self.steer_axis_id = 0
        self.speed_inc_button_id = 0
        self.speed_dec_button_id = 0
        self.steer_inc_button_id = 0
        self.steer_dec_button_id = 0
        self.speed_delta = 0.25
        self.steer_delta = 0.25
        self.speed_max = 1.0
        self.steer_max = 1.0
        
        self.speed_inc = False
        self.speed_dec = False
        self.steer_inc = False
        self.steer_dec = False

        # Get parameters
        if rospy.has_param('/joy_reporter_node/speed_axis'):
            self.speed_axis_id = \
                rospy.get_param('/joy_reporter_node/speed_axis')
        if rospy.has_param('/joy_reporter_node/steer_axis'):
            self.steer_axis_id = \
                rospy.get_param('/joy_reporter_node/steer_axis')
        if rospy.has_param('/joy_reporter_node/spd_up_btn'):
            self.speed_inc_button_id = \
                rospy.get_param('/joy_reporter_node/spd_up_btn')
        if rospy.has_param('/joy_reporter_node/spd_dwn_btn'):
            self.speed_dec_button_id = \
                rospy.get_param('/joy_reporter_node/spd_dwn_btn')
        if rospy.has_param('/joy_reporter_node/yaw_up_btn'):
            self.steer_inc_button_id = \
                rospy.get_param('/joy_reporter_node/yaw_up_btn')
        if rospy.has_param('/joy_reporter_node/yaw_dwn_btn'):
            self.steer_dec_button_id = \
                rospy.get_param('/joy_reporter_node/yaw_dwn_btn')        

        # Define joystick subscriber
        self.sub_joy = rospy.Subscriber('joy', Joy,
                                         self.joy_callback, queue_size=1)

        # Define twist publisher
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)

        # Enter ROS loop
        rospy.spin()
        
        return


    #####################
    # Joystick Callback
    #####################
    def joy_callback(self, msg):
        # Get the joystick data
        axes = msg.axes
        buttons = msg.buttons
        print_debug_line = False
        twist_val = Twist()
        
        # Check buttons for change in speed and steering gains
        # Adjust speed and steer scale
        if buttons[self.speed_inc_button_id] == 1 and not self.speed_inc:
            self.speed_inc = True
            self.speed_max += self.speed_delta
            print_debug_line = True
        elif buttons[self.speed_inc_button_id] == 0 and self.speed_inc:
            self.speed_inc = False
            
        if buttons[self.speed_dec_button_id] == 1 and not self.speed_dec:
            self.speed_dec = True
            print_debug_line = True
            if self.speed_max > 0:
                self.speed_max -= self.speed_delta
        elif buttons[self.speed_dec_button_id] == 0 and self.speed_dec:
            self.speed_dec = False
            
        if buttons[self.steer_inc_button_id] == 1 and not self.steer_inc:
            self.steer_inc = True
            self.steer_max += self.steer_delta
            print_debug_line = True
        elif buttons[self.steer_inc_button_id] == 0 and self.steer_inc:
            self.steer_inc = False
            
        if buttons[self.steer_dec_button_id] == 1 and not self.steer_dec:
            self.steer_dec = True
            print_debug_line = True
            if self.steer_max > 0:
                self.steer_max -= self.steer_delta
        elif buttons[self.steer_dec_button_id] == 0 and self.steer_dec:
            self.steer_dec = False
        
        if print_debug_line:
            rospy.loginfo(f"Max Speed: {self.speed_max} Max Steer: {self.steer_max}")
            
        # Compute speed and steer based on stick inputs
        if not self.speed_inc and not self.speed_dec and not self.steer_inc and not self.steer_dec:
            twist_val.linear.x = axes[self.speed_axis_id]*self.speed_max
            twist_val.angular.z = axes[self.steer_axis_id]*self.steer_max
        
        # Publish cmd_vel message
        self.pub_twist.publish(twist_val)
        
        return
    
#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('joy_reporter_node')
    print("Joy Reporter node initialized")
    
    # Start node
    try:
        JoyReporterNode()
    except rospy.ROSInterruptException:
        pass
