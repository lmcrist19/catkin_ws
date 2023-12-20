#!/usr/bin/env python3

# Import base ROS
import rospy

# Import os
import os

# Import ROS message information
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

##############################
# TimeJudge class definition
##############################
class TimeJudgeNode():
    
    def __init__(self):
        """Time judge node"""

        # Define zone infomation
        self.zones = [ [ [0.0, 3.0], [1.5, 1.5] ],
                       [ [1.5, 3.0], [3.0, 1.5] ],
                       [ [0.0, 1.5], [1.5, 0.0] ],
                       [ [1.5, 1.5], [3.0, 0.0] ],
                       [ [3.0, 3.0], [6.0, 0.0] ],
                       [ [6.0, 3.0], [7.5, 1.5] ],
                       [ [7.5, 3.0], [9.0, 1.5] ],
                       [ [6.0, 1.5], [7.5, 0.0] ],
                       [ [7.5, 1.5], [9.0, 0.0] ] ]
                       # VALIDATE ZONES!!

        self.same_zone = 0
        self.var = 0

        # Define timer information
        self.start_time = 0
        self.stop_time = 0
        self.delta_time = 0
        
        # Define subscriber to 'odom' of type Odometry -> odom_callback
        self.sub_odom = rospy.Subscriber('odom', Odometry,
                                         self.odom_callback, queue_size=1)

        # Define publisher to 'zone' of type Int16
        self.pub_zone = rospy.Publisher('zone',
                                         Int16, queue_size=1)

        # Enter ROS loop
        rospy.spin()
        
        return

    #################
    # Odom Callback
    #################
    def odom_callback(self, msg):
        '''Function to
        1) Get the robot location
        2) Monitor the current zone
        3) Publish zone transition (publish zone on correctly zone entry)
        4) Manage and display zone/timer information
        '''
        # Find the current zone
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        zone = self.find_zone(x,y)
        proper_zones = [2, 3, 4, 5, 6, 8, 7, 4, 1, 0, 2, 3, 4, 7, 8]

        if zone == proper_zones[self.var]:
            rospy.loginfo(f"Zone: {zone}")
            self.pub_zone.publish(zone)
            if self.var == 1:
                self.start_time = rospy.get_time()
                rospy.loginfo("==>> START TIMER <<==")
            if self.var == 14:
                self.stop_time = rospy.get_time()
                rospy.loginfo("==>> FINISH <<==")
                self.delta_time = self.stop_time - self.start_time
                rospy.loginfo('Course complete in %.2f secs' % self.delta_time)
                self.var = 0
            self.var += 1

        return
        
    #####################
    # find_zone function
    #####################
    def find_zone(self, x, y):
        """Function to determine which zone the robot is in"""
        zone_found = False
        zone_num = 0

        # Loop through zones and return the current zone
        while not zone_found and zone_num < 9:
            for zone_areas in self.zones:
                x_min = zone_areas[0][0]
                y_max = zone_areas[0][1]
                x_max = zone_areas[1][0]
                y_min = zone_areas[1][1]
                if x > x_min and x < x_max and y > y_min and y < y_max and zone_num != self.same_zone:
                    zone_found = True
                    self.same_zone = zone_num
                    break
                zone_num += 1
        if x > 9 or x < 0 or y > 3 or y < 0:
           # Log error if the zone location is not found
           rospy.logerr('Zone not found')
        
        return self.same_zone
        
    
#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('time_judge_node')
    print("Timer judge node initialized")
    
    # Start node
    try:
        TimeJudgeNode()
    except rospy.ROSInterruptException:
        pass
