#!/usr/bin/env python3

# Import base ROS
import rospy

# Import NumPy
import math

# Import ROS message information
from std_msgs.msg import Float32

########################################
# MoveObjectsNode class definition
########################################
class MoveObjectsNode():
    def __init__(self):
        """Move obejcts node"""

        # Initialize object locations
        x0, y0 = 3.0, 1.5
        x1, y1 = 6.0, 1.5
        rad = 0.2
        
        # Initialize dynamic reconfigure
        self.rect0_amp = 1.0
        self.rect0_freq = 0.1
        self.rect1_amp = 1.0
        self.rect1_freq = 0.05

        # Define publisher
        self.pub_circ0 = rospy.Publisher('circ0_loc', Float32, queue_size=1)
        self.pub_circ1 = rospy.Publisher('circ1_loc', Float32, queue_size=1)

        # Initialize the time vector
        time0 = rospy.get_time()

        self.rate = rospy.Rate(20)
        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Update obstruction locations
            t = rospy.get_time() - time0
            dx0 = self.rect0_amp * math.sin(2*math.pi*self.rect0_freq*t)
            dx1 = self.rect1_amp * math.sin(2*math.pi*self.rect1_freq*t)

            # Update the parameters
            rospy.set_param('circ0', [x0+dx0, y0, rad] )
            rospy.set_param('circ1', [x1+dx1, y1, rad] )

            # Publish locations
            msg = Float32() 
            msg.data = x0+dx0
            self.pub_circ0.publish(msg)
            msg.data = x1+dx1
            self.pub_circ1.publish(msg)
           
            # Sleep for time step
            self.rate.sleep()
            
        return

    
#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('move_objects_node')
    print("Move Objects node initialized")
    
    # Start node
    try:
        MoveObjectsNode()
    except rospy.ROSInterruptException:
        pass
