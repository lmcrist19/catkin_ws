#!/usr/bin/env python3

# Import base ROS
import rospy

# Import ROS message information
from std_msgs.msg import Empty
from std_msgs.msg import Float32

# Random number
import random


##############################
# NumberSum class definition
##############################
class NumberSum():
    def __init__(self):
        """Number sum node"""

        # Initialize sum
        self.sum = 0.0

        # Define subscribers
        ##
        ## Add subscriber to 'rand_num' topic with sum_number_callback
        ##
        self.sub_float = rospy.Subscriber('rand_num', Float32,
                                          self.sum_number_callback,
                                          queue_size=1)

        self.sub_reset = rospy.Subscriber('reset_sum', Empty,
                                          self.reset_callback,
                                          queue_size=1)

        # Define publishers
        ##
        ## Add publisher to 'sum' topic
        ## self.pub_sum =
        ##
        self.pub_sum = rospy.Publisher('sum',
                                       Float32, queue_size=1)
                                         
        # Enter ROS loop
        rospy.spin()

        return
    
        
    #######################
    # Sum number Callback
    #######################
    def sum_number_callback(self, msg):
        ##
        ## Add sum update and rospy.log sum print out
        ##
        self.sum += msg.data
        rospy.loginfo('Sum = %.2f' % self.sum)


        ##
        ## Publish sum with self.pub_sum
        ## msg_out = Float32()
        ##
        msg_out = Float32()
        msg_out.data = self.sum
        self.pub_sum.publish(msg_out)
        
        
        return


    ##################
    # Reset Callback
    ##################
    def reset_callback(self, msg):
        ## Reset sum
        self.sum = 0.0
        rospy.loginfo('****> Reset Sum <****')
        
        return


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('number_sum_node')
    print("Number sum node initialized")
    
    # Start node
    try:
        NumberSum()
    except rospy.ROSInterruptException:
        pass
