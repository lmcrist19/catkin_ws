#!/usr/bin/env python3

# Import base ROS
import rospy

# Import ROS message information
from std_msgs.msg import Empty
from std_msgs.msg import Float32

# Random number
import random


#####################################
# RandNumPublisher class definition
#####################################
class RandNumPublisher():
    def __init__(self):
        """Random number publisher node"""

        # Initialize
        self.reset = False
        self.reset_sum = 500.0

        # Define subscribers
        ##
        ## Add subscriber to 'sum' topic with callback check_sum_callback
        ##
        self.sub_sum = rospy.Subscriber('sum', Float32,
                                        self.check_sum_callback,
                                        queue_size=1)

        # Define publishers
        self.pub_float = rospy.Publisher('rand_num',
                                         Float32, queue_size=1)
        
        self.pub_reset = rospy.Publisher('reset_sum',
                                         Empty, queue_size=1)

        ##
        ## Add publisher to reset_sum topic (Empty)
        ## self.pub_reset = 

        # Define ROS rate
        self.rate = rospy.Rate(1)

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Generate random number and build message
            ##
            ## Add msg creation and publishing
            msg_out = Float32()
            msg_out.data = random.uniform(0.00, 100.00)
            self.pub_float.publish(msg_out.data)
            rospy.loginfo('-> Sending %.2f' % msg_out.data)
            
            
            # Sleep for time step
            self.rate.sleep()
            
        return

    ##########################
    # Set Check Sum Callback
    ##########################
    def check_sum_callback(self, msg):
        ##
        ## Add
        ## 1) rospy.loginfo of the sum returned by the sum node
        ## 2) Check sum against 500.0 and publish reset message
        ##
        ## Reset message
        ##    msg_out = Empty()
        ##    self.pub_reset.publish(msg_out)
        self.sum = msg.data
        rospy.loginfo('<- Sum %.2f' % self.sum)
        
        if( self.sum > 500 ):
	        msg_out = Empty()
	        self.pub_reset.publish(msg_out)
	        rospy.loginfo('****> Reset Sum <****')
        return


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('random_number_gen_node')
    print("Random number generator node initialized")
    
    # Start node
    try:
        RandNumPublisher()
    except rospy.ROSInterruptException:
        pass
