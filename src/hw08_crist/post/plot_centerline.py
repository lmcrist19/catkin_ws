import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt

########################
# Load Center Line Data
########################
centerline = np.genfromtxt('hw08_center.csv', delimiter=',')

##############
# Load rosbag
##############
title_str = 'Proportional (v = 1.0 m/s, k = 3.3)'
bag = rosbag.Bag('p_control_1.bag')


# Initialize storage
x_pos = []
y_pos = []

# Parse rosbag
for topic, msg, t in bag.read_messages(topics=['/odom']):
    if topic == '/odom':
        x_pos.append(msg.pose.pose.position.x)
        y_pos.append(msg.pose.pose.position.y)
        
bag.close()

# Convert arrays to NumPy
x_pos = np.array(x_pos)
y_pos = np.array(y_pos)


###################
# Plot performance
###################
plt.figure()
plt.plot(centerline[:,0], centerline[:,1], '.', markersize=1, label='Center Line')
plt.plot(x_pos, y_pos, '.', markersize=1, label='Robot Path')
plt.title(title_str)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()
