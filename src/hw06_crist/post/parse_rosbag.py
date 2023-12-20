import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Define the rosbag
bag = rosbag.Bag('subset.bag')

# Initialize storage
time_pose = []
x_position = []
y_position = []

time_zone = []
x_zone = []
y_zone = []

# Parse rosbag
for topic, msg, t in bag.read_messages(topics=['/odom',
                                               '/zone']):
    if topic == '/odom':  
        time_pose.append(round(t.to_sec(), 1))      
        x_position.append(msg.pose.pose.position.x)
        y_position.append(msg.pose.pose.position.y)

    if topic == '/zone':
        time_zone.append(round(t.to_sec(), 1))

bag.close()

for i in range(len(time_zone)):
    position_index = time_pose.index(time_zone[i])
    x_zone.append(x_position[position_index])
    y_zone.append(y_position[position_index])

# Convert arrays to NumPy
x_position = np.array(x_position)
y_position = np.array(y_position)

x_zone = np.array(x_zone)
y_zone = np.array(y_zone)

# Plot the robot path
plt.plot(x_position, y_position, label='Path')
plt.plot(x_zone, y_zone, 'o', label='Zone Entry')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()
