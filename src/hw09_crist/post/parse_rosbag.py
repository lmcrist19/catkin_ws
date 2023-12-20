import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Define the rosbag
bag = rosbag.Bag('k_3_e_0_19.bag')

# Initialize storage
robot_t = []
robot_x = []

c0_t = []
c0_x = []

c1_t = []
c1_x = []

# Parse rosbag
for topic, msg, t in bag.read_messages(topics=['/odom',
                                               '/circ0_loc',
                                               '/circ1_loc']):
    if topic == '/odom':  
        robot_t.append(t.to_sec())      
        robot_x.append(msg.pose.pose.position.x)

    if topic == '/circ0_loc':
        c0_t.append(t.to_sec())
        c0_x.append(msg.data)
        
    if topic == '/circ1_loc':
        c1_t.append(t.to_sec())
        c1_x.append(msg.data)

bag.close()

# Convert arrays to NumPy
robot_t = np.array(robot_t)
robot_x = np.array(robot_x)

c0_t = np.array(c0_t)
c0_x = np.array(c0_x)

c1_t = np.array(c1_t)
c1_x = np.array(c1_x)

t_min = min(robot_t[0], c0_t[0], c1_t[0])
robot_t -= t_min
c0_t -= t_min
c1_t -= t_min

# Interpolate the data
c0_interp = np.interp(robot_t, c0_t, c0_x)
c1_interp = np.interp(robot_t, c1_t, c1_x)

# Calculate error
front_dist = c1_interp - robot_x
rear_dist = robot_x - c0_interp
position_err = front_dist - rear_dist

# Plot the robot path
plt.subplot(2,1,1)
plt.plot(robot_t, robot_x, label='Robot')
plt.plot(robot_t, c0_interp, label='Circle 0')
plt.plot(robot_t, c1_interp, label='Circle 1')
plt.ylabel('X (m)')
plt.title('K = 3.00, Max Error = 0.19')
plt.legend()

plt.subplot(2,1,2)
plt.plot(robot_t, position_err)
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')

plt.show()
