import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Define the rosbag
bag = rosbag.Bag('2023-03-30-17-35-21.bag')

# Initialize storage
time_pos = []
position = []

time_vel = []
velocity = []

# Parse rosbag
for topic, msg, t in bag.read_messages(topics=['odom',
                                               'cmd_vel']):
    if topic == 'odom':  
        time_pos.append(t.to_sec())      
        position.append(msg.pose.pose.position.x)

    if topic == 'cmd_vel':
        time_vel.append(t.to_sec())
        velocity.append(msg.linear.x)

bag.close()

# Convert arrays to NumPy
position = np.array(position)
time_pos = np.array(time_pos)
time_vel = np.array(time_vel)
velocity = np.array(velocity)

# Find minimum time
t_min = min(time_pos[0], time_vel[0])
time_pos -= t_min
time_vel -= t_min
plt.figure()
plt.plot(time_pos, position, label='Position')
plt.plot(time_vel, velocity, label='Velcoity')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m) / Velocity (m/s)')
plt.title('Position and Velocity vs. Time')
plt.legend()
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()

# Cross plot
v_vel_interp = np.interp(time_pos, time_vel, velocity)
plt.figure()
plt.plot(position, v_vel_interp)
plt.title('Speed vs. Position')
plt.xlabel('Position (m)')
plt.ylabel('Speed (m/s)')
plt.show()
