import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

#Parameters
start_delay = 0.5

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])
plan_pos_arm = np.array([])
plan_vel_arm = np.array([])
plan_acc_arm = np.array([])
plan_pos_forearm = np.array([])
plan_vel_forearm = np.array([])
plan_acc_forearm = np.array([])

torque_t = np.array([])
torque_arm = np.array([])
torque_forearm = np.array([])

jointstate_t = np.array([])
jointstate_pos_arm = np.array([])
jointstate_vel_arm = np.array([])
jointstate_pos_forearm = np.array([])
jointstate_vel_forearm = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_arm = np.append(plan_pos_arm, msg.positions[0])
        plan_pos_forearm = np.append(plan_pos_forearm, msg.positions[1])
        plan_vel_arm = np.append(plan_vel_arm, msg.velocities[0])
        plan_vel_forearm = np.append(plan_vel_forearm, msg.velocities[1])
        plan_acc_arm = np.append(plan_acc_arm, msg.accelerations[0])
        plan_acc_forearm = np.append(plan_acc_forearm, msg.accelerations[1])

    if topic == "/joint_torque":
        torque_t = np.append(torque_t, msg.data[0])
        torque_arm = np.append(torque_arm, msg.data[1])
        torque_forearm = np.append(torque_forearm, msg.data[2])

    if topic == "/joint_states":
        jointstate_t = np.append(jointstate_t, float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))
        jointstate_pos_arm = np.append(jointstate_pos_arm, msg.position[0])
        jointstate_pos_forearm = np.append(jointstate_pos_forearm, msg.position[1])
        jointstate_vel_arm = np.append(jointstate_vel_arm, msg.velocity[0])
        jointstate_vel_forearm = np.append(jointstate_vel_forearm, msg.velocity[1])

bag.close()

# Plot data
# Planned trajectory arm
plt.figure(1)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_arm,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_arm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_arm,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_arm,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Planned trajectory forearm
plt.figure(2)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_forearm,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_forearm,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_forearm,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_forearm,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Joint torques
plt.figure(3)

plt.subplot(2, 1, 1)
plt.plot(torque_t,torque_arm)
plt.ylabel('Torque arm [Nm]')
plt.title('Joint torques')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(torque_t,torque_forearm)
plt.ylabel('Torque forearm [Nm]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
