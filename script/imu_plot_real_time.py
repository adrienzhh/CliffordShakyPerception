#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class IMU_Plot:
    def __init__(self):
        self.time_data = []
        self.acceleration_data = []
        self.plot_imu()

    def plot_imu(self):
        # Additional argument for imu_callback
        extra_arg = None  # Replace with the appropriate value if needed

        # Initialize ROS node
        # rospy.init_node('imu_plot_node')

        # Subscribe to IMU topic
        rospy.Subscriber('/imu', Imu, lambda msg: self.imu_callback(msg, extra_arg), queue_size=1)

        # Initialize plot
        plt.figure()

        # Start animation
        ani = FuncAnimation(plt.gcf(), self.update_plot, interval=1000)
        plt.show()

        # Start ROS spin
        # rospy.spin()

    # Callback function for processing IMU messages
    def imu_callback(self, msg, extra_arg):
        # Extract linear acceleration in z-axis
        linear_acceleration_z = msg.linear_acceleration.z
        # Store timestamp and linear acceleration
        self.time_data.append(rospy.Time.now().to_sec())
        self.acceleration_data.append(linear_acceleration_z)

    def update_plot(self, frame):
        plt.cla()
        plt.plot(self.time_data, self.acceleration_data)
        plt.axhline(9.8, color='red', linestyle='--', label='Gravity')
        plt.xlabel('Time')
        plt.ylabel('Linear Acceleration (z-axis)')
        plt.title('IMU Data')

if __name__ == '__main__':
    try:
        IMU_Plot()
    except rospy.ROSInterruptException:
        pass