#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class IMU_Plot:
    def __init__(self):
        self.time_data = []
        self.acceleration_data = []
        self.angular_data = []
        self.acceleration_z = []
        self.plot_imu()

    def plot_imu(self):
        # Additional argument for imu_callback
        extra_arg = None  # Replace with the appropriate value if needed

        # Initialize ROS node
        rospy.init_node('imu_plot_node')

        # Subscribe to IMU topic
        rospy.Subscriber('/imu', Imu, lambda msg: self.imu_callback(msg, extra_arg))

        # Initialize plot
        plt.figure()

        # Start animation
        ani = FuncAnimation(plt.gcf(), self.update_plot, interval=1000)
        plt.show()

        # Start ROS spin
        # rospy.spin()

    # Callback function for processing IMU messages
    def imu_callback(self, msg, extra_arg):
        # Extract linear acceleration in z-axis\
        linear_acceleration_z = msg.linear_acceleration.z

        angular_x = msg.angular_velocity.x
        angular_y = msg.angular_velocity.y
        angular_z = msg.angular_velocity.z
        y = msg.linear_acceleration.y
        x = msg.linear_acceleration.x
        accerleration_mag = math.sqrt(linear_acceleration_z**2 + y**2 + x**2)
        angular_mag = math.sqrt(angular_x**2 + angular_y**2 + angular_z**2)
        # Store timestamp and linear acceleration
        self.time_data.append(rospy.Time.now().to_sec())
        self.acceleration_data.append(accerleration_mag)
        self.angular_data.append(angular_z)
        self.acceleration_z.append(linear_acceleration_z)

    def update_plot(self, frame):
        plt.cla()
        plt.plot(self.time_data, self.angular_data)
        plt.axhline(9.8, color='red', linestyle='--', label='Gravity')
        plt.xlabel('Time')
        plt.ylabel('Acceleration  Z')
        plt.title('IMU Data')

if __name__ == '__main__':
    try:
        IMU_Plot()
    except rospy.ROSInterruptException:
        pass
