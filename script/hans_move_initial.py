#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.msg import ModelStates, LinkStates
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import sys
import tf
import imu_plot_real_time 
import time

class Mover:
    def __init__(self):
        self.vel_msg = Twist()
        self.head_pos_msg = Float64()
        self.head_ang_msg = Float64()
        self.curr_pose = PoseStamped()
        self.pos_z = 0.0
        self.pos_y = 0.0
        self.pos_x = 0.0
        self.angle = 0.0
        self.is_move = True
        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/hans_bot_wheel_controller/cmd_vel/', Twist, queue_size=10)
        self.slider_pos_pub = rospy.Publisher('/hans_bot_slider_controller/command/', Float64, queue_size=10)
        self.head_pos_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.head_ang_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.des_pos_pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
        self.global_pose_sub = rospy.Subscriber('gazebo/link_states', LinkStates, callback=self.pose_callback)
        self.mover()

    def pose_callback(self, msg):
        head_angle = msg.pose[37]
        quaternion = (
            head_angle.orientation.x,
            head_angle.orientation.y,
            head_angle.orientation.z,
            head_angle.orientation.w
        )
        self.pos_x = head_angle.position.x
        self.pos_y = head_angle.position.y
        self.pos_z = head_angle.position.z
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.angle = euler[2]

    def mover(self):
        self.vel_msg.linear.x = 0  # Linear velocity for forward movement
        self.vel_msg.angular.z = 0.0  # Angular velocity (zero for straight motion)
        self.head_pos_msg.data = 0.0
        self.head_ang_msg.data = 0.0
        rate = rospy.Rate(10)  # 10hz
        time_0 = rospy.get_time()
        
        # Target position and angle for the square loop
        target_x = 7.0 # m for distance travel 
        target_angle = np.radians(90)  # 90 degrees counterclockwise
        loop = 0
        omega = 0

        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # Rotating head

            self.head_pos_msg.data = 0.15*np.sin(omega*2*np.pi*time)
            self.head_ang_msg.data = 0.436332*np.sin(omega*2*np.pi*time)

            # Publish the velocity and head position commands
            self.robot_vel_pub.publish(self.vel_msg)
            self.head_pos_pub.publish(self.head_pos_msg)
            self.slider_pos_pub.publish(self.head_pos_msg.data)
            self.head_ang_pub.publish(self.head_ang_msg)
            # time.wait(1)
            if time > 0.5:
                rospy.signal_shutdown("Robot initialized")
            rate.sleep()
            
        
        
        
        

if __name__ == '__main__':
    try:
        Mover()
        
    except rospy.ROSInterruptException:
        pass
