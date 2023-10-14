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
        self.not_move = True
        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/hans_bot_wheel_controller/cmd_vel/', Twist, queue_size=10)
        self.slider_pos_pub = rospy.Publisher('/hans_bot_slider_controller/command/', Float64, queue_size=10)
        self.head_pos_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.head_ang_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.des_pos_pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
        self.global_pose_sub = rospy.Subscriber('gazebo/link_states', LinkStates, callback=self.pose_callback)
        self.mover()

    def pose_callback(self, msg):
        cam_link_index = msg.name.index("robot::cam")
        head_angle = msg.pose[cam_link_index]
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
        self.vel_msg.linear.x = 1  # Linear velocity for forward movement
        self.vel_msg.angular.z = 0.0  # Angular velocity (zero for straight motion)
        self.head_pos_msg.data = 0.0
        self.head_ang_msg.data = 0.0
        rate = rospy.Rate(10)  # 10hz
        time_0 = rospy.get_time()
        
        # Target position and angle for the square loop
        target_x = 7.0 # m for distance travel 
        target_angle = np.radians(90)  # 90 degrees counterclockwise
        loop = 0
        # omega = rospy.get_param('~omega')

        if len(sys.argv) > 1:
            try:
                omega = float(sys.argv[1]) # (0.125) This value will change the speed of the oscillations
                print("Omega:", omega)
            except ValueError:
                print("Invalid input. The first argument should be a number.")
        else:
            omega = 0.125 # 0.125 hz
            print("No command-line argument provided.")

        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # Check if the robot has reached the target x position
            if self.not_move:
                self.vel_msg.linear.x = 1  # Linear velocity for forward movement
                self.vel_msg.angular.z = 0.0  # Angular velocity (zero for straight motion)
                if self.pos_x > target_x:
                    self.vel_msg.linear.x = 0
                    rospy.signal_shutdown("Robot has reached the desired distance")
            # Rotating head

            self.head_pos_msg.data = np.sin(omega*2*np.pi*time)
            self.head_ang_msg.data = 0.436332*np.sin(omega*2*np.pi*time)

#            self.head_pos_msg.data = 0
           # self.head_ang_msg.data = -(45*np.pi)/180

            # Publish the velocity and head position commands
            self.robot_vel_pub.publish(self.vel_msg)
            self.head_pos_pub.publish(self.head_pos_msg)
            self.slider_pos_pub.publish(self.head_pos_msg.data)
            self.head_ang_pub.publish(self.head_ang_msg)
            rate.sleep()
        
        

if __name__ == '__main__':
    try:
        Mover()
    except rospy.ROSInterruptException:
        pass
