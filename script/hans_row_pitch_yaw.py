#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.msg import ModelStates, LinkStates
import numpy as np
import sys
import tf


class Mover:

    def __init__(self):
        self.vel_msg = Twist()
        self.head_pos_msg = Float64()
        self.head_ang_msg = Float64()
        self.curr_pose = PoseStamped()
        self.pos_z = 0.0
        self.angle = 0.0
        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/hans_bot_wheel_controller/cmd_vel/', Twist, queue_size=10)
        self.slider_pos_pub = rospy.Publisher('/hans_bot_slider_controller/command/', Float64, queue_size=10)
        self.head_pos_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.des_pos_pub = rospy.Publisher('/desired_pose',PoseStamped,queue_size=10)
        # self.robot_vel_pub = rospy.Publisher('/test_bot_2_wheel_controller/cmd_vel/', Twist, queue_size=10)
        # self.slider_pos_pub = rospy.Publisher('/test_bot_2_slider_controller/command/', Float64, queue_size=10)
        # self.head_pos_pub = rospy.Publisher('/test_bot_2_head_controller/command/', Float64, queue_size=10)

        self.global_pose_sub = rospy.Subscriber('gazebo/link_states', LinkStates, callback=self.pose_callback)
        self.mover()

    def pose_callback(self,msg):
        
        # print(len(msg.pose))
        head_angle = msg.pose[11]
        quaternion = (
        head_angle.orientation.x,
        head_angle.orientation.y,
        head_angle.orientation.z,
        head_angle.orientation.w)

        self.pos_x = head_angle.position.x
        self.pos_z = head_angle.position.z
        
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.angle = euler[1]
    
    def mover(self):

      
        self.vel_msg.linear.x = 0.5
        self.head_pos_msg.data = 0.0
        self.head_ang_msg.data =  0.0
        rate = rospy.Rate(10) # 10hz
        # print('cmd', sys.argv[1])
        time_0 = rospy.get_time()
        omega = 2 # float(sys.argv[1])# 0.125 # This value will change the speed of the oscillations


        length = 5.0
        width = 6.0
        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0

            self.head_pos_msg.data = 0.15*np.sin(omega*2*np.pi*time)
            self.head_ang_msg.data = 0.436332*np.sin(omega*2*np.pi*time)
            # print(self.head_ang_msg.data)
            self.robot_vel_pub.publish(self.vel_msg)
            self.slider_pos_pub.publish(self.head_pos_msg.data)
            self.head_pos_pub.publish(self.head_ang_msg.data)

            rate.sleep()


if __name__ == '__main__':
    try:
        print("start.")
        Mover()
    except rospy.ROSInterruptException:
        pass


