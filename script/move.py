#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from gazebo_msgs.msg import ModelStates

class Shakey_Perception:

    def __init__(self):
        self.vel_msg = Twist()
        self.head_pos_msg = Float64()
        self.head_ang_msg = Float64()
        self.curr_pose = PoseStamped()

        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/atom/cmd_vel/', Twist, queue_size=10)

        self.mover()

        

    def pose_callback(self,msg):
        position = msg.pose[1].position.y
        self.curr_pose = position

    def mover(self):

        self.vel_msg.linear.x = 10.0
        rate = rospy.Rate(10) # 10hz
        time_0 = rospy.get_time()
        omega = 0.125

        print("Periodic SLAM move controller initialized")

        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # print(self.head_pos_msg.data)
            # print(self.head_ang_msg.data)
            self.robot_vel_pub.publish(self.vel_msg)

            #print(self.curr_pose)
            #print(rospy.get_time())
            rate.sleep()


Shakey_Perception()
# if __name__ == '__main__':
#     try:
#         Shakey_Perception()
#     except rospy.ROSInterruptException:
#         pass
