#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from gazebo_msgs.msg import ModelStates
import rosnode

class Clifford_velodyne:

    def __init__(self):
        self.vel_msg = Twist()
        self.head_pos_msg = Float64()
        self.head_ang_msg = Float64()
        self.curr_pose = PoseStamped()
        
        self.pos_x = 0.0 # postion checker

        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/cliffordDrive', Twist, queue_size=10)

        self.mover()

        

    def pose_callback(self,msg):
        position = msg.pose[1].position.y
        self.curr_pose = position

    def mover(self):

        self.vel_msg.linear.x = 0.5 # 0.5
        rate = rospy.Rate(10) # 10hz
        time_0 = rospy.get_time()

        print("Periodic SLAM for Clifford Velodyne hdl32 move controller initialized")

        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # print(self.head_pos_msg.data)
            # print(self.head_ang_msg.data)
            self.robot_vel_pub.publish(self.vel_msg)
            self.pos_x += self.vel_msg.linear.x
            # print(self.pos_x)

            #print(self.curr_pose)
            #print(rospy.get_time())
            
            if self.pos_x >= 65:
                print('trying to kill node')
                self.vel_msg.linear.x = 0.0
                self.robot_vel_pub.publish(self.vel_msg)
                rosnode.kill_nodes(['move_test.py'])
                
                rospy.signal_shutdown("clifford reaches its destination")
                # os.system("rosnode kill "+ "Mover")
            rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    try:
        Clifford_velodyne()
    except rospy.ROSInterruptException:
        pass

