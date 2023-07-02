#!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Twist, PoseStamped
# from gazebo_msgs.msg import ModelStates, LinkStates
# import numpy as np
# import sys
# import tf


# class Mover:

#     def __init__(self):
#         self.vel_msg = Twist()
#         self.head_pos_msg = Float64()
#         self.head_ang_msg = Float64()
#         self.curr_pose = PoseStamped()
#         self.pos_z = 0.0
#         self.angle = 0.0
#         self.is_turning = True
#         self.is_finished = False
#         rospy.init_node('Mover', anonymous=True)
#         self.robot_vel_pub = rospy.Publisher('/hans_bot_wheel_controller/cmd_vel/', Twist, queue_size=10)
#         self.slider_pos_pub = rospy.Publisher('/hans_bot_slider_controller/command/', Float64, queue_size=10)
#         self.head_pos_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
#         self.des_pos_pub = rospy.Publisher('/desired_pose',PoseStamped,queue_size=10)
#         # self.robot_vel_pub = rospy.Publisher('/test_bot_2_wheel_controller/cmd_vel/', Twist, queue_size=10)
#         # self.slider_pos_pub = rospy.Publisher('/test_bot_2_slider_controller/command/', Float64, queue_size=10)
#         # self.head_pos_pub = rospy.Publisher('/test_bot_2_head_controller/command/', Float64, queue_size=10)

#         self.global_pose_sub = rospy.Subscriber('gazebo/link_states', LinkStates, callback=self.pose_callback)
#         self.mover()

#     def pose_callback(self,msg):
        
#         # print(len(msg.pose))
#         head_angle = msg.pose[37]
#         quaternion = (
#         head_angle.orientation.x,
#         head_angle.orientation.y,
#         head_angle.orientation.z,
#         head_angle.orientation.w)

#         self.pos_x = head_angle.position.x
#         self.pos_z = head_angle.position.z
        
#         euler = tf.transformations.euler_from_quaternion(quaternion)

#         self.angle = euler[2]
    
#     def mover(self):

      
#         self.vel_msg.linear.x = 0.5
#         self.vel_msg.angular.z = 0.5
#         self.head_pos_msg.data = 0.0
#         self.head_ang_msg.data =  0.0
#         rate = rospy.Rate(10) # 10hz
#         # print('cmd', sys.argv[1])
#         time_0 = rospy.get_time()
#         omega = 2 # float(sys.argv[1])# 0.125 # This value will change the speed of the oscillations


#         length = 5.0
#         width = 6.0
#         while not rospy.is_shutdown():
#             # angle_t = self.angle-angle_t1
#             # print(time)
#             time = rospy.get_time() - time_0
#             print(time)
            
#             if abs(time - 7) < 0.05:
#                 self.vel_msg.linear.x = 0.0
#                 # self.vel_msg.angular.z = 0.5

#                 if self.is_turning:
#                     target_angle = np.pi / 2  # Target angle for 90 degrees counterclockwise turn
#                     angle_error = target_angle - self.angle
#                     if abs(angle_error) <= 0.01:
#                         self.is_turning = False  # Stop turning
#                         self.is_finished = True  # Finish the movement
#                         self.vel_msg.angular.z = 0.0  # Stop angular velocity
#                     else:
#                         self.vel_msg.angular.z = 0.5  # Adjust angular velocity for turning


#                 # target_angle = -np.pi / 2  # Target angle for 90 degrees counterclockwise turn
#                 # angle_error = target_angle - self.angle
#                 # # self.vel_msg.angular.z = 1.0 * angle_error

#                 # if abs(angle_error) <= 0.01:
#                 #     # self.is_turning = False  # Stop turning
#                 #     # self.is_finished = True  # Finish the movement
#                 #     self.vel_msg.angular.z = 0.0  # Stop angular velocity
#                 # else:
#                 #     self.vel_msg.angular.z = 1.0 * angle_error  # Adjust angular velocity for turning


#             # x = length * (time % (2 * (length + width))) / (length + width)
#             # print(length)
#             # y = width * (time % (2 * (length + width))) / (length + width)

#             # # Calculate the desired angle based on the current position
#             # desired_angle = np.arctan2(y, x)
#             # print(desired_angle)
#             # # Calculate the distance to the desired angle
#             # angle_error = desired_angle - self.angle
#             # # Adjust the angular velocity based on the angle error
#             # self.vel_msg.angular.z = 1.5 * angle_error

#             # print(time)
#             self.head_pos_msg.data = 0.15*np.sin(omega*2*np.pi*time)
#             self.head_ang_msg.data = 0.436332*np.sin(omega*2*np.pi*time)
#             # print(self.head_ang_msg.data)
#             self.robot_vel_pub.publish(self.vel_msg)
#             self.slider_pos_pub.publish(self.head_pos_msg.data)
#             self.head_pos_pub.publish(self.head_ang_msg.data)

#             rate.sleep()


#         # length = 2.0
#         # width = 2.0
#         # while not rospy.is_shutdown():
#         #     time = rospy.get_time() - time_0
#         #     # Calculate the current position in the rectangular loop
#         #     x = length * (time % (2 * (length + width))) / (length + width)
#         #     y = width * (time % (2 * (length + width))) / (length + width)
#         #     # Calculate the desired angle based on the current position
#         #     desired_angle = np.arctan2(y, x)
#         #     # Calculate the distance to the desired angle
#         #     angle_error = desired_angle - self.angle
#         #     # Adjust the angular velocity based on the angle error
#         #     self.vel_msg.angular.z = 1.5 * angle_error
#         #     # Publish the velocity and head position commands
#         #     self.robot_vel_pub.publish(self.vel_msg)
#         #     self.head_pos_pub.publish(self.head_pos_msg)
#         #     self.head_ang_pub.publish(self.head_ang_msg)
#         #     rate.sleep()


# if __name__ == '__main__':
#     try:
#         Mover()
#     except rospy.ROSInterruptException:
#         pass




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
        self.is_turning = True
        self.is_finished = False
        rospy.init_node('Mover', anonymous=True)
        self.robot_vel_pub = rospy.Publisher('/hans_bot_wheel_controller/cmd_vel/', Twist, queue_size=10)
        self.slider_pos_pub = rospy.Publisher('/hans_bot_slider_controller/command/', Float64, queue_size=10)
        self.head_pos_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.head_ang_pub = rospy.Publisher('/hans_bot_head_controller/command/', Float64, queue_size=10)
        self.des_pos_pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
        self.global_pose_sub = rospy.Subscriber('gazebo/link_states', LinkStates, callback=self.pose_callback)
        self.mover()

    def pose_callback(self, msg):
        # print(len(msg.pose))
        head_angle = msg.pose[37]
        quaternion = (
            head_angle.orientation.x,
            head_angle.orientation.y,
            head_angle.orientation.z,
            head_angle.orientation.w
        )
        self.pos_x = head_angle.position.x
        self.pos_z = head_angle.position.z
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # print(euler)
        self.angle = euler[2]

    def mover(self):
        self.vel_msg.linear.x = 0.5  # Linear velocity (zero for no forward movement)
        self.vel_msg.angular.z = 0 # Angular velocity for 90 degrees turn (counterclockwise)
        self.head_pos_msg.data = 0.0
        self.head_ang_msg.data = 0.0
        rate = rospy.Rate(10)  # 10hz
        time_0 = rospy.get_time()
        
        target_angle = self.angle + np.pi / 2  # Target angle for 90 degrees counterclockwise turn
        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # print(time)
            if abs(time - 7) < 0.05:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = np.pi / 2
                # self.vel_msg.angular.z = 0.5
                # current_angle = self.angle
                # print(current_angle)
                # self.vel_msg.angular.z = np.pi / 2
                # if current_angle >= np.pi / 2:
                #     self.is_finished = True  # Finish the movement
                #     self.vel_msg.angular.z = 0.0  # Stop angular velocity
            
            # if abs(time - 10) < 0.05:
            current_angle = self.angle
            # print(current_angle)
            # self.vel_msg.angular.z = np.pi / 2
            if current_angle >= np.pi / 2:
                self.is_finished = True  # Finish the movement
                self.vel_msg.angular.z = 0.0  # Stop angular velocity
                self.vel_msg.linear.x = 0.5
            

            if abs(time - 12) < 0.05:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = np.pi / 2

            self.robot_vel_pub.publish(self.vel_msg)
            self.head_pos_pub.publish(self.head_pos_msg)
            self.head_ang_pub.publish(self.head_ang_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        Mover()
    except rospy.ROSInterruptException:
        pass
