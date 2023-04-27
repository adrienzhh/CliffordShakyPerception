#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped, Vector3
import numpy as np
from gazebo_msgs.msg import ModelStates
import tkinter

class Clifford_velodyne:

    def __init__(self):
        # self.vel_msg = Twist()
        # self.head_pos_msg = Float64()
        # self.head_ang_msg = Float64()
        # self.curr_pose = PoseStamped()

        # rospy.init_node('Mover', anonymous=True)
        # self.robot_vel_pub = rospy.Publisher('/atom/cmd_vel/', Twist, queue_size=10)

        # self.mover()
        
        rospy.init_node('clifford_move')
        # Retrieve params:
        self._frequency = rospy.get_param('~frequency', 0.0)
        self._scale = rospy.get_param('~scale', 1.0)
        self._holonomic = rospy.get_param('~holonomic', False)

        # Create twist publisher:
        self._pub_cmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)

        # Initialize twist components to zero:
        self._v_x = 0.0
        self._v_y = 0.0
        self._w   = 0.0

        
        self.mover()

        # If frequency is positive, use synchronous publishing mode:
        # if self._frequency > 0.0:
        #     # Create timer for the given frequency to publish the twist:
        #     period = rospy.Duration(1.0 / self._frequency)

        #     self._timer = rospy.Timer(period, self._publish_twist)
        #     # self.mover()

        # Start window event manager main loop:


        

    # def pose_callback(self,msg):
    #     position = msg.pose[1].position.y
    #     self.curr_pose = position
    
        
        
        
        
        
        
        

    def mover(self):

        v_x = 0.5
        v_y = 0.0
        w = 0.0
        
        
        linear  = Vector3(v_x, v_y, 0.0)
        angular = Vector3(0.0, 0.0, w)
        
        twist = Twist(linear, angular)
        # print(twist)
        rate = rospy.Rate(10) # 10hz
        time_0 = rospy.get_time()
        
        
        # rate = rospy.Rate(10) # 10hz
        # time_0 = rospy.get_time()
        # omega = 0.125

        print("Clifford velodyne controller initialized")

        while not rospy.is_shutdown():
            time = rospy.get_time() - time_0
            # print(self.head_pos_msg.data)
            # print(self.head_ang_msg.data)
            print(twist)
            
            self._pub_cmd.publish(twist)

            #print(self.curr_pose)
            #print(rospy.get_time())
            rate.sleep()


Clifford_velodyne()
# if __name__ == '__main__':
#     try:
#         Shakey_Perception()
#     except rospy.ROSInterruptException:
#         pass



# class MouseTeleop():
#     def __init__(self):
#         # Retrieve params:
#         self._frequency = rospy.get_param('~frequency', 0.0)
#         self._scale = rospy.get_param('~scale', 1.0)
#         self._holonomic = rospy.get_param('~holonomic', False)

#         # Create twist publisher:
#         self._pub_cmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)

#         # Initialize twist components to zero:
#         self._v_x = 2.0
#         self._v_y = 0.0
#         self._w   = 0.0

#         # Initialize mouse position (x, y) to None (unknown); it's initialized
#         # when the mouse button is pressed on the _start callback that handles
#         # that event:
#         self._x = None
#         self._y = None
#         # self._send_motion

#         # If frequency is positive, use synchronous publishing mode:
#         if self._frequency > 0.0:
#             # Create timer for the given frequency to publish the twist:
#             period = rospy.Duration(1.0 / self._frequency)

#             self._timer = rospy.Timer(period, self._publish_twist)

#         # Start window event manager main loop:
#         # self._root.mainloop()
#         # self._send_motion

#     def __del__(self):
#         if self._frequency > 0.0:
#             self._timer.shutdown()

#         # self._root.quit()

#     # def _start(self, event):
#     #     self._x, self._y = event.y, event.x

#     #     self._y_linear = self._y_angular = 0

#     #     self._v_x = self._v_y = self._w = 0.0

#     # def _release(self, event):
#     #     self._v_x = self._v_y = self._w = 0.0

#     #     self._send_motion()

#     # def _configure(self, event):
#     #     self._width, self._height = event.height, event.width

#     #     self._c_x = self._height / 2.0
#     #     self._c_y = self._width  / 2.0

#     #     self._r = min(self._height, self._width) * 0.25

#     # def _mouse_motion_linear(self, event):
#     #     self._v_x, self._v_y = self._relative_motion(event.y, event.x)

#     #     self._send_motion()

#     # def _mouse_motion_angular(self, event):
#     #     self._v_x, self._w = self._relative_motion(event.y, event.x)

#     #     self._send_motion()

#     # def _update_coords(self, tag, x0, y0, x1, y1):
#     #     x0 += self._c_x
#     #     y0 += self._c_y

#     #     x1 += self._c_x
#     #     y1 += self._c_y

#     #     self._canvas.coords(tag, (x0, y0, x1, y1))

#     # def _draw_v_x(self, v):
#     #     x = -v * float(self._width)

#     #     self._update_coords('v_x', 0, 0, 0, x)

#     # def _draw_v_y(self, v):
#     #     y = -v * float(self._height)

#     #     self._update_coords('v_y', 0, 0, y, 0)

#     # def _draw_w(self, w):
#     #     x0 = y0 = -self._r
#     #     x1 = y1 =  self._r

#     #     self._update_coords('w', x0, y0, x1, y1)

#     #     yaw = w * numpy.rad2deg(self._scale)

#     #     self._canvas.itemconfig('w', extent=yaw)

#     def _send_motion(self):
#         v_x = self._v_x * self._scale
#         v_y = self._v_y * self._scale
#         w   = self._w   * self._scale

#         linear  = Vector3(v_x, v_y, 0.0)
#         angular = Vector3(0.0, 0.0,   w)

#         self._draw_v_x(self._v_x)
#         if self._holonomic:
#             self._draw_v_y(self._v_y)
#         self._draw_w(self._w)

#         if self._holonomic:
#             self._text_v_x.set('v_x = %0.2f m/s' % self._v_x)
#             self._text_v_y.set('v_y = %0.2f m/s' % self._v_y)
#             self._text_w.set(  'w   = %0.2f deg/s' % np.rad2deg(self._w))
#         else:
#             self._text_v_x.set('Throttle = %0.2f' % v_x)
#             self._text_w.set(  'Steering = %0.2f' % w)

#         twist = Twist(linear, angular)
        
#         rate = rospy.Rate(10) # 10hz
#         time_0 = rospy.get_time()
#         omega = 0.125

#         print("Clifford velodyne controller initialized")

#         while not rospy.is_shutdown():
#             time = rospy.get_time() - time_0
#             # print(self.head_pos_msg.data)
#             # print(self.head_ang_msg.data)
#             self._pub_cmd.publish(twist)
#         # self._pub_cmd.publish(twist)

#     def _publish_twist(self, event):
#         self._send_motion()

#     # def _relative_motion(self, x, y):
#     #     dx = self._x - x
#     #     dy = self._y - y

#     #     dx /= float(self._width)
#     #     dy /= float(self._height)

#     #     dx = max(-1.0, min(dx, 1.0))
#     #     dy = max(-1.0, min(dy, 1.0))

#     #     return dx, dy

#     # def _change_to_motion_linear(self, event):
#     #     if self._y is not None:
#     #         y = event.x

#     #         self._y_angular = self._y - y
#     #         self._y         = self._y_linear + y

#     # def _change_to_motion_angular(self, event):
#     #     if self._y is not None:
#     #         y = event.x

#     #         self._y_linear = self._y - y
#     #         self._y        = self._y_angular + y


# def main():
#     rospy.init_node('mouseControl')

#     MouseTeleop()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass






