#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.msg import ModelStates, LinkStates
import numpy as np
import sys
import tf
import roslaunch
import time

class Automation:

    def __init__(self):
        self.pos_x = 0.0
        self.angle = 0.0
        self.freq = np.linspace(0.125,2.5,num=10).tolist()
        # self.freq = np.linspace(5.0,2.5,num=10, endpoint=False)[::-1].tolist()

        self.test_run()

    
    def test_run(self):
        started = False
        stop = False
        i = 8
        j = 2
        cnt = 2
        while stop == False:

            if started == False:
                # print('here')
                print('j',j)
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                cli_args = ['/home/zhh/catkin_ws/src/Clifford_livox_sim-main/launch/hans_velodyne_short_traj_liosam.launch']
                # roslaunch_args = cli_args[1:]
                # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
                roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]


                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                # j += 1

                # if cnt % 15 == 0 and cnt != 0:
                #     print('here')
                #     j = 0
                #     i += 1
                # # i += 1

                # cnt += 1
                parent.start()
                print("start")

                started = True

            try: 
                rospy.get_master().getPid()
            except:
                time.sleep(10)
                started = False
if __name__ == '__main__':
    try:
        Automation()
    except:
        pass