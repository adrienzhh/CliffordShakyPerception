#!/usr/bin/env python3

import rospy
import subprocess
import roslaunch
import time

def launch_file():
    started = False
    stop = False
    count = 0
    omega = 1 # pitch frequency (hz)
    x_velocity = 1
    # rospy.init_node('iterate_test', anonymous=True)
    # time_0 = rospy.get_time()

    try: 
        while stop == False:
                if started == False:
                    # time = rospy.get_time() - time_0
                    print("file count: ", count)
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    cli_args = ['/home/zhh/catkin_ws/src/Clifford_livox_sim-main/launch/hans_velodyne_iterate_test.launch', 'file_count:='+str(count), 'omega:='+str(omega)]
                    roslaunch_args = cli_args[1:]
                    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
                    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                    parent.start()
                    time.sleep(5)
                    execute_script(omega)
                    time.sleep(10)
                    parent.shutdown()
                    print('shutting down')
                    # parent.spin()  # Keep the launch file running until explicitly stopped
                    count += 1
                    started = True
                try: 
                    rospy.get_master().getPid()
                except:
                    time.sleep(1)
                    started = False
    
    except KeyboardInterrupt:
        # If the user interrupts the program with Ctrl+C, shut down both launches and exit
        parent.shutdown()
        print('\nBoth launches have been shut down.')
        exit(0)

def execute_script(omega):
    script_path = '/home/zhh/catkin_ws/src/Clifford_livox_sim-main/script/hans_move_short_traj.py'
    argument1 = str(omega)
    # argument2 = str(x_velocity)
    subprocess.Popen(['python3', script_path, argument1])

if __name__ == '__main__':
    launch_file()