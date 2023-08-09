import subprocess
import roslaunch
import time
import rospy

def launch_file():
    started = False
    stop = False 
    count = 0
    omega = 0  # pitch frequency (hz)

    try: 
        while stop == False:
            if started == False:
                print("file count: ", count)

                # Launch hans_velodyne_iterate_test.launch
                hans_launch = launch_hans_velodyne_iterate_test(count, omega)
                # time.sleep(5)
                time.sleep(2)
                initialize(0)
                # Launch run_record.launch
                time.sleep(2)
                record_launch = launch_run_record()
                time.sleep(2)
                move(omega)
                # rospy.spin()
                time.sleep(2)
                # Shutdown both launches
                hans_launch.shutdown()
                record_launch.shutdown()
                print('Both launches have been shut down.')

                # Increment the count and reset started flag
                count += 1
                started = False

    except KeyboardInterrupt:
        # If the user interrupts the program with Ctrl+C, shut down both launches and exit
        hans_launch.shutdown()
        record_launch.shutdown()
        print('\nBoth launches have been shut down.')
        exit(0)

def launch_hans_velodyne_iterate_test(count, omega):
    # Function to launch the "hans_velodyne_iterate_test.launch" file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = ['/home/zhh/catkin_ws/src/Clifford_livox_sim-main/launch/hans_velodyne_iterate_test_2.launch', 'file_count:='+str(count), 'omega:='+str(omega)]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
#    time.sleep(5)
#    move(omega)
#    time.sleep(3)
    
    print("successfully initilized--------------------------------------------------------")
    return parent

def launch_run_record():
    # Function to launch the "run_record.launch" file
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Replace the path with the correct path to "run_record.launch"
    cli_args = ['/home/zhh/catkin_ws/src/LIO-SAM-master/launch/run.launch']
    roslaunch_args = []
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    return parent

def move(omega):
    script_path_0 = '/home/zhh/catkin_ws/src/Clifford_livox_sim-main/script/hans_move_short_traj.py'
    argument = str(omega)
    subprocess.call(['python3', script_path_0, argument])

def initialize(omega):
    script_path_1 = '/home/zhh/catkin_ws/src/Clifford_livox_sim-main/script/hans_move_initial.py'
    # argument = str(omega) # omega 0 for initialize
    subprocess.call(['python3', script_path_1])

if __name__ == '__main__':
    launch_file()

