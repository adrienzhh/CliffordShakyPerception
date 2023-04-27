import rosbag
import matplotlib.pyplot as plt

bag = rosbag.Bag("0.125_0_clifford_velodyne.bag")


x_gt = []
z_gt = []

x_est = []
z_est = []
i = 0
# end_time = bag.get_end_time
# print(end_time)
# start_time = end_time-1
# print(start_time)
# msg = bag.read_messages(topics=['/period_shaky/vo/pathGT'],start_time=end_time, end_time=end_time)
for topic, msg, t in bag.read_messages(topics=['/cliffordDrive']):
    x_gt.append(msg.linear.x)
    print(x_gt)

# bag.close()
# plt.rcParams.update({'font.size': 12})
# # print(x_gt)
# # print(z_gt)
# print(z_gt[0])
# print(z_est[0])
# plt.plot(x_gt,z_gt,label="Ground Truth")
# plt.plot(x_est,z_est, label="Periodic SLAM")
# plt.title('Frequency = 1.25 Hz')
# plt.xlabel('X Position (meters)')
# plt.ylabel('Z Position (meters)')
# plt.ylim(0.75,1.25)
# plt.xlim(-1.0,11)
# plt.legend()
# plt.show()