import csv
import numpy as np

dir = 'data2.txt'

file = open(dir, 'r', encoding='utf-8')
reader = csv.reader(file)

data = []

for line in reader:
    data.append(line)

traj_data = np.array(data)
traj_len = traj_data.shape[0]

def trajectory():
    pub = rospy.Publisher('traj', traj, queue_size=1)
    rospy.init_node('agent', anonymous=True)
    rate = rospy.Rate(25)
    i = 0
    while not rospy.is_shutdown():
        pub.publish(traj_data[i,:])
        if i == traj_len:
            break
        else:
            i = i + 1
        rate.sleep()

if __name__ == '__main__':
    trajectory()
