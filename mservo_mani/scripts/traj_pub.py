#!/usr/bin/env python
# license removed for brevity
import rospy
import csv
from mservo_mani.msg import trajectory

dir = '/home/mservo/test_ws/src/mservo_mani/scripts/data.txt'

file = open(dir, 'r')
reader = csv.reader(file)

data = []

for line in reader:
    line_data = [float(item) for item in line]
    data.append(line_data)

def traj_pub():
    pub = rospy.Publisher('traj', trajectory, queue_size=1)
    rospy.init_node('agent', anonymous=True)
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        pub.publish(data[i])
        if i == data.__len__()-1:
            break
        else:
            i = i + 1
        rate.sleep()

if __name__ == '__main__':
    traj_pub()
