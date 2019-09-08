#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import tensorflow as tf
from std_msgs.msg import String
from mservo_mani.msg import motor_data
from mservo_mani.msg import des_toq

check_point_dr= './agent_model/save/'

sess = tf.Session()
saver = tf.train.import_meta_graph(check_point_dr+'epoch-1499.meta')
chkpt = tf.train.latest_checkpoint(check_point_dr)
saver.restore(sess, chkpt)

tf.get_default_graph()

obs = sess.graph.get_tensor_by_name("obs0:0")
action = sess.graph.get_tensor_by_name("actor/Tanh_2:0")

pos = 0
vel = 0
pre_vel = 0
toq = 0
count = 0
obs_state = np.array([0.5, 0.5, 0.5, 0.5, 1.0])

def gravity_1dof(m1, l1, position):
    m1=6.8351
    l1=0.3280
    g = 9.81
    gravity = -1*m1 * g * (l1 / 2) * np.cos(position + np.pi / 2)

    return gravity

def callback(data):
    global pos
    pos = (data.pos/160.0/4096.0/4.0*2.0*np.pi)
    if pos < 0:
        pos2 = pos+ 2 * np.pi
    else:
        pos2 = pos

    if pos2 > np.pi:
        pos2 = pos2 - np.pi
    else:
        pos2 = pos2 + np.pi

    global pre_vel
    pre_vel = vel

    global vel
    vel = data.vel*2.0*np.pi/60.0/160.0

    global toq
    toq = data.toq

    pos_state = (pos2/2*np.pi)
    vel_magnitude = np.abs(vel)/20 + 0.5
    vel_sign = (np.sign(vel))/4 + 0.5
    before_state = (vel-pre_vel)/20 + 0.5
    global count
    if count < 500:
        torque_state=0.0
    elif count < 1000:
        torque_state = 0.99
    else:
        torque_state = 0.0

    global obs_state
    obs_state = np.array([pos_state,vel_magnitude,vel_sign,before_state,torque_state])

def agent():
    global count
    pub = rospy.Publisher('des_toq', des_toq, queue_size=1)
    rospy.init_node('agent', anonymous=True)
    rospy.Subscriber("motor_state", motor_data, callback)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        action_output = sess.run(action, feed_dict={obs: obs_state.reshape((1,-1))})
        gravity=gravity_1dof(6.8351, 0.3280, pos)
        compensation_torque = gravity+0.6* action_output[0,0]* vel + 5.0/3.0*action_output[0,1]*np.sign(vel)
        if count < 500:
            compensation_torque = compensation_torque*1000.0/263.295*1000.0/160.0
        elif count < 1000:
            compensation_torque = compensation_torque * 1000.0 / 263.295 * 1000.0 / 160.0 + 100.0
        else:
            compensation_torque = compensation_torque * 1000.0 / 263.295 * 1000.0 / 160.0
        print(count, pos, vel, toq, compensation_torque-gravity*1000.0/263.295*1000.0/160.0)
        pub.publish(compensation_torque)
        # rospy.spin()
        count = count+1
        rate.sleep()

if __name__ == '__main__':
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
    agent()