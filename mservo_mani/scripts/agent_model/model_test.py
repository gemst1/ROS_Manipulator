import tensorflow as tf
import numpy as np

check_point_dr= './save/'


def gravity_1dof(self, m1, l1, position):
    g = 9.81
    gravity = m1 * g * (l1 / 2) * np.cos(position[0] + np.pi / 2)
    return gravity

sess = tf.Session()
saver = tf.train.import_meta_graph(check_point_dr+'epoch-1499.meta')
chkpt = tf.train.latest_checkpoint(check_point_dr)
saver.restore(sess, chkpt)

tf.get_default_graph()

print(sess.graph.get_collection('variables'))
print(sess.graph.get_collection('variables','actor'))

obs = sess.graph.get_tensor_by_name("obs0:0")
action = sess.graph.get_tensor_by_name("actor/Tanh_2:0")

test_obs = np.array([0, 0, 0, 0, 0]).reshape((1,-1))
action_output = sess.run(action, feed_dict={obs: test_obs})
#

