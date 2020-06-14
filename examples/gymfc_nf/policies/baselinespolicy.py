import numpy as np
import tensorflow as tf
from .policy import Policy
class PpoBaselinesPolicy(Policy):
    def __init__(self, sess):
        graph = tf.get_default_graph()
        self.x = graph.get_tensor_by_name('pi/ob:0') 
        self.y = graph.get_tensor_by_name('pi/pol/final/BiasAdd:0')
        self.sess = sess

    def action(self, state, sim_time=0, desired=np.zeros(3), actual=np.zeros(3) ):

        y_out = self.sess.run(self.y, feed_dict={self.x:[state] })
        return y_out[0]
