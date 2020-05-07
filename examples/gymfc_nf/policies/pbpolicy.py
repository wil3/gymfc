from .policy import Policy
class PbPolicy(Policy):
    """A policy to evaluate a PB format graph"""
    def __init__(self, sess, input_tensor, output_tensor):
        """Create an instance of the policy.
        
        Args: 
            sess: The tensorflow session
            input_tensor: The input tensor typically found using
            graph.get_tensor_by_name()
            output_tensor: The output tensor typically found using
            graph.get_tensor_by_name()
        """
        
        self.sess = sess
        self.input_tensor = input_tensor
        self.output_tensor = output_tensor


    def action(self, state, sim_time=0, desired=np.zeros(3), actual=np.zeros(3) ):
        y_out = self.sess.run(self.output_tensor, feed_dict={self.input_tensor:[state] })
        return y_out[0]

    def reset(self):
        pass
