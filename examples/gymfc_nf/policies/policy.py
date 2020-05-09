class Policy(object):
    """ Base policy """
    def __init__(self, action_low=None, action_high=None):
        self.action_low = action_low
        self.action_high = action_high

    def action(self, state, sim_time=0, desired=[], actual=[]):
        pass

    def reset(self):
        pass
