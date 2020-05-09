from .policy import Policy
import numpy as np
from gymfc_nf.controllers.pid import PidController 

class PidPolicy(Policy):
    def __init__(self, r_pid, p_pid, y_pid, mixer):
        self.controller = PidController(pid_roll = r_pid, pid_pitch = p_pid,
                                        pid_yaw = y_pid, mixer = mixer )

    def action(self, state, sim_time=0, desired=np.zeros(3), actual=np.zeros(3) ):
        motor_values = np.array(self.controller.calculate_motor_values(sim_time, desired, actual))
        # Need to scale from 1000-2000 to -1:1
        return np.array( [ (m - 1000)/500  - 1 for m in motor_values])

    def reset(self):
        self.controller.reset()

