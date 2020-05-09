from gym.envs.registration import register
from .states import *

"""OpenAI gym environments"""

register(
    id="gymfc_nf-step-v1",
    entry_point='gymfc_nf.envs.step:StepEnv',
    kwargs={
        "max_rate": 100,
        "state_fn":state_degrees_error_deltaerror,
        "pulse_width":2.048,
        "max_sim_time": 4.608
    }
)
register(
    id="gymfc_nf-continuous-v1",
    entry_point='gymfc_nf.envs.continuous:ContinuousEnv',
    kwargs={
        "max_rate": 300,
        "state_fn":state_degrees_error_deltaerror,
        "max_sim_time": 5.12,
        "pulse_width":1.024,
    }
)
