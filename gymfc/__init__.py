from gym.envs.registration import register
import math


continuous_kwargs = {
     "memory_size": 1,
     "world": "attitude-iris.world", 
     "omega_bounds": [-math.pi*2,math.pi*2],
     "command_time_off":[0.1, 1.0],
     "command_time_on":[0.1, 1.0],
     "max_sim_time": 60,
     }
id = 'QuadcopterFCContinuous-v0'
register(
    id=id,
    entry_point='gymfc.envs:AttitudeContinuousControlEnv',
    kwargs=continuous_kwargs)

episodic_damage_kwargs = {
    "memory_size": 1,
    "world": "attitude-iris-motor-damage.world", 
    "omega_bounds": [-math.pi*2,math.pi*2],
    "max_sim_time": 4.,
    "motor_count":4,
    }
id = 'QuadcopterFCEpisodicDamage-v0'.format(i)
register(
    id=id,
    entry_point='gymfc.envs:AttitudeControlEnv',
    kwargs=episodic_kwargs)


episodic_kwargs = {
    "memory_size": 1,
    "world": "attitude-iris.world", 
    "omega_bounds": [-math.pi*2,math.pi*2],
    "max_sim_time": 1.,
    "motor_count":4,
    }
id = 'QuadcopterFCEpisodic-v0'
register(
    id=id,
    entry_point='gymfc.envs:AttitudeControlEnv',
    kwargs=episodic_kwargs)

