from gym.envs.registration import register
import math
MAX_MEMORY = 11

#Episodic task with ESC supporting sensors for telemetry
episodic_kwargs = {
    "memory_size": 1,
    "world": "attitude-iris.world", 
    "omega_bounds": [-math.pi*2,math.pi*2],
    "max_sim_time": 1.,
    "motor_count":4,
    }
id = 'AttFC_GyroErr-MotorVel_M4_Ep-v0'
register(
    id=id,
    entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackEnv',
    kwargs=episodic_kwargs)

# Optionally allow different memories
for i in range(1,MAX_MEMORY):
    episodic_kwargs = {
        "memory_size": i,
        "world": "attitude-iris.world", 
        "omega_bounds": [-math.pi*2,math.pi*2],
        "max_sim_time": 1.,
        "motor_count":4,
        }
    id = 'AttFC_GyroErr{}-MotorVel{}_M4_Ep-v0'.format(i, i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackEnv',
        kwargs=episodic_kwargs)


# Continuous task
continuous_kwargs = {
     "memory_size": 1,
     "world": "attitude-iris.world", 
     "omega_bounds": [-math.pi*2,math.pi*2],
     "command_time_off":[0.1, 1.0],
     "command_time_on":[0.1, 1.0],
     "max_sim_time": 60,
     }
id = 'AttFC_GyroErr-MotorVel_M4_Con-v0'
register(
    id=id,
    entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackContinuousEnv',
    kwargs=continuous_kwargs)

# And with extra memory
for i in range(1,MAX_MEMORY):
    continuous_kwargs = {
         "memory_size": i,
         "world": "attitude-iris.world", 
         "omega_bounds": [-math.pi*2,math.pi*2],
         "command_time_off":[0.1, 1.0],
         "command_time_on":[0.1, 1.0],
         "max_sim_time": 60,
         }
    id = 'AttFC_GyroErr{}-MotorVel{}_M4_Con-v0'.format(i, i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackContinuousEnv',
        kwargs=continuous_kwargs)


# For flight control systems without ESC sensors
for i in range(1,MAX_MEMORY):
    episodic_kwargs = {
        "memory_size": i,
        "world": "attitude-iris.world", 
        "omega_bounds": [-math.pi*2,math.pi*2],
        "max_sim_time": 1.,
        "motor_count":4,
        }
    id = 'AttFC_GyroErr{}_M4_Ep-v0'.format(i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorFeedbackEnv',
        kwargs=episodic_kwargs)
