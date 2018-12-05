from gym.envs.registration import register
import math
MAX_MEMORY = 11

default_kwargs = {
}

#Episodic task with ESC supporting sensors for telemetry
kwargs = {
    "memory_size": 1,
    "max_sim_time": 1.,
    }
kwargs.update(default_kwargs)
id = 'AttFC_GyroErr-MotorVel_M4_Ep-v0'
register(
    id=id,
    entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackEnv',
    kwargs=kwargs)

# Optionally allow different memories
for i in range(1,MAX_MEMORY):
    kwargs = {
        "memory_size": i,
        "max_sim_time": 1.,
        }
    kwargs.update(default_kwargs)
    id = 'AttFC_GyroErr{}-MotorVel{}_M4_Ep-v0'.format(i, i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackEnv',
        kwargs=kwargs)


# Continuous task
kwargs = {
     "memory_size": 1,
     "command_time_off":[0.1, 1.0],
     "command_time_on":[0.1, 1.0],
     "max_sim_time": 60,
     }
kwargs.update(default_kwargs)
id = 'AttFC_GyroErr-MotorVel_M4_Con-v0'
register(
    id=id,
    entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackContinuousEnv',
    kwargs=kwargs)

# And with extra memory
for i in range(1,MAX_MEMORY):
    kwargs = {
         "memory_size": i,
         "command_time_off":[0.1, 1.0],
         "command_time_on":[0.1, 1.0],
         "max_sim_time": 60,
         }
    kwargs.update(default_kwargs)
    id = 'AttFC_GyroErr{}-MotorVel{}_M4_Con-v0'.format(i, i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorESCVelocityFeedbackContinuousEnv',
        kwargs=kwargs)


# For flight control systems without ESC sensors
for i in range(1,MAX_MEMORY):
    kwargs = {
        "memory_size": i,
        "max_sim_time": 1.,
        }
    kwargs.update(default_kwargs)
    id = 'AttFC_GyroErr{}_M4_Ep-v0'.format(i)
    register(
        id=id,
        entry_point='gymfc.envs:GyroErrorFeedbackEnv',
        kwargs=kwargs)
