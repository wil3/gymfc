from gym.envs.registration import register
import math
MAX_MEMORY = 11

default_kwargs = {
    "world": "attitude-iris.world", 
    "omega_bounds": [-math.pi*2,math.pi*2],
    "motor_count":4,
    "setup_file": "/usr/share/gazebo-8/setup.sh",
    "rate_units": "rad/s",
    "rotor_units": "RPM",
    # For setting up network connections to the aircraft and gazebo
    "hostname": "localhost",
    # If this is False then a random port will be sampled from the supplied range.
    # This is helpful when running multiple in parellel and it takes time to 
    # tear down an old connection. If this is set to True then the
    # start port is used. 
    "use_static_network_ports": False,
    "aircraft_start_port": 9005,
    "aircraft_end_port": 10005,
    "gazebo_start_port": 11345,
    "gazebo_end_port": 12345,
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
