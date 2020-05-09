"""A collection of various states to be fed as input to the controller.
When initializing a gym, a state function must be provided so the gym is 
known of the states to send to the digital twin.

For neuroflight control best performance was achieved with i
state_degrees_error_deltaerror which is the state used by Neuroflight.

Remaining state functions are for reference and further experimentation.
"""
import numpy as np
import math


def state_degrees_error_deltaerror_q(local):
    error_delta = local.measured_error - local.last_measured_error 
    q = local.imu_orientation_quat
    return np.concatenate([local.measured_error, error_delta, q]) 

def state_degrees_error_deltaerror(local):
    error_delta = local.measured_error - local.last_measured_error 
    return np.concatenate([local.measured_error, error_delta]) 

def state_error_deltaerror(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)
    return np.concatenate([err_rad, error_delta]) 

def state_error_deltaerror_y(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)
    return np.concatenate([err_rad, error_delta, local.y]) 

def state_error_deltaerror_y_deltaomega(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)

    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    last_omega = np.array(list(map(math.radians, local.last_gyro)))

    deltaomega = omega - last_omega
    return np.concatenate([err_rad, error_delta, local.y, deltaomega]) 

def state_error_deltaerror_deltaomega(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)

    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    last_omega = np.array(list(map(math.radians, local.last_gyro)))

    deltaomega = omega - last_omega
    return np.concatenate([err_rad, error_delta, deltaomega]) 

def state_degrees_error_deltaerror_deltaomega(local):
    error_delta = local.measured_error - local.last_measured_error
    deltaomega = local.gyro - local.last_gyro
    return np.concatenate([local.measured_error, error_delta, deltaomega]) 

def state_error_deltaerror_omega_y_q(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    q = local.imu_orientation_quat
    return np.concatenate([err_rad, error_delta, local.y, omega, q]) 

def state_error_deltaerror_y_q(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    last_err_rad = np.array(list(map(math.radians, local.last_measured_error)))
    error_delta = (err_rad - last_err_rad)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    q = local.imu_orientation_quat
    return np.concatenate([err_rad, error_delta, local.y, q]) 

def state_error_y_omega(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    return np.concatenate([err_rad, omega , local.y]) 

def state_error_omega(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    return np.concatenate([err_rad, omega]) 

def state_error_omega_deltaomega(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    last_omega = np.array(list(map(math.radians, local.last_gyro)))# / math.radians(self.max_rate)
    deltaomega = omega - last_omega

    return np.concatenate([err_rad, omega, deltaomega]) 

def state_error_y_omega_localtion(local):
    err_rad = np.array(list(map(math.radians, local.measured_error)))# / math.radians(self.max_rate)
    omega = np.array(list(map(math.radians, local.gyro)))# / math.radians(self.max_rate)
    return np.concatenate([err_rad, omega , local.y]) 

def state_derivative(local):
    current_state = state_error_y_omega(local) 
    local.state_history.append(current_state)
    if len(local.state_history) == 1:
        return np.concatenate([current_state, np.zeros(len(current_state))])

    last_two = local.state_history[-2:]
    deltas = np.diff(last_two, axis=0)[0]
    s = np.concatenate([current_state, deltas])
    return s

def state_history(local):
    history_len = local.state_memory_size
    current_state = state_error_y_omega(local) 
    local.state_history.append(current_state)
    
    n = len(local.state_history)
    pad = []
    if n < history_len: # -1 because we will add one
        pad = np.zeros((history_len - n) * len(current_state)).tolist()

    actual_n = min(history_len, n)
    past_states = np.array(local.state_history[-actual_n:]).flatten().tolist()

    states = past_states + pad
    return np.array(states)

def state_error_quat(local):
    err_rad = np.array(list(map(math.radians, local.measured_error))) / math.radians(1000)
    q = local.imu_orientation_quat
    return np.concatenate([err_rad, q]) 

def state_error_motor(local):
    max_rpm = 21e3
    err_rad = np.array(list(map(math.radians, local.measured_error)))
    rpm_norm = local.esc_motor_angular_velocity/max_rpm 
    return np.concatenate([err_rad, rpm_norm]) 

def state_error_quat_motor(local):
    max_rpm = 21e3
    err_rad = np.array(list(map(math.radians, local.measured_error))) / math.radians(1000)
    q = local.imu_orientation_quat
    rpm_norm = self.esc_motor_angular_velocity/max_rpm 
    return np.concatenate([err_rad, q, rpm_norm]) 


