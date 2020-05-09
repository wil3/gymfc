import numpy as np
import logging

class PID(object):

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.reset()
    
    def update(self, t, e):

        # TODO add anti-windup logic
        # Most environments have a short execution time 
        # the controller doesn't have much time to wind up
        dt = t - self.last_t
        self.last_t = t

        p_term = self.kp * e

        self.accum += e * dt
        i_term = self.ki * self.accum

        de = e - self.last_e
        self.last_e = e
        d_term = self.kd * de / dt if dt > 0 else 0

        return p_term + i_term + d_term 

    def reset(self):
        self.last_t = 0
        self.last_e = 0
        self.accum = 0

class PidController(object):
    """ This is a loose port from Betaflight """
    FD_ROLL = 0
    FD_PITCH = 1
    FD_YAW = 2
    PTERM_SCALE = 0.032029
    ITERM_SCALE = 0.244381
    DTERM_SCALE = 0.000529
    minthrottle = 1000
    maxthrottle = 2000

    def __init__(self, 
                 pid_roll = [40, 40, 30], 
                 pid_pitch = [58, 50, 35], 
                 pid_yaw = [80, 45, 20], 
                 mixer = [],
                 itermLimit = 150):

        # init gains and scale
        self.Kp = [pid_roll[0], pid_pitch[0], pid_yaw[0]]
        self.Kp = [self.PTERM_SCALE * p for p in self.Kp]

        self.Ki = [pid_roll[1], pid_pitch[1], pid_yaw[1]]
        self.Ki = [self.ITERM_SCALE * i for i in self.Ki]

        self.Kd = [pid_roll[2], pid_pitch[2], pid_yaw[2]]
        self.Kd = [self.DTERM_SCALE * d for d in self.Kd]

        self.itermLimit = itermLimit 

        self.previousRateError = [0]*3
        self.previousTime = 0 
        self.previous_motor_values = [self.minthrottle]*4
        self.pid_rpy = [PID(*pid_roll), PID(*pid_pitch), PID(*pid_yaw)]

        self.mixer = mixer

    def calculate_motor_values(self, current_time, sp_rates, gyro_rates):
        rpy_sums = []
        for i in range(3):
            u = self.pid_rpy[i].update(current_time, sp_rates[i] - gyro_rates[i])
            rpy_sums.append(u)
        return self.mix(*rpy_sums)

    def constrainf(self, amt, low, high):
        # From BF src/main/common/maths.h
        if amt < low:
            return low
        elif amt > high:
            return high
        else:
            return amt

    def mix(self, r, p, y):
        PID_MIXER_SCALING = 1000.0
        pidSumLimit = 10000.#500
        pidSumLimitYaw = 100000.#1000.0#400
        motorOutputMixSign = 1
        motorOutputRange = self.maxthrottle - self.minthrottle# throttle max - throttle min 
        motorOutputMin = self.minthrottle

        mixer_index_throttle = 0
        mixer_index_roll = 1
        mixer_index_pitch = 2 
        mixer_index_yaw = 3

        scaledAxisPidRoll = self.constrainf(r, -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING
        scaledAxisPidPitch = self.constrainf(p, -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING
        scaledAxisPidYaw = self.constrainf(y, -pidSumLimitYaw, pidSumLimitYaw) / PID_MIXER_SCALING
        scaledAxisPidYaw = -scaledAxisPidYaw

        # Find roll/pitch/yaw desired output
        motor_count = 4
        motorMix = [0]*motor_count
        motorMixMax = 0
        motorMixMin = 0
        # No additional throttle, in air mode
        throttle = 0
        motorRangeMin = 1000
        motorRangeMax = 2000

        for i in range(motor_count):
            mix = (scaledAxisPidRoll  * self.mixer[i][1] +
                scaledAxisPidPitch * self.mixer[i][2] +
                scaledAxisPidYaw   * self.mixer[i][3])

            if mix > motorMixMax:
                motorMixMax = mix
            elif mix < motorMixMin:
                motorMixMin = mix
            motorMix[i] = mix

        motorMixRange = motorMixMax - motorMixMin

        if motorMixRange > 1.0:
            for i in range(motor_count): 
                motorMix[i] /= motorMixRange
            # Get the maximum correction by setting offset to center when airmode enabled
            throttle = 0.5

        else:
            # Only automatically adjust throttle when airmode enabled. Airmode logic is always active on high throttle
            throttleLimitOffset = motorMixRange / 2.0
            throttle = self.constrainf(throttle, 0.0 + throttleLimitOffset, 1.0 - throttleLimitOffset)

        motor = []
        for i in range(motor_count):
            motorOutput = motorOutputMin + (motorOutputRange * (motorOutputMixSign * motorMix[i] + throttle * self.mixer[i][mixer_index_throttle]))
            motorOutput = self.constrainf(motorOutput, motorRangeMin, motorRangeMax);
            motor.append(motorOutput)

        motor = list(map(int, np.round(motor)))
        return motor

    def reset(self):
        for pid in self.pid_rpy:
            pid.reset()

