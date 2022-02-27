import math


class PIDController:
    """
    PID 控制算法，可以用两个 PID 分别控制速度和 yaw
    参考 https://github.com/fan0210/DJIM100-people-detect-track/blob/master/dji_sdk_demo/src/control/PID/PIDController.cpp
    和 https://zhuanlan.zhihu.com/p/39573490
    """
    def __init__(self, kP, kI, kD, max_out):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.max_out = max_out
        self.P = 0
        self.I = 0
        self.D = 0
        self.prev_err = 0
        self.last_err = 0
        self.cur_err = 0
        # self.last_PID_out = 0

    def getOutput(self, curerr):
        self.prev_err = self.last_err
        self.last_err = self.cur_err
        self.cur_err = curerr

        self.P = self.kP * self.cur_err
        self.I = self.kI * (self.cur_err + self.last_err + self.prev_err)
        self.D = self.kD * (self.cur_err - self.prev_err)

        pwm_value = self.P + self.I + self.D
        # PID_out = self.last_PID_out + pwm_value
        PID_out = pwm_value

        # self.last_PID_out = PID_out

        if PID_out > self.max_out:
            PID_out = self.max_out
        if PID_out < -1 * self.max_out:
            PID_out = -1 * self.max_out

        return PID_out