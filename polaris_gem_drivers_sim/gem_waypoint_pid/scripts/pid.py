#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.set_point = set_point

        self.sample_time = sample_time

        self.out_limits = out_limits

        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback_val):
        """Compute PID control value based on feedback_val.
        """

        # TODO: implement PID control
        error = self.set_point - feedback_val
        delta_error = error - self.last_err
        cur_time = time.time()
        delta_time = cur_time - self.last_time

        if (delta_time >= self.sample_time):
            self.p_term = self.Kp * error
            self.i_term += self.Ki * (error + delta_time)
            if self.i_term > self.out_limits[1]:
                self.i_term = self.out_limits[1]
            if self.i_term < self.out_limits[0]:
                self.i_term = self.out_limits[0]
            self.d_term = self.Kd * (delta_error / delta_time)

        self.last_err = error
        self.last_time = cur_time

        self.output = self.p_term + self.i_term + self.d_term
        
        return self.output


    def __call__(self, feeback_val):
        return self.update(feeback_val)
