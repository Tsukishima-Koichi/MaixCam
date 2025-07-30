from maix import time
import math

class PIDIncrementalController:
    def __init__(self, P, I, D, sample_time):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = sample_time
        self.current_time = time.ticks_ms()
        self.last_time = self.current_time
        self.clear()

    def limit(self, olimit):
        self.olimit = olimit

    def set_point(self, setpoint):
        self.TarPoint = setpoint

    def clear(self):
        self.TarPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.error = 0.0
        self.pre_error = 0.0
        self.ppre_error = 0.0
        self.output = 0.0

    def update(self, feedback_value):
        self.current_time = time.ticks_ms()
        delta_time = self.current_time - self.last_time

        if (delta_time >= self.sample_time):
            self.error = self.TarPoint - feedback_value
            # error = feedback_value - self.TarPoint
            # if (abs(self.error) <= 1):
            #     self.error = 0
            
            self.PTerm = self.Kp * (self.error - self.pre_error)  # 比例
            self.ITerm = self.Ki * self.error # 积分
            if (abs(self.error) <= 2):
                self.DTerm = 0
            else:
                self.DTerm = self.Kd * (self.error - 2*self.pre_error + self.ppre_error) # 微分

            self.ppre_error = self.pre_error
            self.pre_error = self.error

        self.output = self.PTerm + self.ITerm + self.DTerm
        self.output = max(-self.olimit, min(self.output, self.olimit))
        
    


class PIDPositionController:
    def __init__(self, P, I, D, sample_time):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = sample_time
        self.current_time = time.ticks_ms()
        self.last_time = self.current_time
        self.clear()

    def limit(self, int_error, dif_error, slimit, olimit):
        self.int_error = int_error
        self.dif_error = dif_error
        self.slimit = slimit
        self.olimit = olimit

    def clear(self):
        self.TarPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        
    def update(self, feedback_value):
        error = self.TarPoint - feedback_value
        # error = feedback_value - self.TarPoint
        self.current_time = time.ticks_ms()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error # 比例

            if abs(error) > self.int_error:
                self.ITerm += self.Ki * error * delta_time # 积分
            self.ITerm = max(-self.slimit, min(self.ITerm, self.slimit))

            self.DTerm = 0.0
            if abs(delta_error) > self.dif_error:
                self.DTerm = self.Kd * delta_error / delta_time#微分

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + self.ITerm + self.DTerm
            self.output = max(-self.olimit, min(self.output, self.olimit))

    def set_point(self, setpoint):
        self.TarPoint = setpoint