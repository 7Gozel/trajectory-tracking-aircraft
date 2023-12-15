class PID:
    def __init__(self, ki, kp, kd, dt, upper_saturation, lower_saturation, anti_windup=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.time_step = max(dt, 1e-6)
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.derivative_error = 0.0
        self.upper_saturation = upper_saturation
        self.lower_saturation = lower_saturation
        self.output = 0.0
        self.anti_windup = anti_windup

    def reset_error(self):
        self.error = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.output = 0.0

    def set_parameters(self, kp, ki, kd, upper_saturation, lower_saturation):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.upper_saturation = upper_saturation
        self.lower_saturation = lower_saturation

    def compute(self, target, pos):
        self.error = target - pos
        self.integral_error += self.error * self.time_step
        prev_integral_error = self.integral_error
        if self.anti_windup and self.ki != 0.0:
            if self.output >= self.upper_saturation:
                self.integral_error = prev_integral_error  # self.upper_saturation / self.ki
            elif self.output <= self.lower_saturation:
                self.integral_error = prev_integral_error  # self.lower_saturation / self.ki

        self.derivative_error = (self.error - self.prev_error) / self.time_step
        self.prev_error = self.error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error

        if self.output >= self.upper_saturation:
            self.output = self.upper_saturation
        elif self.output <= self.lower_saturation:
            self.output = self.lower_saturation
        return self.output
