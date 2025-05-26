# %% Controller
class PID_controller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.prev_error = 0
    
    def control(self, mea, ref, dt):
        error = ref - mea
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        PID = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return PID

# %%
