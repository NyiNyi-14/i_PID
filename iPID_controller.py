# %%
class iPID_controller:
    def __init__(self, Kp, Ki, Kd, alpha = 1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.alpha = alpha
        self.integral = 0
        self.prev_error = 0
        self.prev_meas = 0
        self.prev_ref = 0
 
    def control(self, mea, ref, dt):
        error = ref - mea
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        PID = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
 
        d_mea = (mea - self.prev_meas) / dt
        self.prev_meas = mea
        d_ref = (ref - self.prev_ref) / dt
        self.prev_ref = ref
        u = PID * self.alpha

        info = -d_mea + u + d_ref
        iPID = PID + info
        return iPID

# %%
