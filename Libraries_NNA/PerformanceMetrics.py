# %% Import Libraries
import numpy as np

# %%
class PerformanceMetrics:
    def __init__(self, time, ref):
        self.time = time
        self.ref = ref
    
    def step_response(self, feedback):
        # feedback = np.mean(feedback, axis=0)  # Average over all samples
        ref_scalar = self.ref if np.isscalar(self.ref) else self.ref[-1]
        self.rise_time = next(t for t, y in zip(self.time, feedback) if y >= 0.9 * ref_scalar)
        self.settling_time = next(t for t in reversed(self.time) if abs(feedback[np.where(self.time == t)][0] - ref_scalar) > 0.02 * ref_scalar)
        self.overshoot = max(0.0, (max(feedback) - ref_scalar) / ref_scalar * 100)
        self.ss_error = abs(feedback[-1] - ref_scalar)
        dt = self.time[1] - self.time[0]
        self.total_AE = np.sum(np.abs(self.ref - feedback)) * dt # int over time
        epsilon = 1e-6  # to avoid division by zero
        self.total_RE = np.sum(np.abs(self.ref - feedback) / (np.abs(self.ref) + epsilon)) * dt # int over time
        return self.rise_time, self.settling_time, self.overshoot, self.ss_error, self.total_AE, self.total_RE
    
    def robustness(self, feedback):
        mean = np.mean(feedback, axis=0)
        deviation = np.std(feedback, axis=0)
        return np.mean(mean), np.mean(deviation)

    def sensitivity(self, samples, feedback):
        sensitivity_results = {}
        rise_times, settling_times, overshoots, ss_errors = [], [], [], []
        for i in range(feedback.shape[0]): 
            response = feedback[i]
            rise_time = next(t for t, y in zip(self.time, response) if y >= 0.9 * self.ref)
            settling_time = next(t for t in reversed(self.time) if abs(response[np.where(self.time == t)][0] - self.ref) > 0.02 * self.ref)
            overshoot = (max(response) - self.ref) / self.ref * 100
            ss_error = abs(response[-1] - self.ref)
            
            rise_times.append(rise_time)
            settling_times.append(settling_time)
            overshoots.append(overshoot)
            ss_errors.append(ss_error)

        for param_name, param_values in samples.items():
            param_values = np.array(param_values)
            sensitivity_results[param_name] = {
                "Rise Time": np.corrcoef(param_values, rise_times)[0, 1],
                "Settling Time": np.corrcoef(param_values, settling_times)[0, 1],
                "Peak Overshoot": np.corrcoef(param_values, overshoots)[0, 1],
                "Steady-State Error": np.corrcoef(param_values, ss_errors)[0, 1]
            }
        
        return sensitivity_results

    def to_plot(self, feedback):
        avg = np.mean(feedback, axis=0)
        mu_025 = np.percentile(feedback, 2.5, axis=0)
        mu_975 = np.percentile(feedback, 97.5, axis=0)
        return avg, mu_025, mu_975

# %%
