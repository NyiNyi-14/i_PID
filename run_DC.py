# %%
# Nyi Nyi Aung
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import time as tm
import seaborn as sns
import pandas as pd
import scipy.stats as stats
import pyDOE
from scipy.signal import find_peaks

import os
print(os.getcwd())
os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID/Libraries_NNA")
print(os.listdir())

# Local
from PID_controller import PID_controller
from iPID_controller import iPID_controller
from Closed_loop import Closed_loop
from DC_Motor import DC_motor
from InputShaping import InputShaping

# %%
# Test run with nominal value, to see DC motor and PID properly working
motor_params = {
    "R": (0.2, 0.1),
    "L": (0.5, 0.002),
    "Kb": (1.0, 0.005),
    "Kt": (1.0, 0.005),
    "J": (2.0, 0.002),
    "B": (0.05, 0.0002)
}

omega = 0
ia = 0
init = [ia, omega]

dt = 0.01  # Time step
duration = 100  # Simulation time in seconds
time = np.arange(0, duration, dt)
omega_ref = 80.0  # REF speed (rad/s)
TL = 100 * np.ones_like(time)  # Load torque
TL[:int(duration/dt/1.4)] = 0

motor_test = DC_motor(motor_params["R"][0], 
                 motor_params["L"][0], 
                 motor_params["Kb"][0], 
                 motor_params["Kt"][0], 
                 motor_params["J"][0],
                 motor_params["B"][0],
                 TL = TL)

Kp = 3.33648935332675
Ki = 0.656107950597057
Kd = -0.0141651780403183
PID = PID_controller(Kp, Ki, Kd)
iPID = iPID_controller(Kp, Ki, Kd)
PID_loop = Closed_loop(motor_test, PID)
iPID_loop = Closed_loop(motor_test, iPID)

time, state = PID_loop.simulate_extra(time, dt, omega_ref, init = init, control_index = 1, extra_args_func = TL)
time, i_state = iPID_loop.simulate_extra(time, dt, omega_ref, init = init, control_index = 1, extra_args_func = TL)

# %%
plt.plot(time, omega_ref * np.ones_like(time), label="Reference Speed (rad/s)")
plt.plot(time, state[:, 1], label="PID (rad/s)")
plt.plot(time, i_state[:, 1], label="iPID (rad/s)")
# plt.plot(time, ia_test)
plt.xlabel("Time (s)")
plt.ylabel("Motor Speed (rad/s)")
plt.title("DC Motor PID Control")
plt.legend()
plt.grid()
plt.show()


# %% Input shaper design
zeta = 0.0 # look at MATLAB Root Locus
omega_n = 2.31 # look at MATLAB Root Locus
IS = InputShaping(zeta, omega_n)

PIDpeak_indices, _ = find_peaks(state[:, 1], )
PIDpeak_values = state[:, 1][PIDpeak_indices]
PIDpeak_times = time[PIDpeak_indices]
print(f"Peak values of PID step: {PIDpeak_values}")
 
if len(PIDpeak_times) >= 2:
    PIDt_delta = PIDpeak_times[1] - PIDpeak_times[0]
    print(f"Time between first two peaks: {PIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
threshold_time = 70
after_70_mask = PIDpeak_times > threshold_time
after_70_indices = PIDpeak_indices[after_70_mask]
after_70_times = time[after_70_indices]
after_70_values = state[:, 1][after_70_indices]
load_dt = after_70_times[1] - after_70_times[0]

if len(after_70_times) >= 2:
    print(f"Second interval after {threshold_time}s: {load_dt:.4f} s")
else:
    print(f"Less than two peaks found after {threshold_time}s")
    
PIDt_switch = np.pi/ (2*np.pi*(1/PIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {PIDt_switch:.4f} seconds")


# %%
normal_is = 80 * IS.delay(time, PIDt_switch)
plt.plot(time, normal_is, label='Input Shaping')
robust_is = 80 * IS.robust_is(time, PIDt_switch)
plt.plot(time, robust_is, label='Input Shaping')

# %%

# Suppose `state[:, 1]` is your omega (motor speed) and `time` is your time array
inverted_signal = -state[:, 1]  # Invert to detect minima
min_peak_indices, _ = find_peaks(inverted_signal, height=None)

# Get the times and values of those minima
min_peak_values = state[:, 1][min_peak_indices]
min_peak_times = time[min_peak_indices]

print("Min peak values:", min_peak_values)
print("Min peak times:", min_peak_times)

# Example: time difference between two first dips after 70s
mask = min_peak_times > 70
min_peaks_after_70 = min_peak_times[mask]
if len(min_peaks_after_70) >= 2:
    dt_tt = min_peaks_after_70[1] - min_peaks_after_70[0]
    print(f"Time between first two dips after 70s: {dt_tt:.4f} seconds")

PID_Lswitch = np.pi/ (2*np.pi*(1/dt_tt) * np.sqrt(1-zeta**2))
print(f"t_switch for load: {PID_Lswitch:.4f} seconds")

normal_Lis = TL * IS.delay(time, PID_Lswitch + time[int(duration/dt/1.4)])
plt.plot(time, normal_Lis, label='Load Input Shaping')
pre = IS.robust_is(time, PID_Lswitch)
robust_Lis = np.zeros_like(pre)
robust_Lis[int(duration/dt/1.4):] = TL[-1] * pre[:len(time) - int(duration/dt/1.4)]
plt.plot(time, robust_Lis, label='Load Input Shaping')

# %%
time, state_is = PID_loop.simulate_extra(time, dt, normal_is, init = init, control_index = 1, extra_args_func = normal_Lis)
time, state_robust = PID_loop.simulate_extra(time, dt, robust_is, init = init, control_index = 1, extra_args_func = robust_Lis)

# %%
plt.plot(time, state_is[:, 1], label="PID normal IS (rad/s)")
plt.plot(time, state_robust[:, 1], label="PID robust IS (rad/s)")
# plt.plot(time, ia_test)
plt.xlabel("Time (s)")
plt.ylabel("Motor Speed (rad/s)")
plt.title("DC Motor PID Control")
plt.legend()
plt.grid()
plt.show()

# %% Input shaper design for iPID
zeta = 0.0 # look at MATLAB Root Locus
omega_n = 2.31 # look at MATLAB Root Locus
IS = InputShaping(zeta, omega_n)

iPIDpeak_indices, _ = find_peaks(i_state[:, 1], )
iPIDpeak_values = i_state[:, 1][iPIDpeak_indices]
iPIDpeak_times = time[iPIDpeak_indices]
print(f"Peak values of PID step: {iPIDpeak_values}")
 
if len(iPIDpeak_times) >= 2:
    iPIDt_delta = iPIDpeak_times[1] - iPIDpeak_times[0]
    print(f"Time between first two peaks: {iPIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
iPIDt_switch = np.pi/ (2*np.pi*(1/iPIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {iPIDt_switch:.4f} seconds")

i_normal_is = 80 * IS.delay(time, iPIDt_switch)
plt.plot(time, i_normal_is, label='Input Shaping')
i_robust_is = 80 * IS.robust_is(time, iPIDt_switch)
plt.plot(time, i_robust_is, label='Input Shaping')

# %%
# Suppose `state[:, 1]` is your omega (motor speed) and `time` is your time array
i_inverted_signal = -i_state[:, 1]  # Invert to detect minima
i_min_peak_indices, _ = find_peaks(i_inverted_signal, height=None)

# Get the times and values of those minima
i_min_peak_values = i_state[:, 1][i_min_peak_indices]
i_min_peak_times = time[i_min_peak_indices]

print("Min peak values:", i_min_peak_values)
print("Min peak times:", i_min_peak_times)

# Example: time difference between two first dips after 70s
i_mask = i_min_peak_times > 70
i_min_peaks_after_70 = i_min_peak_times[i_mask]
if len(i_min_peaks_after_70) >= 2:
    i_dt_tt = i_min_peaks_after_70[1] - i_min_peaks_after_70[0]
    print(f"Time between first two dips after 70s: {i_dt_tt:.4f} seconds")

iPID_Lswitch = np.pi/ (2*np.pi*(1/i_dt_tt) * np.sqrt(1-zeta**2))
print(f"t_switch for load: {iPID_Lswitch:.4f} seconds")

i_normal_Lis = TL * IS.delay(time, iPID_Lswitch + time[int(duration/dt/1.4)])
plt.plot(time, i_normal_Lis, label='Load Input Shaping')
i_pre = IS.robust_is(time, iPID_Lswitch)
i_robust_Lis = np.zeros_like(i_pre)
i_robust_Lis[int(duration/dt/1.4):] = TL[-1] * i_pre[:len(time) - int(duration/dt/1.4)]
plt.plot(time, i_robust_Lis, label='Load Input Shaping')



# %%
time, i_state_is = iPID_loop.simulate_extra(time, dt, i_normal_is, init = init, control_index = 1, extra_args_func = i_normal_Lis)
time, i_state_robust = iPID_loop.simulate_extra(time, dt, i_robust_is, init = init, control_index = 1, extra_args_func = i_robust_Lis)

# %%
plt.plot(time, i_state_is[:, 1], label="iPID normal IS (rad/s)")
plt.plot(time, i_state_robust[:, 1], label="iPID robust IS (rad/s)")
# plt.plot(time, ia_test)
plt.xlabel("Time (s)")
plt.ylabel("Motor Speed (rad/s)")
plt.title("DC Motor PID Control")
plt.legend()
plt.grid()
plt.show()

# %% Parameter variation
R_vary = [0.1, 0.15, 0.2, 0.25, 0.3]
PID_state_store = np.zeros((len(R_vary), len(time)))
iPID_state_store = np.zeros((len(R_vary), len(time)))

for i in range(len(R_vary)):
    R_i = R_vary[i]
    motor_vary = DC_motor(R_i, 
                 motor_params["L"][0], 
                 motor_params["Kb"][0], 
                 motor_params["Kt"][0], 
                 motor_params["J"][0],
                 motor_params["B"][0],
                 TL = TL)
    PID = PID_controller(Kp, Ki, Kd)
    PID_CL = Closed_loop(motor_vary, PID)
    t, PID_v = PID_CL.simulate_extra(time, dt, ref = omega_ref, init = init, control_index = 1, extra_args_func = TL)
    PID_state_store[i, :] = PID_v.T[1]

    iPID = iPID_controller(Kp, Ki, Kd)
    iPID_CL = Closed_loop(motor_vary, iPID)
    t, iPID_v = iPID_CL.simulate_extra(time, dt, ref = omega_ref, init = init, control_index = 1, extra_args_func = TL)

    iPID_state_store[i, :] = iPID_v.T[1]

# %% 
plt.figure(figsize=(8, 6))
for i in range(len(R_vary)):
    plt.plot(time, PID_state_store[i,:], label = r'$R =  $' + str(R_vary[i]))

# plt.plot(t, i_normal_is,'c--', label = r'$ x_{\mathrm{normal\ shaped\ ref}} $',linewidth=2)
# plt.plot(t, iPID_normal_is[:,0],'c-', label=r'$ x_{\mathrm{response\ to\ normal\ shaped}} $', linewidth=2)
# plt.plot(t, i_robust_is,'b--', label = r'$ x_{\mathrm{robust\ shaped\ ref}} $',linewidth=2)
# plt.plot(t, iPID_robusts_is[:,0],'b-', label=r'$ x_{\mathrm{response\ to\ robust\ shaped}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# plt.ylim(0, 1.6)
# plt.xlim(0, duration)

plt.figure(figsize=(8, 6))
for i in range(len(R_vary)):
    plt.plot(time, iPID_state_store[i,:], label = r'$R =  $' + str(R_vary[i]))
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %%


















