#%% Import Libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks
import pandas as pd
import os

import os
print(os.getcwd())
os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID/Libraries_NNA")
print(os.listdir())

from PID_controller import PID_controller
from iPID_controller import iPID_controller
from Closed_loop import Closed_loop
from InputShaping import InputShaping
from LinearizedPendulum import LinearizedPendulum
from PerformanceMetrics import PerformanceMetrics

# %% Simulation Setup
# Characteristics of the system
M = 1
m = 0.1
l = 0.5
g = 9.81

duration = 15
dt = 0.01
time = np.arange(0, duration, dt)
x0 = 0
dx0 = 0
theta0 = 175
dtheta0 = 0
init = [x0, dx0, theta0, dtheta0]
Pendulum = LinearizedPendulum(M, m, l, g)

step_ref = 180 * np.ones_like(time) # Stepping Reference signal
# step_ref[:int(1/dt)] = 0
exp_ref = 180 * (1 - np.exp(-0.5*time)) # Exponential Reference signal

# %% Ziegler–Nichols tuning
# Ku = 10
# T = 1.256
Ku = 50
T = 0.75
Kp_zn = 0.6 * Ku
Ki_zn = (2 * Kp_zn)/ T
Kd_zn = (Kp_zn * T)/ 8

PID_zn = PID_controller(Kp_zn, Ki_zn, Kd_zn)
iPID_zn = iPID_controller(Kp_zn, Ki_zn, Kd_zn)
PID_zn = Closed_loop(Pendulum, PID_zn)
iPID_zn = Closed_loop(Pendulum, iPID_zn)
t, zn_PID = PID_zn.simulate(time, dt, ref=step_ref, init = init, control_index = 2)
t, zn_iPID = iPID_zn.simulate(time, dt, ref=step_ref, init = init, control_index = 2)

plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'r', label = r'$ \theta_{\mathrm{ref}} $',linewidth=2)
plt.plot(t, zn_PID[:,2],'c-', label=r'$ \theta_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, zn_iPID[:,2],'b-', label=r'$ \theta_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.1)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_zn.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %% Random tuning
Kp = 50
Ki = 60
Kd = 2

PID = PID_controller(Kp, Ki, Kd)
iPID = iPID_controller(Kp, Ki, Kd)
PID_loop = Closed_loop(Pendulum, PID)
iPID_loop = Closed_loop(Pendulum, iPID)

t, PID_exp = PID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 2)
t, iPID_exp = iPID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 2)
plt.figure(figsize=(8, 6))
plt.plot(t, exp_ref,'r', label = r'$ \theta_{\mathrm{ref}} $',linewidth=2)
plt.plot(t, PID_exp[:,2],'c-', label=r'$ \theta_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_exp[:,2],'b-', label=r'$ \theta_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_exp.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

t, PID_step = PID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 2)
t, iPID_step = iPID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 2)
plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'r', label = r'$ \theta_{\mathrm{ref}} $',linewidth=2)
plt.plot(t, PID_step[:,2],'c-', label=r'$ \theta_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_step[:,2],'b-', label=r'$ \theta_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_step.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %% Input shaper design
zeta = 0.0383 # look at MATLAB Root Locus
omega_n = 6.11 # look at MATLAB Root Locus
# zeta = 0.0
IS = InputShaping(zeta, omega_n)

PIDpeak_indices, _ = find_peaks(PID_step[:, 2], height = 180)
PIDpeak_values = PID_step[:,2][PIDpeak_indices]
PIDpeak_times = time[PIDpeak_indices]
print(f"Peak values of PID step: {PIDpeak_values}")
 
if len(PIDpeak_times) >= 2:
    PIDt_delta = PIDpeak_times[1] - PIDpeak_times[0]
    print(f"Time between first two peaks: {PIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
PIDt_switch = np.pi/ (2*np.pi*(1/PIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {PIDt_switch:.4f} seconds")
# PIDt_switch = np.pi/(omega_n * np.sqrt(1-zeta**2))
# print(f"t_switch: {PIDt_switch:.4f} seconds")

normal_is = 180 * IS.delay(time, PIDt_switch)
plt.plot(time, normal_is, label='Input Shaping')
robust_is = 180 * IS.robust_is(time, PIDt_switch)
plt.plot(time, robust_is, label='Input Shaping')
# %%
t, PID_normal_is = PID_loop.simulate(time, dt, ref=normal_is, init = init, control_index = 2)
t, PID_robust_is = PID_loop.simulate(time, dt, ref=robust_is, init = init, control_index = 2)

plt.figure(figsize=(8, 6))
plt.plot(t, normal_is,'c--', label = r'$ \theta_{\mathrm{normal\ shaped\ ref}} $',linewidth=2)
plt.plot(t, PID_normal_is[:,2],'c-', label=r'$ \theta_{\mathrm{response\ to\ normal\ shaped}} $', linewidth=2)
plt.plot(t, robust_is,'b--', label = r'$ \theta_{\mathrm{robust\ shaped\ ref}} $',linewidth=2)
plt.plot(t, PID_robust_is[:,2],'b-', label=r'$ \theta_{\mathrm{response\ to\ robust\ shaped}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_PID_shaped.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %% Input shaper design for iPID loop
i_zeta = 0.332 # look at MATLAB Root Locus
i_omega_n = 12.9 # look at MATLAB Root Locus
i_IS = InputShaping(i_zeta, i_omega_n)

iPIDpeak_indices, _ = find_peaks(iPID_step[:, 2], height = 180)
iPIDpeak_values = iPID_step[:,2][iPIDpeak_indices]
iPIDpeak_times = time[iPIDpeak_indices]
print(f"Peak values of PID step: {iPIDpeak_values}")
 
if len(iPIDpeak_times) >= 2:
    iPIDt_delta = iPIDpeak_times[1] - iPIDpeak_times[0]
    print(f"Time between first two peaks: {iPIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
iPIDt_switch = np.pi/ (2*np.pi*(1/iPIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {iPIDt_switch:.4f} seconds")
# iPIDt_switch = np.pi/(omega_n * np.sqrt(1-zeta**2))
# print(f"t_switch: {iPIDt_switch:.4f} seconds")

i_normal_is = 180 * i_IS.delay(time, iPIDt_switch)
plt.plot(time, i_normal_is, label='Input Shaping')
i_robust_is = 180 * i_IS.robust_is(time, iPIDt_switch)
plt.plot(time, i_robust_is, label='Input Shaping')

t, iPID_normal_is = iPID_loop.simulate(time, dt, ref=i_normal_is, init = init, control_index = 2)
t, iPID_robusts_is = iPID_loop.simulate(time, dt, ref=i_robust_is, init = init, control_index = 2)

plt.figure(figsize=(8, 6))
plt.plot(t, i_normal_is,'c--', label = r'$ \theta_{\mathrm{normal\ shaped\ ref}} $',linewidth=2)
plt.plot(t, iPID_normal_is[:,2],'c-', label=r'$ \theta_{\mathrm{response\ to\ normal\ shaped}} $', linewidth=2)
plt.plot(t, i_robust_is,'b--', label = r'$ \theta_{\mathrm{robust\ shaped\ ref}} $',linewidth=2)
plt.plot(t, iPID_robusts_is[:,2],'b-', label=r'$ \theta_{\mathrm{response\ to\ robust\ shaped}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_iPID_shaped.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %%
PIDstep_evaluation = PerformanceMetrics(time, step_ref)
PIDstep_tr, PIDstep_ts, PIDstep_o_s, PIDstep_sse, PIDstep_ae, PIDstep_re = PIDstep_evaluation.step_response(PID_step[:, 2])

PIDis_evaluation = PerformanceMetrics(time, normal_is)
PIDis_tr, PIDis_ts, PIDis_o_s, PIDis_sse, PIDis_ae, PIDis_re = PIDis_evaluation.step_response(PID_normal_is[:, 2])

PIDrobustis_evaluation = PerformanceMetrics(time, robust_is)
PIDrobustis_tr, PIDrobustis_ts, PIDrobustis_o_s, PIDrobustis_sse, PIDrobustis_ae, PIDrobustis_re = PIDrobustis_evaluation.step_response(PID_robust_is[:, 2])

iPIDstep_evaluation = PerformanceMetrics(time, step_ref)
iPIDstep_tr, iPIDstep_ts, iPIDstep_o_s, iPIDstep_sse, iPIDstep_ae, iPIDstep_re = iPIDstep_evaluation.step_response(iPID_step[:, 2])

iPIDis_evaluation = PerformanceMetrics(time, i_normal_is)
iPIDis_tr, iPIDis_ts, iPIDis_o_s, iPIDis_sse, iPIDis_ae, iPIDis_re = iPIDis_evaluation.step_response(iPID_normal_is[:, 2])

iPIDrobustis_evaluation = PerformanceMetrics(time, i_robust_is)
iPIDrobustis_tr, iPIDrobustis_ts, iPIDrobustis_o_s, iPIDrobustis_sse, iPIDrobustis_ae, iPIDrobustis_re = iPIDrobustis_evaluation.step_response(iPID_robusts_is[:, 2])

table = {
    "Performance Evaluation": ["PID - Step", "PID - IS", "PID - Robust IS", "iPID - Step", "iPID - IS", "iPID - Robust IS"],
    "Rise Time (s)": [PIDstep_tr, PIDis_tr, PIDrobustis_tr, iPIDstep_tr, iPIDis_tr, iPIDrobustis_tr],
    "Settling Time (s)": [PIDstep_ts, PIDis_ts, PIDrobustis_ts, iPIDstep_ts, iPIDis_ts, iPIDrobustis_ts],
    "Overshoot (%)": [PIDstep_o_s, PIDis_o_s, PIDrobustis_o_s, iPIDstep_o_s, iPIDis_o_s, iPIDrobustis_o_s],    
    # "Steady-State Error": [PIDstep_sse, PIDis_sse, PIDrobustis_sse, iPIDstep_sse, iPIDis_sse, iPIDrobustis_sse],
    "Total Absolute Error": [PIDstep_ae, PIDis_ae, PIDrobustis_ae, iPIDstep_ae, iPIDis_ae, iPIDrobustis_ae],
    # "Total Relative Error": [PIDstep_re, PIDis_re, PIDrobustis_re, iPIDstep_re, iPIDis_re, iPIDrobustis_re],
}

data_frame = pd.DataFrame(table)


# %% Parameter variation
l_vary = [0.3, 0.4, 0.5, 0.6, 0.7]
PID_state_store = np.zeros((len(l_vary), len(time)))
PID_is_store = np.zeros((len(l_vary), len(time)))
PID_robust_store = np.zeros((len(l_vary), len(time)))

iPID_state_store = np.zeros((len(l_vary), len(time)))
iPID_is_store = np.zeros((len(l_vary), len(time)))
iPID_robust_store = np.zeros((len(l_vary), len(time)))

Pendulum = LinearizedPendulum(M, m, l, g)
for i in range(len(l_vary)):
    l_i = l_vary[i]
    Pen_sys = LinearizedPendulum(M = M,
                                m = m,
                                l = l_i,
                                g = g)
    PID = PID_controller(Kp, Ki, Kd)
    PID_CL = Closed_loop(Pen_sys, PID)
    t, states = PID_CL.simulate(time, dt, ref = step_ref, init = init, control_index=2)
    t, states_is = PID_CL.simulate(time, dt, ref = normal_is, init = init, control_index=2)
    t, states_robust = PID_CL.simulate(time, dt, ref = robust_is, init = init, control_index=2)
    PID_state_store[i, :] = states.T[2]
    PID_is_store[i, :] = states_is.T[2]
    PID_robust_store[i, :] = states_robust.T[2]

    iPID = iPID_controller(Kp, Ki, Kd)
    iPID_CL = Closed_loop(Pen_sys, iPID)
    t, i_states = iPID_CL.simulate(time, dt, ref = step_ref, init = init, control_index=2)
    t, i_states_is = iPID_CL.simulate(time, dt, ref = i_normal_is, init = init, control_index=2)
    t, i_states_robust = iPID_CL.simulate(time, dt, ref = i_robust_is, init = init, control_index=2)
    iPID_state_store[i, :] = i_states.T[2]
    iPID_is_store[i, :] = i_states_is.T[2]
    iPID_robust_store[i, :] = i_states_robust.T[2]

# %%
plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, PID_state_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, PID_is_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, PID_robust_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %%
plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, iPID_state_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, iPID_is_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

plt.figure(figsize=(8, 6))
for i in range(len(l_vary)):
    plt.plot(time, iPID_robust_store[i,:], label = r'$m =  $' + str(l_vary[i]), linewidth=2)

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ \theta \ \ \mathrm{[\degree]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/yy.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position











# %%
theta = zn_PID[:, 2]
import numpy as np
import matplotlib.pyplot as plt

# Example: theta = pendulum angle over time
# Assume you already have these from simulation:
# t = np.linspace(0, 30, 3000)
# theta = your simulation result

# FFT calculation
dt = t[1] - t[0]                      # Sampling interval
Fs = 1 / dt                           # Sampling frequency
N = len(theta)                        # Number of samples

Y = np.fft.fft(theta)
freq = np.fft.fftfreq(N, d=dt)

# Only take the positive half of the spectrum
positive_freqs = freq[:N//2]
Y_magnitude = np.abs(Y[:N//2]) / N

# Plot
plt.figure(figsize=(10, 5))
plt.plot(positive_freqs, Y_magnitude)
plt.title('FFT of Pendulum Angle $\\theta(t)$')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude')
plt.grid(True)
plt.tight_layout()
plt.show()

# %%