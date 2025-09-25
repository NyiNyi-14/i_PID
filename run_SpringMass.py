#%% Import Libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import pandas as pd

import os
print(os.getcwd())
os.chdir("...")
print(os.listdir())

# Local
from PID_controller import PID_controller
from iPID_controller import iPID_controller
from InputShaping import InputShaping
from Closed_loop import Closed_loop
from SpringMassSystem import SpringMassSystem
from PerformanceMetrics import PerformanceMetrics

# %% Simulation Parameters
m = 1.0  # mass (kg)
k = 1.0  # spring constant (N/m)

duration = 35
dt = 0.01
time = np.arange(0, duration, dt)
x0 = 0
v0 = 0
init = [x0, v0]
spring = SpringMassSystem(m, k)

step_ref = np.ones_like(time) 
step_ref[:int(1/dt)] = 0
exp_ref = 1 - np.exp(-0.5*time) 

# %% Ziegler–Nichols tuning, Fully known dynamics
Ku = 1.0
T = 4.4
Kp_zn = .6*Ku
Ki_zn = (2*Kp_zn)/T
Kd_zn = (Kp_zn*T)/8
PID_zn = PID_controller(Kp_zn, Ki_zn, Kd_zn)
iPID_zn = iPID_controller(Kp_zn, Ki_zn, Kd_zn)
PID_zn = Closed_loop(spring, PID_zn)
iPID_zn = Closed_loop(spring, iPID_zn)
t, zn_PID = PID_zn.simulate(time, dt, ref=step_ref, init = init, control_index = 0)
t, zn_iPID = iPID_zn.simulate(time, dt, ref=step_ref, init = init, control_index = 0)

plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, zn_PID[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, zn_iPID[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

# %% Partially known dynamics
Kp = 5
Ki = 3
Kd = 1
PID = PID_controller(Kp, Ki, Kd)
iPID = iPID_controller(Kp, Ki, Kd)
PID_loop = Closed_loop(spring, PID)
iPID_loop = Closed_loop(spring, iPID)

# Exp response
t, PID_exp = PID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 0)
t, iPID_exp = iPID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 0)
plt.figure(figsize=(8, 6))
plt.plot(t, exp_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, PID_exp[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_exp[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

# Step response
t, PID_step = PID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 0)
t, iPID_step = iPID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 0)
plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, PID_step[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_step[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

# %% Input shaper design for PID loop
zeta = 0.0866 # look at MATLAB Root Locus
omega_n = 2.31 # look at MATLAB Root Locus
IS = InputShaping(zeta, omega_n)

PIDpeak_indices, _ = find_peaks(PID_step[:, 0])
PIDpeak_values = PID_step[:,0][PIDpeak_indices]
PIDpeak_times = time[PIDpeak_indices]
print(f"Peak values of PID step: {PIDpeak_values}")
 
if len(PIDpeak_times) >= 2:
    PIDt_delta = PIDpeak_times[1] - PIDpeak_times[0]
    print(f"Time between first two peaks: {PIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
PIDt_switch = np.pi/ (2*np.pi*(1/PIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {PIDt_switch:.4f} seconds")

normal_is = IS.delay(time, PIDt_switch)
robust_is = IS.robust_is(time, PIDt_switch)

t, PID_normal_is = PID_loop.simulate(time, dt, ref=normal_is, init = init, control_index = 0)
t, PID_robust_is = PID_loop.simulate(time, dt, ref=robust_is, init = init, control_index = 0)

plt.figure(figsize=(8, 6))
plt.plot(t, normal_is,'c--', label = r'$ x_{\mathrm{normal\ shaped}}^* $',linewidth=2)
plt.plot(t, PID_normal_is[:,0],'c-', label=r'$ x_{\mathrm{response\ to\ normal\ shaped}} $', linewidth=2)
plt.plot(t, robust_is,'b--', label = r'$ x_{\mathrm{robust\ shaped}}^* $',linewidth=2)
plt.plot(t, PID_robust_is[:,0],'b-', label=r'$ x_{\mathrm{response\ to\ robust\ shaped}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

# %% Input shaper design for iPID loop
i_zeta = 0.384 # look at MATLAB Root Locus
i_omega_n = 3.08 # look at MATLAB Root Locus
i_IS = InputShaping(i_zeta, i_omega_n)

iPIDpeak_indices, _ = find_peaks(iPID_step[:, 0])
iPIDpeak_values = iPID_step[:,0][iPIDpeak_indices]
iPIDpeak_times = time[iPIDpeak_indices]
print(f"Peak values of PID step: {iPIDpeak_values}")
 
if len(iPIDpeak_times) >= 2:
    iPIDt_delta = iPIDpeak_times[1] - iPIDpeak_times[0]
    print(f"Time between first two peaks: {iPIDt_delta:.4f} seconds")
else:
    print("Less than two peaks found.")
 
iPIDt_switch = np.pi/ (2*np.pi*(1/iPIDt_delta) * np.sqrt(1-zeta**2))
print(f"t_switch: {iPIDt_switch:.4f} seconds")

i_normal_is = i_IS.delay(time, iPIDt_switch)
plt.plot(time, i_normal_is, label='Input Shaping')
i_robust_is = i_IS.robust_is(time, iPIDt_switch)
plt.plot(time, i_robust_is, label='Input Shaping')

t, iPID_normal_is = iPID_loop.simulate(time, dt, ref=i_normal_is, init = init, control_index = 0)
t, iPID_robusts_is = iPID_loop.simulate(time, dt, ref=i_robust_is, init = init, control_index = 0)

plt.figure(figsize=(8, 6))
plt.plot(t, i_normal_is,'c--', label = r'$ x_{\mathrm{normal\ shaped}}^* $',linewidth=2)
plt.plot(t, iPID_normal_is[:,0],'c-', label=r'$ x_{\mathrm{response\ to\ normal\ shaped}} $', linewidth=2)
plt.plot(t, i_robust_is,'b--', label = r'$ x_{\mathrm{robust\ shaped}}^* $',linewidth=2)
plt.plot(t, iPID_robusts_is[:,0],'b-', label=r'$ x_{\mathrm{response\ to\ robust\ shaped}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

# %% Performance Evaluation
PIDstep_evaluation = PerformanceMetrics(time, step_ref)
PIDstep_tr, PIDstep_ts, PIDstep_o_s, PIDstep_sse, PIDstep_ae, PIDstep_re = PIDstep_evaluation.step_response(PID_step[:, 0])

PIDis_evaluation = PerformanceMetrics(time, normal_is)
PIDis_tr, PIDis_ts, PIDis_o_s, PIDis_sse, PIDis_ae, PIDis_re = PIDis_evaluation.step_response(PID_normal_is[:, 0])

PIDrobustis_evaluation = PerformanceMetrics(time, robust_is)
PIDrobustis_tr, PIDrobustis_ts, PIDrobustis_o_s, PIDrobustis_sse, PIDrobustis_ae, PIDrobustis_re = PIDrobustis_evaluation.step_response(PID_robust_is[:, 0])

iPIDstep_evaluation = PerformanceMetrics(time, step_ref)
iPIDstep_tr, iPIDstep_ts, iPIDstep_o_s, iPIDstep_sse, iPIDstep_ae, iPIDstep_re = iPIDstep_evaluation.step_response(iPID_step[:, 0])

iPIDis_evaluation = PerformanceMetrics(time, i_normal_is)
iPIDis_tr, iPIDis_ts, iPIDis_o_s, iPIDis_sse, iPIDis_ae, iPIDis_re = iPIDis_evaluation.step_response(iPID_normal_is[:, 0])

iPIDrobustis_evaluation = PerformanceMetrics(time, i_robust_is)
iPIDrobustis_tr, iPIDrobustis_ts, iPIDrobustis_o_s, iPIDrobustis_sse, iPIDrobustis_ae, iPIDrobustis_re = iPIDrobustis_evaluation.step_response(iPID_robusts_is[:, 0])

table = {
    "Performance Evaluation": ["PID - Step", "PID - IS", "PID - Robust IS", "iPID - Step", "iPID - IS", "iPID - Robust IS"],
    "Rise Time (s)": [PIDstep_tr, PIDis_tr, PIDrobustis_tr, iPIDstep_tr, iPIDis_tr, iPIDrobustis_tr],
    "Settling Time (s)": [PIDstep_ts, PIDis_ts, PIDrobustis_ts, iPIDstep_ts, iPIDis_ts, iPIDrobustis_ts],
    "Overshoot (%)": [PIDstep_o_s, PIDis_o_s, PIDrobustis_o_s, iPIDstep_o_s, iPIDis_o_s, iPIDrobustis_o_s],    
    "Total Absolute Error": [PIDstep_ae, PIDis_ae, PIDrobustis_ae, iPIDstep_ae, iPIDis_ae, iPIDrobustis_ae],
}

data_frame = pd.DataFrame(table)

# %% Parameter variation
k_vary = [0.8, 0.9, 1.0, 1.1, 1.2]
PID_state_store = np.zeros((len(k_vary), len(time)))
iPID_state_store = np.zeros((len(k_vary), len(time)))

for i in range(len(k_vary)):
    k_i = k_vary[i]
    SM_sys = SpringMassSystem(m = m,
                              k = k_i)
    PID = PID_controller(Kp, Ki, Kd)
    PID_CL = Closed_loop(SM_sys, PID)
    t, states = PID_CL.simulate(time, dt, ref = step_ref, init = [x0, v0], control_index=0)

    iPID = iPID_controller(Kp, Ki, Kd)
    iPID_CL = Closed_loop(SM_sys, iPID)
    t, i_states = iPID_CL.simulate(time, dt, ref = step_ref, init = [x0, v0], control_index=0)
    PID_state_store[i, :] = states.T[0]
    iPID_state_store[i, :] = i_states.T[0]
 
plt.figure(figsize=(8, 6))
for i in range(len(k_vary)):
    plt.plot(time, PID_state_store[i,:], label = 'k =  ' + str(k_vary[i]))

plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()

plt.figure(figsize=(8, 6))
for i in range(len(k_vary)):
    plt.plot(time, iPID_state_store[i,:], label = 'k =  ' + str(k_vary[i]))

# %%