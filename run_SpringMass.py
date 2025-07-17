#%% Import Libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks
import pandas as pd

import os
print(os.getcwd())
os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID/Libraries_NNA")
print(os.listdir())

from PID_controller import PID_controller
from iPID_controller import iPID_controller
from InputShaping import InputShaping
from Closed_loop import Closed_loop
from SpringMassSystem import SpringMassSystem
from PerformanceMetrics import PerformanceMetrics
from sampling import sampling
import seaborn as sns

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

# %% Ziegler–Nichols tuning
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
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_zn.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %% Random tuning
Kp = 5
Ki = 3
Kd = 1
PID = PID_controller(Kp, Ki, Kd)
iPID = iPID_controller(Kp, Ki, Kd)
PID_loop = Closed_loop(spring, PID)
iPID_loop = Closed_loop(spring, iPID)

t, PID_exp = PID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 0)
t, iPID_exp = iPID_loop.simulate(time, dt, ref=exp_ref, init = init, control_index = 0)
plt.figure(figsize=(8, 6))
plt.plot(t, exp_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, PID_exp[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_exp[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_exp.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

t, PID_step = PID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 0)
t, iPID_step = iPID_loop.simulate(time, dt, ref=step_ref, init = init, control_index = 0)
plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, PID_step[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, iPID_step[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
# plt.ylim(0, step_ref[-1]*1.3)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()
plt.tight_layout()
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_step.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %% Input shaper design
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
# t_switch = np.pi/(omega_n * np.sqrt(1-zeta**2))
# print(f"t_switch: {t_switch:.4f} seconds")

normal_is = IS.delay(time, PIDt_switch)
# plt.plot(time, normal_is, label='Input Shaping')
robust_is = IS.robust_is(time, PIDt_switch)
# plt.plot(time, robust_is, label='Input Shaping')

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
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_PID_shaped.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

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
# iPIDt_switch = np.pi/(omega_n * np.sqrt(1-zeta**2))
# print(f"t_switch: {iPIDt_switch:.4f} seconds")

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
# plt.savefig('/Users/nyinyia/Documents/13_Paper_mine/02_ACC_iPID/figures/sm_iPID_shaped.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

# %%
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
    # "Steady-State Error": [PIDstep_sse, PIDis_sse, PIDrobustis_sse, iPIDstep_sse, iPIDis_sse, iPIDrobustis_sse],
    "Total Absolute Error": [PIDstep_ae, PIDis_ae, PIDrobustis_ae, iPIDstep_ae, iPIDis_ae, iPIDrobustis_ae],
    # "Total Relative Error": [PIDstep_re, PIDis_re, PIDrobustis_re, iPIDstep_re, iPIDis_re, iPIDrobustis_re],
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

plt.figure(figsize=(8, 6))
for i in range(len(k_vary)):
    plt.plot(time, iPID_state_store[i,:], label = 'k =  ' + str(k_vary[i]))

#%% Running the simulation
simulation = Closed_loop(spring, PID)
graph_time = 35
evaluation_time = np.arange(0, graph_time, dt)  

#%%
t, step_states = simulation.simulate(time, dt, ref=step_ref, init = [x0, v0], control_index=0)

plt.figure(figsize=(8, 6))
plt.plot(t, step_ref,'b-', label = r'$ x_{ref} $',linewidth=2)
plt.plot(t, step_states[:,0],'c-', label=r'$ x $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 14)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 14)
plt.xlim(0, graph_time)
plt.legend(fontsize = 12)
plt.tick_params(axis='both', labelsize=14) 
plt.grid()
plt.tight_layout()
# plt.savefig(r'\Brad\VS Code\Figures\Spring Mass\PID_Spring_step_test.pdf', format='pdf', bbox_inches='tight')# %% Plotting exponential reference signal and position

#%%
t, exp_states = simulation.simulate(time, dt, ref=exp_ref, init = [x0, v0], control_index=0)

plt.figure(figsize=(8, 6))
plt.plot(t, exp_ref,'b-', label = r'$ x_{ref} $',linewidth=2)
plt.plot(t, exp_states[:,0],'c-', label=r'$ x $',linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 14)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 14)
plt.xlim(0, graph_time)

plt.legend(fontsize = 12)
plt.tick_params(axis='both', labelsize=14) 
plt.grid()
plt.tight_layout()
# plt.savefig(r'\Brad\VS Code\Figures\Spring Mass\PID_Spring_exp_test.pdf', format='pdf', bbox_inches='tight')


# %%
# t, is_states = simulation.simulate(time, dt, ref=is_ref_ro, init = [x0, v0], control_index=0)
t, is_states = simulation.simulate(time, dt, ref=is_ref, init = [x0, v0], control_index=0)

# %% 
plt.figure(figsize=(8, 6))
plt.plot(t, is_ref,'b-', label = r'$ x_{ref} $',linewidth=2)
plt.plot(t, is_states[:,0],'c-', label=r'$ x $',linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 14)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 14)
plt.xlim(0, graph_time)
# plt.ylim(0, is_ref[-1] +.10)
plt.legend(fontsize = 14)
plt.tick_params(axis='both', labelsize=12) 
plt.grid()
plt.tight_layout()
plt.savefig(r'\Brad\VS Code\Figures\Spring Mass\PID_Spring_is_test.pdf', format='pdf', bbox_inches='tight')


# %% Sampling
spring_para = {
    "m": [1],
    "k": [1],
}

stiffness = [0.8, 0.9, 1.0, 1.1, 1.2]
state_stored = []
theta_store = np.zeros((len(stiffness), len(time)))

select_ref = step_ref

for i in range(len(stiffness)):
    k_i = stiffness[i]
    pen_LHS = SpringMassSystem(m = spring_para["m"][0],
                              k = k_i)
    PID = PID_controller(Kp, Ki, Kd)
    CL_MC = Closed_loop(pen_LHS, PID)
    time, all_states = CL_MC.simulate(time, dt, select_ref, init = [x0, v0], control_index=0)
    theta_store[i, :] = all_states.T[0] 
 
# %%
for i in range(len(stiffness)):
    plt.plot(time, theta_store[i,:], label = 'k =  ' + str(stiffness[i]))
plt.ylim(0, 1.6)
plt.xlim(0, duration)
 
# Force y-ticks to include 180
yticks = np.arange(0, 1.8, .2)  # e.g., ticks every 20 units
if 1 not in yticks:
    yticks = np.append(yticks, 1)
    yticks = np.sort(yticks)
plt.yticks(yticks)
 
# Optionally, add a horizontal reference line for visual clarity
plt.plot(time, select_ref, label = r"$\theta_{ref}$", color = 'blue', linestyle='--', linewidth=1)

plt.legend() 
plt.show()
 
# %%
theta_store_tran = theta_store.T
tr_stored = np.zeros(len(stiffness))
ts_stored = np.zeros(len(stiffness))
o_s_stored = np.zeros(len(stiffness))
sse_stored = np.zeros(len(stiffness))
ae_stored = np.zeros(len(stiffness))
re_stored = np.zeros(len(stiffness))

evaluation = PerformanceMetrics(time, select_ref)
for i in range(len(stiffness)):
    tr, ts, o_s, sse, ae, re = evaluation.step_response(theta_store_tran[:, i])
    tr_stored[i] = tr
    ts_stored[i] = ts
    o_s_stored[i] = o_s
    sse_stored[i] = sse
    ae_stored[i] = ae
    re_stored[i] = re
    
table = {
    "Performance Evaluation": [" k =  " + str(stiffness[0]), " k =  " + str(stiffness[1]), " k =  " + str(stiffness[2]), " k =  " + str(stiffness[3]), " k =  " + str(stiffness[4])],
    "Rise Time (s)": [tr_stored[0], tr_stored[1], tr_stored[2], tr_stored[3], tr_stored[4]],
    "Settling Time (s)": [ts_stored[0], ts_stored[1], ts_stored[2], ts_stored[3], ts_stored[4]],
    "Overshoot (%)": [o_s_stored[0], o_s_stored[1], o_s_stored[2], o_s_stored[3], o_s_stored[4]],
    "Steady-State Error": [sse_stored[0], sse_stored[1], sse_stored[2], sse_stored[3], sse_stored[4]],
    "Total Absolute Error": [ae_stored[0], ae_stored[1], ae_stored[2], ae_stored[3], ae_stored[4]],
    "Total Relative Error": [re_stored[0], re_stored[1], re_stored[2], re_stored[3], re_stored[4]],
}
data_frame = pd.DataFrame(table)

# %%