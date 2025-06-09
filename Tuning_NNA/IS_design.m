
clear all; close all; clc;

% Spring Mass
s = tf('s');
m = 1;
k = 1;
P_sm = 1/ (m*s^2 + k);

% Controller, PID
Kp = 5;
Ki = 3;
Kd = 1; 

C_pid = Kp + Ki/s + Kd * s;
G_pid = C_pid * P_sm; 

% sisotool(G_pid/(1 + G_pid))

% figure;
% rlocus(G_pid);
% grid on;

%% 
% Controller, iPID
% C_ipid = Kp + Ki/s + Kd*s;
f_gain = 1;

Closed_ipid = (2*C_pid + s)/ ((1/P_sm) + s + 2*f_gain*C_pid);

sisotool(Closed_ipid)

% figure;
% rlocus(Closed_ipid);
% grid on;

%% Inverted pendulum
s = tf('s');
M = 1;
m = 0.1;
l = 0.5;
g = 9.81;
% P_pen = (m*l*s^2)/ ((M+m)*(l*s^4 + g*s^2));
P_pen = s^2 / (l*M*s^4 - (M + m)*g*s^2);

% Controller, PID
Kp_pen = 50;
Ki_pen = 60;
Kd_pen = 2;
C_pen = Kp_pen + Ki_pen/s + Kd_pen * s;

G_pen = C_pen * P_pen; 

figure;
rlocus(G_pen);
grid on;

%% 
% Controller, iPID

f_gain = 1;

Pen_ipid = (2*C_pen + s)/ ((1/P_pen) + s + 2*f_gain*C_pen);

figure;
rlocus(Pen_ipid);
grid on;


%% Spring mass, test
s = tf('s');
m = 1;
k = 1;
P_sm = 1/ (m*s^2 + k);

Kp = 5;
Ki = 3;
Kd = 1;

C_pid = Kp + Ki/s + Kd*s;
PID_open = P_sm * C_pid;
% step(P_sm);
rlocus(PID_open);

%% 

% Define the equation as a function
eqn = @(x) (0.2 ./ x) .* sqrt(1 - x.^2) - 2.31;

% Initial guess (try x ≈ 0.1 since sqrt(1-x²) requires |x| ≤ 1)
x0 = 0.1; 

% Solve numerically
x_solution = fsolve(eqn, x0);

% Display the solution
fprintf('Numerical solution: x = %.4f\n', x_solution);

% Verify the solution
left_side = (0.2 / x_solution) * sqrt(1 - x_solution^2);
fprintf('Verification: %.4f = 1.4062\n', left_side);






















