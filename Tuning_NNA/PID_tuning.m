%% DC motor modelling, finding rlocus and gain
% Parameters
R_nom  = 0.2;      % Ohm 
L_nom  = 0.5;      % H 
Kb_nom = 1.0;      % V·s/rad 
Kt_nom = 1.0;      % Nm/A 
J_nom  = 2.0;      % kg·m^2
B_nom  = 0.05;     % Nm·s/rad 

%% 
s = tf("s");
G = Kt_nom/ ((Kb_nom*Kt_nom) + (R_nom + s*L_nom)*(s*J_nom + B_nom));

% sisotool(G)
% controlSystemDesigner('rlocus', G);

figure (1);
rlocus(G);
grid on;

figure (2);
step(G);
grid on;

C = pidtune(G, "PID"); % alternate tuning

%% Spring mass
s = tf('s');
m = 1;
k = 1;
P_sm = 1/ (m*s^2 + k);

figure (3);
rlocus(P_sm);
grid on;

figure (4);
step(P_sm);
grid on;

C_sm = pidtune(P_sm, "PID"); % alternate tuning

% ZN tuning
Ku = 1.0;
T = 4.4;
Kp_zn = 0.6*Ku;
Ki_zn = (2*Kp_zn)/T;
Kd_zn = (Kp_zn*T)/8;

%% 





