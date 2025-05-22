
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MFC based on Fliess 2008 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

% choice_user = 1; % 1: PID and 2: iPI

s = tf('s');

TF = (s+2)^2/(s+1.5)^3;

K = 4;
T = 2.018;
tau = 0.2424;

Kp = 100*(0.4*tau+T)/(120*K*tau);
Ki = 1/(1.33*K*tau);
Kd = 0.35*T/K;

% extracting information out of transfer function
[num, den] = tfdata(TF);
[A,B,C,D] = tf2ss(num{1}, den{1});

t_vec = linspace(0,30,101);
r_vec = 1 - exp(-t_vec);
rd_vec = exp(-t_vec);


% normal PID

% iPID
x0_vec = [0; 0; 0; 0]; % initial condition
[TT_PID, YY_PID] = ode45(@(t,x) fct_ode(t,x,A,B,C,D,Kp,Ki,Kd,r_vec,rd_vec,t_vec,1), t_vec, x0_vec);
[TT_iPI, YY_iPI] = ode45(@(t,x) fct_ode(t,x,A,B,C,D,Kp,Ki,Kd,r_vec,rd_vec,t_vec,2), t_vec, x0_vec);


%%%%%%%%%%%%%%%%
%%% Plotting %%%
%%%%%%%%%%%%%%%%
figure(1); hold on;
plot(TT_PID, YY_PID(:,3), 'b');
plot(TT_iPI, YY_iPI(:,3), 'g');
plot(t_vec, r_vec, 'r--');
legend('PID','iPI','reference')

figure(2); hold on;
plot(TT_PID, abs(YY_PID(:,3)-r_vec'), 'b');
plot(TT_PID, abs(YY_iPI(:,3)-r_vec'), 'g');
legend('PID','iPI')

function dxdt = fct_ode(t,x,A,B,C,D,Kp,Ki,Kd,r_vec,rd_vec,t_vec,choice_user)
r = interp1(t_vec,r_vec,t);
r_d = interp1(t_vec,rd_vec,t);

e = r - x(3);                      % position error

if choice_user == 1
    u = Kp*e + Ki*x(4) + Kd*(0-x(2));  % PID control law
elseif choice_user == 2
    y_d_est = x(2);
    % u = Kp*e + Ki*x(4) + Kd*(0-x(2));  % PID control law
    u = r;
    F_e = x(1)  - u;
    PI_e = Kp*e + Ki*x(4);
    u = -F_e + r_d + PI_e % iPI control law
else

end


dxdt(1:3,1) = A*x(1:3) + B*u;
dxdt(4,1) = e;
end