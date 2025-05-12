
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MFC based on Fliess 2008 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

% choice_user = 1; % 1: PID and 2: iPI

s = tf('s');

TF = 1/(s^2+1);

Kp = 200;
Ki = 100;
Kd = 100;

t_vec = linspace(0,10,1001);
% r_vec = 1 - exp(-5*t_vec);
% rd_vec = exp(-5*t_vec);

r_vec = ones(1,1001);
rd_vec = zeros(1,1001);


% normal PID

% iPID
x0_vec = [0; 0; 0]; % initial condition
[TT_PID, YY_PID] = ode45(@(t,x) fct_ode(t,x,Kp,Ki,Kd,r_vec,rd_vec,t_vec,1), t_vec, x0_vec);
[TT_iPI, YY_iPI] = ode45(@(t,x) fct_ode(t,x,Kp,Ki,Kd,r_vec,rd_vec,t_vec,2), t_vec, x0_vec);


%%%%%%%%%%%%%%%%
%%% Plotting %%%
%%%%%%%%%%%%%%%%
figure(1); hold on;
plot(TT_PID, YY_PID(:,1), 'b');
plot(TT_iPI, YY_iPI(:,1), 'g');
plot(t_vec, r_vec, 'r--');
legend('PID','iPI','reference')

figure(2); hold on;
plot(TT_PID, abs(YY_PID(:,1)-r_vec'), 'b');
plot(TT_PID, abs(YY_iPI(:,1)-r_vec'), 'g');
legend('PID','iPI')

sum(abs(YY_PID(:,1)-r_vec'))
sum(abs(YY_iPI(:,1)-r_vec'))

function dxdt = fct_ode(t,x,Kp,Ki,Kd,r_vec,rd_vec,t_vec,choice_user)
r = interp1(t_vec,r_vec,t);
r_d = interp1(t_vec,rd_vec,t);

% r = (0.5 + 0.5*heaviside(t-0.22)) * r;

e = r - x(1);                      % position error

if choice_user == 1
    u = Kp*e + Ki*x(3) + Kd*(0-x(2));  % PID control law
elseif choice_user == 2
    y_d_est = x(2);

    F_e = y_d_est - r;
    PI_e = Kp*e + Ki*x(3);
    u = -F_e + r_d + PI_e; % iPI control law
else

end

dxdt(1,1) = x(2);
dxdt(2,1) = -1*x(1) + u;
dxdt(3,1) = e;
end