
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Model Free Control %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% changed to spring-mass

% just changed the beginning

clear all; clc; close all;

m = 1;
k = 1;

N = 200;
tsamp = 0.1; % Sampling time
L = 6*tsamp; % L is a parameter required in MFC 
endtime = tsamp*N;
t_vec = 0:tsamp:endtime-tsamp;

parameters.L = L;
parameters.alpha = 1; % another parameter in MFC. This alpha should be such that it magnitude of y and alpha*u should be same.
parameters.tsamp = tsamp;
parameters.Kp = 3.2; % Kp in the P controller. Tune it a little bit if necessary based on your model


% This part is generating data from a sample model to be used in MFC. 
sys.A = [0 1; -k/m 0];
sys.B = [0; 1/m];
sys.C = [1 0];
sys.D = 0;
sys_SM = ss(sys.A,sys.B,sys.C,sys.D); % state-space system

% Simulation setup
u = ones(1,length(t_vec));       % step force input (1 Newton)

% Simulate
[y, t_out, x] = lsim(sys_SM, u, t_vec);

y = y';
x = x';

% u = ones(1,length(t_vec));
% y = zeros(1,length(t_vec));
% x(:,length(t_vec)) = zeros(length(sys.A),1);
% x(:,1) = 0; 
% for k = 1:length(t_vec)
%     y(k) = sys.C * x(:,k) + sys.D * u(k);
%     x(:,k+1) = sys.A * x(:,k) + sys.B * u(k);
% end

% Control part prelim: This is reference trajectory generation. Whatever desired output you want, make that the setpoint. 
setpoint = 3*ones(1,length(t_vec));
G_inertialComp = tf(1,[1 1]);
ref = lsim(G_inertialComp,setpoint,t_vec)';
G_inertialComp_ddt = tf([1 0],[1 1]);
dref = lsim(G_inertialComp_ddt,setpoint,t_vec)';

% Initial settings 
n = length(0:tsamp:L); %n is the number of samples in [t-L,t]
a = 0;
k_a = find(abs(t_vec-(a)) < 1e-6); 
b = a + L;
k_b = find(abs(t_vec-(b)) < 1e-6); 

for i = 1:length(t_vec)-n
    reference.ref = ref(k_b);
    reference.dref = dref(k_b);
    
    e(i) = y(k_b) - ref(k_b);  
    
    % This is the core MFC. Give it previous few input and output measurements, reference and the parameters and you will get the desired control signal uc.
    uc = fct_IPID(u(k_a:k_b),y(k_a:k_b),reference,parameters);
    
    u(k_b+1) = uc;
    y(k_b+1) = sys.C*x(:,k_b+1) + sys.D*u(k_b+1);
    x(:,k_b+2) = sys.A*x(:,k_b+1) + sys.B*u(k_b+1);
    
    a = a + tsamp;
    b = b + tsamp;
    k_a = find(abs(t_vec-(a)) < 1e-6); 
    k_b = find(abs(t_vec-(b)) < 1e-6); 
end

% Plots for analysis
figure;
t_vec = 0:1:length(setpoint)-1;
plot(t_vec,setpoint,'-.k',t_vec,ref,t_vec,y,'LineWidth',2);
legend({'Setpoint','Reference','Measured'},'Location','Southwest','FontSize',12)
title(['# of samples = ' num2str(n)],'FontSize', 15)
xlabel('Time in seconds','FontSize', 15)
ylabel('Output of the system','FontSize', 15)

figure; plot(e); title('Error')

% Plot Desired Trajectory vs Actual Trajectory
figure;
timevector_plot = 0:tsamp:endtime-tsamp; % use correct timevector
plot(timevector_plot, ref, 'k--', 'LineWidth', 2); hold on;
plot(timevector_plot, y(1:length(timevector_plot)), 'b-', 'LineWidth', 2);
grid on;
legend({'Desired Trajectory (ref)', 'Actual Trajectory (y)'}, 'Location', 'Southwest', 'FontSize', 12)
xlabel('Time (seconds)', 'FontSize', 14)
ylabel('Output', 'FontSize', 14)
title('Desired vs Actual Trajectory', 'FontSize', 16)