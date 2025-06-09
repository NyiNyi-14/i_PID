clear all; close all; clc;
 
% ==============================
% SYSTEM DEFINITION
% ==============================
% We define a 2nd order underdamped plant with a PD controller.
zeta = 0.1;          % Damping ratio
wn = 1;              % Natural frequency
Kp = 1;              % Proportional gain
Kd = 0.1;            % Derivative gain
% Ki = 10; 

% Create transfer functions
s = tf('s');
Gp = 1 / (s^2 + 2*zeta*wn*s + wn^2);   % Plant
% C = Kp + Kd * s + Ki/s;                       % PD controller
C = Kp + Kd * s;
G_open = C * Gp;                       % Open-loop transfer function
 
% Get numerator and denominator polynomials
[num, den] = tfdata(G_open, 'v');
 
%% 

% ==============================
% CLOSED-FORM DERIVATIVE (ds/dK)
% ==============================
% Characteristic equation: D(s) + K*N(s) = 0
% Implicitly differentiate w.r.t. K:
%    d/dK[D(s(K)) + K*N(s(K))] = 0
%    => (dD/ds + K*dN/ds) * ds/dK + N(s) = 0
%    => ds/dK = -N(s) / (dD/ds + K*dN/ds)
syms s_sym K_sym
N = poly2sym(num, s_sym);   % N(s)
D = poly2sym(den, s_sym);   % D(s)
 
dD_ds = diff(D, s_sym);     % dD/ds
dN_ds = diff(N, s_sym);     % dN/ds
 
% Symbolic expression for slope of root locus
ds_dK_expr = -N / (dD_ds + K_sym * dN_ds);

 %% 

% ==============================
% ROOT LOCUS EVALUATION
% ==============================
% Use rlocus to get poles for many K values
k = linspace(0.01, 250, 200);  % Skip K = 0 to avoid divide-by-zero
[r, K] = rlocus(G_open, k);     % r: pole locations, K: gains
 
% Display header
fprintf('\n%-15s %-15s %-15s %-20s\n', 'Real Part', 'Imag Part', 'Magnitude', 'Slope Angle (deg)');
fprintf('%s\n', repmat('-', 1, 70));
 
% Initialize list of turning points
turning_points = [];
 
% ==============================
% LOOP THROUGH ROOT LOCUS POINTS
% ==============================
[n_branches, n_samples] = size(r);
 
for i = 1:n_branches
    for j = 1:n_samples
        s_val = r(i, j);     % Current pole
        k_val = K(j);        % Current gain
        
        % Numerically evaluate the symbolic derivative at (s, K)
        ds_dK_val = double(subs(ds_dK_expr, {s_sym, K_sym}, {s_val, k_val}));
        
        % Get the angle (direction of motion of pole)
        angle_deg = rad2deg(angle(ds_dK_val));
        
        % Normalize angle to [-180, 180] to handle wraparound
        angle_deg_mod = mod(angle_deg + 180, 360) - 180;
        
        % Print results to command window
        fprintf('%-15.4f %-15.4f %-15.4f %-20.2f\n', ...
            real(s_val), imag(s_val), abs(s_val), angle_deg_mod);
        
        horizontal_threshold = 2;  % degrees
        % Check if slope is nearly horizontal (left or right)
        is_horizontal = abs(angle_deg_mod) <= horizontal_threshold || ...
                        abs(angle_deg_mod + 180) <= horizontal_threshold;

        if is_horizontal
            turning_points = [turning_points; s_val];
        end

%         % Turning point condition: slope approximately horizontal
%         if abs(angle_deg_mod) < 2 || abs(abs(angle_deg_mod) - 180) < 2
%             turning_points = [turning_points; s_val];
%         end

    end
end
 
% ==============================
% PLOTTING
% ==============================
figure;
rlocus(G_open); hold on; grid on;
 
% Mark turning points
plot(real(turning_points), imag(turning_points), 'ro', ...
     'MarkerSize', 8, 'LineWidth', 2);
 
title('Root Locus with Turning Points (Closed-form Slope)');
xlabel('Real Axis'); ylabel('Imaginary Axis');
legend('Root Locus', 'Turning Points');





