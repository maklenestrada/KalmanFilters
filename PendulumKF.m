clear all;clc;close all
%Constants 
g = 9.81;   % m/s^2
m = 1;      % kg 
l = 0.5;    % m 

%Noise Covariances 
q = 1e-2 * eye(2); % Process Noise 
r = 1e-3;          % Measurment Noise 


ts = 0.1;          % Sampling Time 
SimTime = 2;        % Sec
steps = SimTime/ts;

%State Space Representation 
%Small Angle Assumption sin(theta) ~= theta
%This makes the inverted pendulum a lin sys
A = [0 1;...
    -g/l 0];
B = [ 0 ; 1/(m*l^2)];
C = [1 0];
D = 0;
sys_c = ss(A, B, C, D);       % define continuous-time system
                              % xdot(t) = Ax(t) + Bu(t)
sys_d = c2d(sys_c, ts);       % convert to discrete-time using zero-order hold
                              % x_k+1 = A_d*x_k + B_d*u_k
A = sys_d.A;
B = sys_d.B;

%Initial State and covariance 
x_true = [.01;0];
x_est = [0;0];
sig = eye(2)*1e-2;
      

%Pre-allocate arrays to store results
x_est_sim = zeros(2, steps);
x_true_sim = zeros(2, steps);
y_sim = zeros(1, steps);

% For simulating process and measurement noise
processNoise = mvnrnd([0 0], q, steps)';  %Rand Gaussian Dist
measurementNoise = sqrt(r) * randn(1, steps);

for i = 1:steps
    %u = 0.03 * sin(2 * pi * 0.2 * i * ts) + 0.01;
    u = 0;
    % Simulate true system (discrete-time ZOH approximation)
    x_true = A*x_true + B*u + processNoise(:, i);
    
    % Simulate measurement
    y = C*x_true + measurementNoise(i);
    
    % Run Kalman Filter
    [x_est, sig] = KalmanFilter(x_est,u,y,sig,A,B,C,q,r);
    
    % Store data
    x_est_sim(:,i) = x_est;
    x_true_sim(:,i) = x_true;
    y_sim(i) = y;
end

% Plot results
time = 0:ts:(SimTime - ts);
figure;
plot(time, x_true_sim(1, :), 'b-', 'LineWidth', 2);
hold on;
plot(time, x_est_sim(1, :), 'r--', 'LineWidth', 2);
legend('True Angle', 'Estimated Angle');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
title('Kalman Filter Pendulum Angle Estimation');
grid on;

figure;
plot(time, x_true_sim(2, :), 'b-', 'LineWidth', 2);
hold on;
plot(time, x_est_sim(2, :), 'r--', 'LineWidth', 2);
legend('True Angular Velocity', 'Estimated Angular Velocity');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Kalman Filter Velocity Estimation');
grid on;

function [x_p1,sig_p1] = KalmanFilter(x_k,u_k,y_k,sig,A,B,C,q,r)
%Predict
x_p = A*x_k + B*u_k;
sig_p = A*sig*A' + q;

%Kalman Gain 
K = sig_p*C'*(C*sig_p*C' + r)^-1;

%Correction 
x_p1 = x_p + K * (y_k - C*x_p);
sig_p1 = (eye(size(sig_p)) - K * C) * sig_p;
end
