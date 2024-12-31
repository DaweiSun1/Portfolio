% Define the system matrices
A = [1, 0.0952;
     0, 0.905];
B = [0.0484; 
     0.952];
C = [1, 0];
D = 0;

% State feedback gain matrix (replace with your values)
K = [0.2897, 0.1468];  % Replace k1, k2 with your computed gains

% Desired observer poles (choose faster dynamics than the system poles)
observer_poles = [0.5, 0.4];  % Example values, adjust as needed
L = place(A', C', observer_poles)';  % Compute observer gain

% Closed-loop dynamics with observer
A_cl = [A - B*K, B*K;
        zeros(size(A)), A - L*C];
B_cl = [B; zeros(size(B))];
C_cl = [C, zeros(size(C))];
D_cl = 0;

% Define initial conditions
x0 = [10; 0];   % Initial true state
x_hat0 = [0; 1]; % Initial observer state
x_aug0 = [x0; x_hat0]; % Augmented state vector

% Create augmented system
Ts = 0.1;  % Sampling time
sys_aug = ss(A_cl, B_cl, C_cl, D_cl, Ts);

% Simulate step response
t = 0:Ts:5;  % Time vector
[y, t_out, x_aug] = step(sys_aug, t);

% Extract true states and observer states from augmented state
x_true = x_aug(:, 1:2);   % True states
x_hat = x_aug(:, 3:4);    % Estimated states

% Plot results
figure;
subplot(2,1,1);
plot(t_out, x_true, 'LineWidth', 1.5);
grid on;
title('True States of the System');
xlabel('Time (seconds)');
ylabel('State Variables');
legend('x_1 (Position)', 'x_2 (Velocity)');

subplot(2,1,2);
plot(t_out, x_hat, '--', 'LineWidth', 1.5);
grid on;
title('Observer Estimated States');
xlabel('Time (seconds)');
ylabel('Estimated State Variables');
legend('\hat{x}_1', '\hat{x}_2');

% Verify output
figure;
plot(t_out, y, 'LineWidth', 1.5);
grid on;
title('Step Response of the Closed-Loop System with Observer');
xlabel('Time (seconds)');
ylabel('Output Position y(k)');
legend('Closed-Loop Response');
