% % clear all; close all; clc
% % 
% % % time steup
% % dt = 0.1;
% % tspan = 0:dt:100;
% % t0 = 0;
% % t = t0;
% % 
% % % initial missile conditions
% % p = 0;
% % v = 0;
% % 
% % % initial target conditions
% % r = 10;
% % 
% % % inputs for rk4
% % numTimePts = length(tspan);
% % 
% % tRecord = nan(1, numTimePts);
% % tRecord(1,1) = t;
% % 
% % % for missile
% % x_0 = [p; v];
% % 
% % x = x_0;
% % 
% % xRecord = nan(length(x_0), numTimePts);
% % xRecord(:,1) = x;
% % 
% % % rk4 Missile
% % colNum = 1;
% % for i = 1:length(tspan)
% %     colNum = colNum + 1;
% % 
% %     t = t + dt;
% % 
% %     k1 = dt * plant(x, r, t);
% %     k2 = dt * plant(x + (1/2)*k1, r, t);
% %     k3 = dt * plant(x + (1/2)*k2, r, t);
% %     k4 = dt * plant(x + k3, r, t);
% % 
% %     x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;
% % 
% %     xRecord(:, colNum) = x;
% %     tRecord(1, colNum) = t;
% % end
% % 
% % % plot
% % figure;
% % plot([tspan 100.1],xRecord(1,:))
% % 
% % function dx = plant(x, r, dt)
% % m = 1;
% % K = 1.2;
% % %g = 9.8;
% % 
% % p = x(1);
% % v = x(2);
% % 
% % err = r- p;
% % 
% % u = err*K;
% % 
% % dx(1) = v;
% % dx(2) = u/m;
% % dx = dx';
% % end 
% 
% clear all; close all; clc
% 
% % Time setup
% dt = 0.1;
% tspan = 0:dt:100;
% 
% % Initial missile conditions
% p = 0;  % Position
% v = 0;  % Velocity
% r = 10; % Target position
% 
% % PID gains
% K_p = 0.5; % Proportional gain
% K_i = 1.5; % Integral gain
% K_d = 0; % Derivative gain
% 
% % Initialize PID terms
% integral_error = 0;
% prev_error = 0;
% 
% % RK4 initialization
% numTimePts = length(tspan);
% tRecord = nan(1, numTimePts);
% xRecord = nan(2, numTimePts);
% 
% tRecord(1) = 0;       % Initial time
% xRecord(:, 1) = [p; v]; % Initial state
% 
% x = [p; v]; % Initial state vector
% 
% % RK4 integration with PID control
% for i = 1:numTimePts-1
%     t = tspan(i);
% 
%     % Compute RK4 steps
%     k1 = dt * plant(x, r, t, K_p, K_i, K_d, integral_error, prev_error);
%     k2 = dt * plant(x + (1/2)*k1, r, t, K_p, K_i, K_d, integral_error, prev_error);
%     k3 = dt * plant(x + (1/2)*k2, r, t, K_p, K_i, K_d, integral_error, prev_error);
%     k4 = dt * plant(x + k3, r, t, K_p, K_i, K_d, integral_error, prev_error);
% 
%     % Update state
%     x = x + (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;
% 
%     % Update PID terms
%     error = r - x(1); % Current error
%     integral_error = integral_error + error * dt; % Accumulate integral of error
%     prev_error = error; % Store current error for next derivative calculation
% 
%     % Store results
%     xRecord(:, i+1) = x;
%     tRecord(i+1) = t + dt;
% end
% 
% % Plotting results
% figure;
% plot(tRecord, xRecord(1, :), 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Position (m)');
% title('Missile Position Over Time with PID Control');
% grid on;
% 
% % Plant function with PID control
% function dx = plant(x, r, t, K_p, K_i, K_d, integral_error, prev_error)
%     m = 5;  % Mass
% 
%     % Extract state variables
%     p = x(1); % Position
%     v = x(2); % Velocity
% 
%     % Compute error
%     error = r - p;
% 
%     % Compute derivative of error
%     derivative_error = (error - prev_error) / 0.1; % dt is hardcoded here for simplicity
% 
%     % PID control law
%     u = K_p * error + K_i * integral_error + K_d * derivative_error;
% 
%     % State derivatives
%     dx(1) = v;      % dp/dt = velocity
%     dx(2) = u / m;  % dv/dt = acceleration
%     dx = dx';       % Return as column vector
% end


s = tf('s');
P = 1/(s^2 + 10*s + 20);
step(P)