clc; clear variables; close all;

% Define system parameters
L_s = 3; % Servo arm length (cm)
L_l = 8; % Linkage length (cm)
L_b = 3; % Blade control horn length (cm)

theta_s = linspace(-20, 20, 80); % Servo angle range (degrees)
theta_s = deg2rad(theta_s); % Convert to radians

% Solve for blade pitch angle theta_b
theta_b = acos(((L_s .* cos(theta_s) + L_l).^2 + L_b^2 - L_l^2) ./ (2 * L_b * (L_s .* cos(theta_s) + L_l)));

% Convert back to degrees
theta_s = rad2deg(theta_s);
theta_b = rad2deg(theta_b);

% Plot the relationship
figure;
plot(theta_s, theta_b, 'b', 'LineWidth', 2);
xlabel('Servo Angle (°)');
ylabel('Blade Pitch Angle (°)');
title('Servo Input vs. Blade Output');
grid on;
axis equal;
