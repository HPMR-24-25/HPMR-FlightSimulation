clear all; close all; clc
global N m
N = [1 2 3 4 5];
for m = 1:length(N)
% time steup
dt = 0.1;
tspan = 0:dt:140;
t0 = 0;
t = t0;

% initial missile conditions
px = 0;
py = 0;
V = 1;
gamma = deg2rad(90);
Vx = V*cos(gamma);
Vy = V*sin(gamma);

% initial target conditions
px_t = 10;
py_t = 10;
V_t = 1;
theta = deg2rad(180);
Vx_t = V*cos(theta);
Vy_t = V*sin(theta);

% inputs for rk4
numTimePts = length(tspan);

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

tRecord_t = nan(1, numTimePts);
tRecord_t(1,1) = t;

% for missile
x_0 = [px; py; Vx; Vy; gamma; px_t; py_t; Vx_t; Vy_t; theta];

x = x_0;

xRecord = nan(length(x_0), numTimePts);
xRecord(:,1) = x;

% rk4 Missile
colNum = 1;
for i = 1:length(tspan)
    colNum = colNum + 1;

    t = t + dt;

    k1 = dt * MissileDynamicModel(x, t);
    k2 = dt * MissileDynamicModel(x + (1/2)*k1, t);
    k3 = dt * MissileDynamicModel(x + (1/2)*k2, t);
    k4 = dt * MissileDynamicModel(x + k3, t);

    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord(:, colNum) = x;
    tRecord(1, colNum) = t;
end

% plot
%figure;
plot(xRecord(1,:), xRecord(2,:));
hold on
% plot(xRecord(6,:), xRecord(7,:));
%axis([-30 0 -1 5]);
ylim([0 12])

% % Initialize animated objects
% missilePlot = plot(xRecord(1,1), xRecord(2,1), 'ro', 'MarkerSize', 8, 'DisplayName', 'Missile');
% targetPlot = plot(xRecord(6,1), xRecord(7,1), 'bo', 'MarkerSize', 8, 'DisplayName', 'Target');
% legend('Location','southwest');
% 
% % Animate
% for i = 1:numTimePts
%     set(missilePlot, 'XData', xRecord(1,i), 'YData', xRecord(2,i));
%     set(targetPlot, 'XData', xRecord(6,i), 'YData', xRecord(7,i));
%     pause(0.0001); % Adjust to control animation speed
% end
end
plot(xRecord(6,:), xRecord(7,:));
Legend=cell(5,1);%  two positions 
Legend{1}='N = 1' ;
Legend{2}='N = 2';
Legend{3}='N = 3';
Legend{4}='N = 4';
Legend{5}='N = 5';
legend(Legend);
%%
% missile dynamic model
function x_dot = MissileDynamicModel(x, t)

    global N m
    % Extract state variables
    px = x(1);   py = x(2);   % Missile position
    Vx = x(3);   Vy = x(4);   % Missile velocity
    gamma = x(5);
    px_t = x(6);   py_t = x(7);   % Missile position
    Vx_t = x(8);   Vy_t = x(9);   % Missile velocity
    theta = x(10);

    % Missile Velocity
    V = sqrt(Vx^2+Vy^2);

    % Target Velocity
    V_t = sqrt(Vx_t^2+Vy_t^2);

    % LOS Vector
    LOS0 = [px_t; py_t]-[px; py];

    % LOS Magnitude
    LOS_mag = norm(LOS0);

    % time to go
    t_go = LOS_mag/V;

    % predicted future target position
    px_t_f = Vx_t*t_go+px_t;
    py_t_f = Vy_t*t_go+py_t;

    % Future LOS Vector
    LOS = [px_t_f; py_t_f]-[px; py];

    % Future LOS Unit Vector
    LOS_vec = LOS./norm(LOS);

    % Future LOS Angle
    lamda = acos(dot(LOS_vec,[1; 0]));

    % rate of LOS Vector
    LOS_dot = [Vx_t; Vy_t]-[Vx; Vy];

    % rate of LOS Unit Vector
    LOS_dot_vec = LOS_dot./norm(LOS_dot);

    % rate of LOS Angle
    lamda_dot = -1/sqrt(1-(dot(LOS_vec,[1; 0]))^2)*dot(LOS_dot_vec,[1; 0]);

    % proportional gain value
    %N = 2;

    % Derivatives for missile
    px_dot = V*cos(gamma);
    py_dot = V*sin(gamma);
    Vx_dot = -V * sin(gamma) * N(m)*lamda_dot;
    Vy_dot = V * cos(gamma) * N(m)*lamda_dot;
    gamma_dot = N(m)*lamda_dot;

    % Derivatives for target
    px_t_dot = V_t*cos(theta);
    py_t_dot = V_t*sin(theta);
    Vx_t_dot = -V_t * sin(theta) * 0;
    Vy_t_dot = V_t * cos(theta) * 0;
    theta_dot = 0;    

    % Combine derivatives into x_dot
    x_dot = [px_dot; py_dot; Vx_dot; Vy_dot; gamma_dot; px_t_dot; py_t_dot; Vx_t_dot; Vy_t_dot; theta_dot];
end