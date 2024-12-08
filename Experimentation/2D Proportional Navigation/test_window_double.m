clear all; close all; clc

% time steup
dt = 0.01;
tspan = 0:dt:40;
t0 = 0;
t = t0;

% initial missile conditions
px = 0;
py = 0;
V = 20;
gamma = deg2rad(90);
Vx = V*cos(gamma);
Vy = V*sin(gamma);

% initial target conditions
px_t = 20;
py_t = 20;
V_t = 18;
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

% rk4 Future Missile
t = t0;
x = x_0;
colNum = 1;
for i = 1:length(tspan)
    colNum = colNum + 1;

    t = t + dt;

    k1 = dt * MissileDynamicModelF(x, t);
    k2 = dt * MissileDynamicModelF(x + (1/2)*k1, t);
    k3 = dt * MissileDynamicModelF(x + (1/2)*k2, t);
    k4 = dt * MissileDynamicModelF(x + k3, t);

    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecordF(:, colNum) = x;
    tRecordF(1, colNum) = t;
end

% miss distance
for i = 1:length(tRecord)
%     missilepos(i) = sqrt(xRecord(1,i)^2+xRecord(2,i)^2);
%     missileposF(i) = sqrt(xRecordF(1,i)^2+xRecordF(2,i)^2);
%     targetpos(i) = sqrt(xRecord(6,i)^2+xRecord(7,i)^2);
%     miss_dist(i) = abs(targetpos(i)-missilepos(i));
%     miss_distF(i) = abs(targetpos(i)-missileposF(i));
    LOS = [xRecord(1,i); xRecord(2,i)]-[xRecord(6,i); xRecord(7,i)];
    LOSF = [xRecordF(1,i); xRecordF(2,i)]-[xRecord(6,i); xRecord(7,i)];
    miss_dist(i) = abs(norm(LOS));
    miss_distF(i) = abs(norm(LOSF));
end

figure;
plot(tRecord,miss_dist)
hold on
plot(tRecord,miss_distF)
legend('Non','F')
hold off

% plot
figure;
plot(xRecord(1,:), xRecord(2,:));
hold on
plot(xRecord(6,:), xRecord(7,:));
plot(xRecordF(1,:), xRecordF(2,:));
%axis([-30 0 -1 5]);
%ylim([0 12])

% Initialize animated objects
missilePlot = plot(xRecord(1,1), xRecord(2,1), 'ro', 'MarkerSize', 8, 'DisplayName', 'Missile');
targetPlot = plot(xRecord(6,1), xRecord(7,1), 'bo', 'MarkerSize', 8, 'DisplayName', 'Target');
missilePlotF = plot(xRecordF(1,1), xRecordF(2,1), 'ko', 'MarkerSize', 8, 'DisplayName', 'MissileF');
legend('Location','southwest');

% Animate
for i = 1:numTimePts
    set(missilePlot, 'XData', xRecord(1,i), 'YData', xRecord(2,i));
    set(targetPlot, 'XData', xRecord(6,i), 'YData', xRecord(7,i));
    set(missilePlotF, 'XData', xRecordF(1,i), 'YData', xRecordF(2,i));
    pause(0.000000001); % Adjust to control animation speed
end


%%
% missile dynamic model
function x_dot = MissileDynamicModel(x, t)

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
    LOS = [px_t; py_t]-[px; py];

    % LOS Unit Vector
    LOS_vec = LOS./norm(LOS);
    %LOS_vec = LOS./(norm([px_t; py_t])-norm([px; py]));

    % x-axis unit vector
    i = [1;0];

    % LOS Angle
    lamda = acos(dot(LOS_vec,i));

    % rate of LOS Vector
    LOS_dot = [Vx_t; Vy_t]-[Vx; Vy];

    % rate of LOS Unit Vector
    LOS_dot_vec = LOS_dot./norm(LOS_dot);
    %LOS_dot_vec = LOS_dot./(norm([Vx_t; Vy_t])-norm([Vx; Vy]));

    % rate of LOS Angle
    lamda_dot = -1/sqrt(1-(dot(LOS_vec,i))^2)*dot(LOS_dot_vec,i);

    % proportional gain value
    N = 2;

    % Derivatives for missile
    px_dot = V*cos(gamma);
    py_dot = V*sin(gamma);
    Vx_dot = -V * sin(gamma) * N*lamda_dot;
    Vy_dot = V * cos(gamma) * N*lamda_dot;
    gamma_dot = N*lamda_dot;

    % Derivatives for target
    px_t_dot = V_t*cos(theta);
    py_t_dot = V_t*sin(theta);
    Vx_t_dot = -V_t * sin(theta) * 0;
    Vy_t_dot = V_t * cos(theta) * 0;
    theta_dot = 0;    

    % Combine derivatives into x_dot
    x_dot = [px_dot; py_dot; Vx_dot; Vy_dot; gamma_dot; px_t_dot; py_t_dot; Vx_t_dot; Vy_t_dot; theta_dot];
end

% future missile dynamic model
function x_dot = MissileDynamicModelF(x, t)

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
    %LOS_vec = LOS./(norm([px_t_f; py_t_f])-norm([px; py]));

    % Future LOS Angle
    lamda = acos(dot(LOS_vec,[1; 0]));

    % rate of LOS Vector
    LOS_dot = [Vx_t; Vy_t]-[Vx; Vy];

    % rate of LOS Unit Vector
    LOS_dot_vec = LOS_dot./norm(LOS_dot);
    %LOS_dot_vec = LOS_dot./(norm([Vx_t; Vy_t])-norm([Vx; Vy]));


    % rate of LOS Angle
    lamda_dot = -1/sqrt(1-(dot(LOS_vec,[1; 0]))^2)*dot(LOS_dot_vec,[1; 0]);

    % proportional gain value
    N = 2;

    % Derivatives for missile
    px_dot = V*cos(gamma);
    py_dot = V*sin(gamma);
    Vx_dot = -V * sin(gamma) * N*lamda_dot;
    Vy_dot = V * cos(gamma) * N*lamda_dot;
    gamma_dot = N*lamda_dot;

    % Derivatives for target
    px_t_dot = V_t*cos(theta);
    py_t_dot = V_t*sin(theta);
    Vx_t_dot = -V_t * sin(theta) * 0;
    Vy_t_dot = V_t * cos(theta) * 0;
    theta_dot = 0;

    % Combine derivatives into x_dot
    x_dot = [px_dot; py_dot; Vx_dot; Vy_dot; gamma_dot; px_t_dot; py_t_dot; Vx_t_dot; Vy_t_dot; theta_dot];
end