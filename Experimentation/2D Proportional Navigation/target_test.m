clear all; close all; clc
% theta = [0, 45, 90, 135, 180, 225, 270, 315, 360];
% for i=1:length(theta)
% time steup
dt = 0.001;
tspan = 0:dt:15;
t0 = 0;
t = t0;

% initial target conditions
Rtx_i = 0;
Rtz_i = 0;
Vtx_i = -25;
Vtz_i = 0;
B = deg2rad(180);

% inputs for rk4
numTimePts = length(tspan);

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

tRecord_t = nan(1, numTimePts);
tRecord_t(1,1) = t;

% for missile
x_0 = [B; Rtx_i; Rtz_i;Vtx_i; Vtz_i];

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
plot(xRecord(2,:), xRecord(3,:));
xlim([-400, 400])
ylim([-400, 400])
hold on
% end

% missile dynamic model
function dx = MissileDynamicModel(x, t)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2); Rtz_i = x(3); % target inertial position
    Vtx_i = x(4); Vtz_i = x(5); % target inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vtz_i^2);

    % target derivatives
    dRtx_i = Vt*cos(B);
    dRtz_i = Vt*sin(B);
    dVtx_i = -Vt * sin(B) * 0;
    dVtz_i = Vt * cos(B) * 0;
    dB = 0;

    % state derivative
    dx = [dB; dRtx_i; dRtz_i; dVtx_i; dVtz_i];
end