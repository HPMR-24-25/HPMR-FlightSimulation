clear all; close all; clc

% time steup
dt = 1e-3;  % use to increase accuracy
tf = 50;
t = 0:dt:tf-dt;
nt = length(t);

% gravity
g = 32.2;

% initial target conditions
Vt = 3400;
B = pi;
Rtx_i = 40000;
Rty_i = 10000;
Rtz_i = 1000;
Vtx_i = Vt*cos(B);
Vty_i = Vt*sin(B);
Vtz_i = 0;
aT = 3*g;

% inputs for rk4

% for missile
x_0 = [B; Rtx_i; Rty_i; Rtz_i; Vtx_i; Vty_i; Vtz_i];

x = x_0;

xRec = zeros(length(x_0), nt);
xRec(:,1) = x_0;


% rk4 Missile
for i = 1:nt-1

    k1 = dt * TargetKinematicModel(t, x, aT);
    k2 = dt * TargetKinematicModel(t, x + (1/2)*k1, aT);
    k3 = dt * TargetKinematicModel(t, x + (1/2)*k2, aT);
    k4 = dt * TargetKinematicModel(t, x + k3, aT);
    x = x + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRec(:, i+1) = x;

end

% plot
figure(1)
plot3(xRec(2,1:tf), xRec(3,1:tf), xRec(4,1:tf),'linewidth', 2);
title('Target')
grid on
hold off

% Target Kinematic model
function dx = TargetKinematicModel(t, x, aT)

    % Extract State Variables
    B = x(1); % target flight path angle
    Rtx_i = x(2);  Rty_i = x(3);  Rtz_i = x(4); % target inertial position
    Vtx_i = x(5);  Vty_i = x(6);  Vtz_i = x(7); % target inertial velocity

    % target velocity magnitude
    Vt = sqrt(Vtx_i^2+Vty_i^2+Vtz_i^2);

    % target derivatives
    dB = aT/Vt;
    dRtx_i = Vt*cos(B);
    dRty_i = Vt*sin(B);
    dRtz_i = 0;
    dVtx_i = aT * sin(B);
    dVty_i = aT * cos(B);
    dVtz_i = 0;

    % state derivative
    dx = [dB; dRtx_i; dRty_i; dRtz_i; dVtx_i; dVty_i; dVtz_i];
end