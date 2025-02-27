clear all; close all; clc

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 200;

t = time.t0;

simCfg.time = time;

const = setupConstants();

%% Target Initialization
targetLat = 42.33599546; % [deg] Latitude
targetLon = -71.8098593; % [deg] Longitude
targetAlt = 4752; % [m] Altitude MSL

targetLLA = [targetLat, targetLon, targetAlt];
currTargetLLA = targetLLA;

target_ECEF = lla2ecef(targetLLA);

%% Target State Initialization
% initial target conditions
Vt = 300;
B = pi;
Rtx_i = target_ECEF(1);
Rty_i = target_ECEF(2);
Rtz_i = target_ECEF(3);
Vtx_i = Vt*cos(B);
Vty_i = Vt*sin(B);
Vtz_i = 0;
aT = 0*const.g_e;
% x_0_target = [B; Rtx_i; Rty_i; Rtz_i; Vtx_i; Vty_i; Vtz_i];

% stationary target
x_0_target = [B; Rtx_i; Rty_i; Rtz_i; 0; 0; 0];

x_t_target = x_0_target;


numTimePts = time.tf / time.dt+1;

xRecord_target = nan(length(x_0_target), numTimePts);
xRecord_target(:,1) = x_t_target;


%% Target
for i = 1:numTimePts

    k1 = time.dt * TargetKinematicModel(t, x_t_target, aT);
    k2 = time.dt * TargetKinematicModel(t, x_t_target + (1/2)*k1, aT);
    k3 = time.dt * TargetKinematicModel(t, x_t_target + (1/2)*k2, aT);
    k4 = time.dt * TargetKinematicModel(t, x_t_target + k3, aT);
    x_t_target = x_t_target + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;


    xRecord_target(:, i) = x_t_target;

end

% % plot
% figure(1)
% plot3(xRec(2,1:tf), xRec(3,1:tf), xRec(4,1:tf),'linewidth', 2);
% title('Target')
% grid on
% hold off

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