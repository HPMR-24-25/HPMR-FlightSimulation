%% WPI High Power Rocket MQP - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 9.29.2024

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
kins = HPMR_MissileKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices

% Aerodynamics Model
AeroModel = initMissileAeroModel();

% Motor Model
MotorModel = initMotorModel();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 50;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat = 44.8244069; % [deg] Latitude
launchLon = -73.1656987; % [deg] Longitude
launchAlt = 183; % [m] Altitude MSL

launchLLA = [launchLat, launchLon, launchAlt];

launch_ECEF_m = lla2ecef(launchLLA);

%% Attitude Initialization
roll_0 = deg2rad(0);
pitch_0 = deg2rad(85);
yaw_0 = deg2rad(0);

q_0 = rpy2quat(roll_0, pitch_0, yaw_0);

% Angular Rate Initialization
w_ib_x = 0.01; % [rad/s]
w_ib_y = 0.01; % [rad/s]
w_ib_z = 0.01; % [rad/s]

%% Velocity Initialization
Vx_E_0 = 0.01; % [m/s]
Vy_E_0 = 0.01; % [m/s]
Vz_E_0 = 0.01; % [m/s]

%% State Initialization
x_0 = [
    q_0';
    launch_ECEF_m';
    Vx_E_0;
    Vy_E_0;
    Vz_E_0;
    w_ib_x;
    w_ib_y;
    w_ib_z;
    kins.m_i;
];

x_t = x_0;

%% RK4 Data Storage
t = time.t0;

numTimePts = time.tf / time.dt+1;

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

xRecord = nan(length(x_0), numTimePts);
xRecord(:,1) = x_t;

%% Prepare Plots
global drag_force_history; % Declare the global variable for drag force history
drag_force_history = [];   % Initialize it as an empty array

% Create a figure for plotting drag force and altitude before the simulation loop
figure();
yyaxis left; % Use left y-axis for drag force
hold on;
xlabel('Sample');
ylabel('Drag Force (N)');
title('Real-time Drag Force and Altitude');
grid on;

% Prepare secondary y-axis for altitude
yyaxis right; % Switch to the right y-axis
ylabel('Altitude (m)');
grid on;

colNum = 1;
while(t <= time.tf)
    colNum = colNum + 1;

    t = t + time.dt;

    k1 = time.dt * MissileDynamicModel(x_t, t, AeroModel, MotorModel, const, kins, inds);
    k2 = time.dt * MissileDynamicModel(x_t + (1/2)*k1, t, AeroModel, MotorModel, const, kins, inds);
    k3 = time.dt * MissileDynamicModel(x_t + (1/2)*k2, t, AeroModel, MotorModel, const, kins, inds);
    k4 = time.dt * MissileDynamicModel(x_t + k3, t, AeroModel, MotorModel, const, kins, inds);

    x_t = x_t + (1/5)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    currLLA = ecef2lla([x_t(inds.px_ecef)', x_t(inds.py_ecef)', x_t(inds.pz_ecef)']);


    % Plotting drag forces on the left y-axis
    yyaxis left;
    plot(drag_force_history(1, 1:end), 'b-'); % Drag force component 1
    plot(drag_force_history(2, 1:end), 'g--'); % Drag force component 2
    plot(drag_force_history(3, 1:end), 'r-');  % Drag force component 3

    % Plotting altitude on the right y-axis
    altitude = currLLA(3); % Assuming altitude is stored in the z coordinate of ECEF
    yyaxis right; % Switch to the right y-axis
    plot(colNum, altitude, 'k.'); % Plot current altitude

    % Update the plot
    drawnow;

    % Switch back to the drag force figure
    yyaxis left; % Ensure the next plots are on the left y-axis

    xRecord(:, colNum) = x_t;
    tRecord(1, colNum) = t;
end

lla = ecef2lla([xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)']);

% Create a geoglobe
uif = uifigure('Name', 'Vehicle Trajectory');
g = geoglobe(uif);

geoplot3(g, lla(:, 1), lla(:,2), lla(:,3));
function q_total = rpy2quat(roll, pitch, yaw)
    % Function to convert Roll, Pitch, Yaw angles to a quaternion
    % Inputs: roll, pitch, yaw (in radians)
    
    % Half-angles for quaternion computation
    half_roll = roll / 2;
    half_pitch = pitch / 2;
    half_yaw = yaw / 2;
    
    % Quaternion for roll (about X axis)
    q_roll = [cos(half_roll), sin(half_roll), 0, 0]; % [qw, qx, qy, qz]
    
    % Quaternion for pitch (about Y axis)
    q_pitch = [cos(half_pitch), 0, sin(half_pitch), 0]; % [qw, qx, qy, qz]
    
    % Quaternion for yaw (about Z axis)
    q_yaw = [cos(half_yaw), 0, 0, sin(half_yaw)]; % [qw, qx, qy, qz]
    
    % Quaternion multiplication: q_total = q_yaw * q_pitch * q_roll
    q_total = quatmultiply(quatmultiply(q_yaw, q_pitch), q_roll);
end