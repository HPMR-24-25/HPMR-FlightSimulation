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
time.tf = 60*5; % [s] Final Time

t = time.t0;

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
pitch_0 = deg2rad(0);
yaw_0 = deg2rad(20);

q_0 = eul2quat([yaw_0, pitch_0, roll_0], 'ZYX');

% Angular Rate Initialization
w_ib_x = 0; % [rad/s]
w_ib_y = 0; % [rad/s]
w_ib_z = 0; % [rad/s]

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
    kins.m_0;
];

% MissileDynamicModel(x_0, t, AeroModel, MotorModel, const, kins, inds)