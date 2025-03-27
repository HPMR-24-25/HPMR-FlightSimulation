%% WPI High Power Rocket MQP - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 12.15.2024

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
% kins = HPMR_MissileKinematics();
kins = HPMR_ModelRocketKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices

% Aerodynamics Model
% AeroModel = initMissileAeroModel();
AeroModel = initRocketAeroModel();

% IMU Model
ImuModel = getASM330Params();

% Motor Model
MotorModel = initMotorModel();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 75;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat =  42.2738703; % [deg] Latitude
launchLon = -71.8098593; % [deg] Longitude
launchAlt = 25; % [m] Altitude MSL

launchLLA = [launchLat, launchLon, launchAlt];
% currLLA = launchLLA;

launch_ECEF_m = lla2ecef(launchLLA);

% Attitude Initialization
yaw_0 = deg2rad(0);
roll_0 = deg2rad(0);
pitch_0 = deg2rad(45);

eul_0 = [roll_0; pitch_0; yaw_0];

q_0 = hpmr_eul2quat(yaw_0, pitch_0, roll_0);

% Angular Rate Initialization
w_ib_x = 1e-5; % [rad/s]
w_ib_y = 1e-5; % [rad/s]
w_ib_z = 1e-5; % [rad/s]

% Velocity Initialization
R_ET = [
    -sind(launchLat)*cosd(launchLon), -sind(launchLon), -cosd(launchLat)*cosd(launchLon);
    -sind(launchLat)*sind(launchLon),  cosd(launchLon), -cosd(launchLat)*sind(launchLon);
     cosd(launchLat),            0,         -sind(launchLat)
];

R_TB = quat2rotm(q_0');
R_EB = R_ET * R_TB;

V_0_B = [1; 1e-5; 1e-5];

V_0_E = R_EB * V_0_B;

% Initial Mass
m_0 = kins.m_0 + MotorModel.emptyWt + MotorModel.propWt;

%% State Initialization
x_0 = [
    q_0;
    launch_ECEF_m';
    V_0_E(1);
    V_0_E(2);
    V_0_E(3);
    w_ib_x;
    w_ib_y;
    w_ib_z;
    m_0;
];

%% Load Simulink Model
modelName = 'FlightSimulation';
saveRate = 10; % [Hz]
saveDir = fullfile(pwd, 'SIM_OUT');

% Open Simulation

open_system(modelName);

% Start Timer
tic

SimOut = sim(modelName, 'StopTime', num2str(simCfg.time.tf), 'SaveOutput', 'on');

runTime = toc;

fprintf("Run Time: %.1f sec\n", runTime);

% Assign Parameters
assignin('base', 'const', const);
assignin('base', 'kins', kins);
assignin('base', 'inds', inds);
assignin('base', 'AeroModel', AeroModel);
assignin('base', 'ImuModel', ImuModel);
assignin('base', 'MotorModel', MotorModel);
assignin('base', 'simCfg', simCfg);
assignin('base', 'launchLLA', launchLLA);
assignin('base', 'x_0', x_0);