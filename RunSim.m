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

%% Simulator Config

% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
time.tf = 60*5; % [s] Final Time


simCfg.time = time;