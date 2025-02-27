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
Rtx_i = target_ECEF(1);
Rty_i = target_ECEF(2);
Rtz_i = target_ECEF(3);

% stationary target
x_0_target = [Rtx_i; Rty_i; Rtz_i];

x_t_target = x_0_target;


numTimePts = time.tf / time.dt+1;


A = ones(1, numTimePts);
B = x_t_target*A;
