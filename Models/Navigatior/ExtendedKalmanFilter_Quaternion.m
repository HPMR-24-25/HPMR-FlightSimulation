%% HPMR MQP
% Extended Kalman Filter
% Author: Daniel Pearson
% Version: 11/15/2024

clear variables; close all; clc;

%% Constants

g = 9.80665; % [m/s^2]

%% Sensor Data
sensorData = readtable('polarisTestLog.csv');

%% Load Constants
inds = getKfInds();

%% Time Setup
