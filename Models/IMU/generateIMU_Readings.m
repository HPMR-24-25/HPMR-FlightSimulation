function sensorReadings = generateIMU_Readings(state, accel_ecef, ImuModel, inds, const)
% GENERATEIMU_READINGS: Simulates noisy and biased sensor readings
%
% Inputs:
%   state - Current State Vector
%   accel_ecef - ECEF Acceleration Vector
%   sensorParams - Parameter struct of IMU constants
%   inds - Indices struct for accessing state variables
%
% Outputs:
%   sensorReadings - Struct containing simulated sensor readings

%% Extract State
pos_ECEF = state(inds.pos);
vel_ECEF = state(inds.vel);
angRates = state(inds.w_ib);

% Assemble Rotation Matrix
quat = [state(inds.qw), state(inds.qx), state(inds.qy), state(inds.qz)];

lla = ecef2lla(pos_ECEF', 'WGS84');
lat = lla(1);
lon = lla(2);
alt = lla(3);

%% Rotation Matrix Setup

R_ET = [
    -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
    -sind(lat)*sind(lon),  cosd(lon), -cosd(lat)*sind(lon);
     cosd(lat),            0,         -sind(lat)
];

R_TB = quat2rotm(quat);

R_EB = R_ET * R_TB;

% Accelerometer Readings
accel_NED = R_ET' * accel_ecef ./ const.g_e;
accel_NED = accel_NED + [0; 0; 1]; % Add Gravity
accelBody = R_TB' * accel_NED;

accelNoise = ImuModel.accel.noise .* randn(3,1);
accelBias = [-0.1e-6; 0.2e-6; 0.09e-6];
accelReading = accelBody + accelNoise + accelBias;

% Gyroscope Readings
gyroNoise = ImuModel.gyro.noise .* randn(3,1);
gyroBias = [ImuModel.gyro.bias.x; ImuModel.gyro.bias.y; ImuModel.gyro.bias.z];
gyroReading = angRates + gyroNoise + gyroBias;

% GPS Readings
gpsPosNoise = 2 .* randn(3,1);
gpsVelNoise = 2 .* randn(3,1);

gpsPosReading = pos_ECEF + gpsPosNoise;
gpsVelReading = vel_ECEF + gpsVelNoise;

sensorReadings = struct(...
    'accel', accelReading, ...
    'gyro', gyroReading,...
    'gpsPos', gpsPosReading, ...
    'gpsVel', gpsVelReading);

end