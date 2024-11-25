%% HPMR MQP - Sensor Mocking
% Generate mock sensor readings based off of arbitrary trajectories
% Author: Daniel Pearson
% Version: 11/24/2024

clear variables; close all; clc;

%% Constants
g = 9.80665; % [m/s^2] Gravitational Acceleration

mag_ned = [50; 0; -20]; % [uT] NED magnetic field vector

%% Time Keeping
t_total = 0:0.01:500; % [s] Total Time Span

% Define transition phases for smooth movement
t_static = t_total(t_total <= 5); % 5 seconds of stationary state
t_transition = t_total(t_total > 5 & t_total <= 15); % 10 seconds for smooth acceleration
t_dynamic = t_total(t_total > 15); % Dynamic motion

%% Generate 3D Position Trajectory
% Stationary phase: position stays constant
position_static = zeros(3, length(t_static));

% Transition phase: smooth acceleration using sine function
a_max = 0.5; % Maximum acceleration magnitude [m/s^2]
accel_transition = a_max * sin(pi * (t_transition - t_transition(1)) / (t_transition(end) - t_transition(1)));

velocity_transition = cumtrapz(t_transition, accel_transition); % Integrate acceleration
position_transition = cumtrapz(t_transition, velocity_transition); % Integrate velocity

% Dynamic phase: simulate random variations in acceleration
accel_dynamic = a_max * (0.8 + 0.2 * rand(3, length(t_dynamic))); % Randomized acceleration
velocity_dynamic = cumtrapz(t_dynamic, accel_dynamic, 2); % Integrate acceleration
position_dynamic = cumtrapz(t_dynamic, velocity_dynamic, 2); % Integrate velocity

% Combine all phases
% position = [zeros(size(position_transition, 1), size(position_transition, 2)); position_transition]


%% Orientation (Yaw, Pitch, Roll) Trajectory
% Generate smooth yaw, pitch, and roll trajectories based on position velocity
velocity = [zeros(3, 1), diff(position, 1, 2) ./ diff(t_total)]; % Velocity vector
yaw = atan2d(velocity(2, :), velocity(1, :)); % [deg] Yaw angle
pitch = asind(velocity(3, :) ./ vecnorm(velocity, 2, 1)); % [deg] Pitch angle
roll = zeros(size(yaw)); % Assume roll remains constant for simplicity

%% IMU Sensor Readings
% Initialize angular velocity (wx, wy, wz)
wx = zeros(1, length(t_total));
wy = zeros(1, length(t_total));
wz = zeros(1, length(t_total));

% Initialize accelerometer readings (ax, ay, az)
ax = zeros(1, length(t_total));
ay = zeros(1, length(t_total));
az = zeros(1, length(t_total));

% Initialize magnetometer readings (mx, my, mz)
mx = zeros(1, length(t_total));
my = zeros(1, length(t_total));
mz = zeros(1, length(t_total));

% Add bias and noise to sensor readings
bias_gyro = [0.0175; -0.02; 0.015]; % Gyroscope bias [rad/s]
bias_accel = [0.049; -0.07; 0.024]; % Accelerometer bias [m/s^2]
bias_mag = [0.3; -0.4; 0.2]; % Magnetometer bias [uT]

std_dev_gyro = 0.0005; % Gyroscope noise [rad/s]
std_dev_accel = 0.0049; % Accelerometer noise [m/s^2]
std_dev_mag = 0.05; % Magnetometer noise [uT]

% Compute angular velocities and sensor readings
for i = 2:length(t_total)
    dt = t_total(i) - t_total(i-1);
    
    % Derivatives of Euler angles
    d_yaw = (yaw(i) - yaw(i-1)) / dt;
    d_pitch = (pitch(i) - pitch(i-1)) / dt;
    d_roll = (roll(i) - roll(i-1)) / dt;
    
    % Angular velocity
    wx(i) = d_roll - sin(pitch(i)) * d_yaw;
    wy(i) = cos(roll(i)) * d_pitch + sin(roll(i)) * cos(pitch(i)) * d_yaw;
    wz(i) = -sin(roll(i)) * d_pitch + cos(roll(i)) * cos(pitch(i)) * d_yaw;
    
    % Accelerometer readings
    ax(i) = g * sin(pitch(i));
    ay(i) = -g*sin(roll(i))*cos(pitch(i));
    az(i) = -g*cos(roll(i))*cos(pitch(i));

    % Magnetometer readings
    R = angle2dcm(deg2rad(yaw(i)), deg2rad(pitch(i)), deg2rad(roll(i)), 'ZYX');
    mag_body = R * mag_ned;

    mx(i) = mag_body(1);
    my(i) = mag_body(2);
    mz(i) = mag_body(3);
end

% Add noise and bias
wx = wx + bias_gyro(1) + std_dev_gyro * randn(size(wx));
wy = wy + bias_gyro(2) + std_dev_gyro * randn(size(wy));
wz = wz + bias_gyro(3) + std_dev_gyro * randn(size(wz));

ax = ax + bias_accel(1) + std_dev_accel * randn(size(ax));
ay = ay + bias_accel(2) + std_dev_accel * randn(size(ay));
az = az + bias_accel(3) + std_dev_accel * randn(size(az));

mx = mx + bias_mag(1) + std_dev_mag * randn(size(mx));
my = my + bias_mag(2) + std_dev_mag * randn(size(my));
mz = mz + bias_mag(3) + std_dev_mag * randn(size(mz));

%% Save Data
imu_data.time = t_total;
imu_data.accelerometer = [ax; ay; az];
imu_data.gyroscope = [wx; wy; wz];

truth_data.euler_angles.yaw = yaw;
truth_data.euler_angles.pitch = pitch;
truth_data.euler_angles.roll = roll;

save('SmoothFakeTraj.mat', 'imu_data', 'truth_data');

%% Plot Results
figure;
subplot(3, 1, 1); plot(t_total, yaw); title('Yaw (ψ)'); xlabel('Time (s)'); ylabel('Angle (deg)');
subplot(3, 1, 2); plot(t_total, pitch); title('Pitch (θ)'); xlabel('Time (s)'); ylabel('Angle (deg)');
subplot(3, 1, 3); plot(t_total, roll); title('Roll (φ)'); xlabel('Time (s)'); ylabel('Angle (deg)');
