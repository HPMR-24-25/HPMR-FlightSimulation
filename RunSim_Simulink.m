clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
kins = HPMR_MissileKinematics();

% Aerodynamics and Motor Models
AeroModel = initMissileAeroModel();
MotorModel = initMotorModel();

%% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Simulation Start Time
time.tf = 200; % [s] Simulation End Time

simCfg.time = time;

%% Launch Site Initialization
launchLat =  42.2738703;  % [deg] Latitude
launchLon = -71.8098593;  % [deg] Longitude
launchAlt = 180;          % [m] Altitude MSL
launchLLA = [launchLat, launchLon, launchAlt];
currLLA = launchLLA;

% Convert Launch Lat/Lon/Alt to ECEF
launch_ECEF_m = lla2ecef(launchLLA);


%% Attitude Initialization
roll_0 = deg2rad(0);
pitch_0 = deg2rad(85);
yaw_0 = deg2rad(0);

q_0 = eul2quat(roll_0, pitch_0, yaw_0);

% Angular Rate Initialization
w_ib_x = 0.00; % [rad/s]
w_ib_y = 0.00; % [rad/s]
w_ib_z = 0.00; % [rad/s]

% Velocity Initialization
vx_ECEF_0 = 1e-2; % [m/s]
vy_ECEF_0 = 1e-2; % [m/s]
vz_ECEF_0 = 1e-2; % [m/s]

%% State Initialization
x_0 = [
    q_0';
    launch_ECEF_m';
    vx_ECEF_0;
    vy_ECEF_0;
    vz_ECEF_0;
    w_ib_x;
    w_ib_y;
    w_ib_z;
    kins.m_i;
];

%% Simulation Configuration
assignin('base', 'const', const);
assignin('base', 'kins', kins);
assignin('base', 'AeroModel', AeroModel);
assignin('base', 'MotorModel', MotorModel);
assignin('base', 'time', time);
assignin('base', 'x_0', x_0);
assignin('base', 'launch_ECEF_m', launch_ECEF_m);

%% Run the simulink model
simOut = sim('FlightSimulation.slx',...
    'StopTime', num2str(time.tf), ...
    'Solver', 'ode45', ...
    'FixedStep', num2str(time.dt), ...
    'SaveOutput', 'on');

