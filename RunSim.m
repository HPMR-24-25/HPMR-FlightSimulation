%% WPI High Power Rocket MQP - Flight Simulator
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 12.15.2024

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
kins = HPMR_MissileKinematics();
% kins = HPMR_ModelRocketKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices

% Aerodynamics Model
AeroModel = initMissileAeroModel();
% AeroModel = initRocketAeroModel();

% Motor Model
MotorModel = initMotorModel();

ImuModel = getASM330Params();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 200;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat =  42.2738703; % [deg] Latitude
launchLon = -71.8098593; % [deg] Longitude
launchAlt = 180; % [m] Altitude MSL

launchLLA = [launchLat, launchLon, launchAlt];
currLLA = launchLLA;

launch_ECEF_m = lla2ecef(launchLLA);

%% Target Initialization
targetLat = 42.33599546; % [deg] Latitude
targetLon = -71.8098593; % [deg] Longitude
targetAlt = 4752; % [m] Altitude MSL

targetLLA = [targetLat, targetLon, targetAlt];
currTargetLLA = targetLLA;

target_ECEF = lla2ecef(targetLLA);

%% Attitude Initialization
roll_0 = deg2rad(0);
pitch_0 = deg2rad(45);
yaw_0 = deg2rad(0);

q_0 = eul2quat(roll_0, pitch_0, yaw_0);

%$ Angular Rate Initialization
w_ib_x = 0.00; % [rad/s]
w_ib_y = 0.00; % [rad/s]
w_ib_z = 0.00; % [rad/s]

%% Velocity Initialization
Vx_E_0 = 1e-2; % [m/s]
Vy_E_0 = 1e-2; % [m/s]
Vz_E_0 = 1e-2; % [m/s]

%% Initial Mass
m_0 = kins.m_0 + MotorModel.emptyWt + MotorModel.propWt;

%% Target Velocity Initialization
V_NED_m = 482.26; %[m/s]
psi = 0;
Vx_target = V_NED_m*sin(psi); % [m/s]
Vy_target = V_NED_m*cos(psi); % [m/s]
Vz_target = 0; % [m/s]

R_ET = [
        -sind(targetLat)*cosd(targetLon), -sind(targetLon), -cosd(targetLat)*cosd(targetLon);
        -sind(targetLat)*sind(targetLon),  cosd(targetLon), -cosd(targetLat)*sind(targetLon);
         cosd(targetLat),                                0,                  -sind(targetLat)
    ];

V_target_NED = [Vx_target; Vy_target; Vz_target];

V_target_ECEF = R_ET*V_target_NED;

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
    m_0;
];

x_t = x_0;

%% Target State Initialization
x_0_target = [target_ECEF'; V_target_ECEF];

x_t_target = x_0_target;
x_t_targetCircle = x_t_target;

%% State Data Storage
t = time.t0;

numTimePts = time.tf / time.dt+1;

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

xRecord = nan(length(x_0), numTimePts);
xRecord(:,1) = x_t;

xRecord_target = nan(length(x_0_target), numTimePts);
xRecord_target(:,1) = x_t_target;

xRecord_targetCircle = nan(length(x_0_target), numTimePts);
xRecord_targetCircle(:,1) = x_t_targetCircle;

tSpan = [0, time.tf];  % Start time and end time
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);  % Tolerances for ode45

% Guidance Storage
cmdHist = zeros(4, numTimePts);
% Initialize buffer and max actuation rate for canards (e.g., 0.1 rad/s)
stateBuffer = nan(size(x_t, 1), 2);
prevCanardInput = struct('d1', 0, 'd2', 0, 'd3', 0, 'd4', 0); % Initial canard deflections

%% Fill steady state data for set period of time (Simulate Launcher)
steadyStateDuration = 5; % [s]
numSteadyPts = steadyStateDuration / time.dt;

% Pre-Fill steady state values
tRecord(1:numSteadyPts) = linspace(-steadyStateDuration+time.dt, time.t0, numSteadyPts);
xRecord(:, 1:numSteadyPts) = repmat(x_0, 1, numSteadyPts); % Repeat initial state
cmdHist(:, 1:numSteadyPts) = zeros(4, numSteadyPts); % Zero canard deflections
colNum = numSteadyPts;

while(currLLA(3) >= -5)
    colNum = colNum + 1;

    % Update buffer with the latest state; shift older states
    stateBuffer(:, 2) = stateBuffer(:, 1);
    stateBuffer(:, 1) = x_t;

    % Attempt to control roll between 4s and 8s
    if(t >= steadyStateDuration + 4 && t <= steadyStateDuration + 8)
        rollCmd = deg2rad(35);
        canardTargetInput = RollController_PID(stateBuffer, rollCmd, 0.4, 0, 0, time.dt);
        % canardTargetInput = RollPitchYawController_PID(stateBuffer, 0, 0, 0, 0.4, 0, 0, 0.4, 0, 0, 0.4, 0, 0, time.dt);

        canardInput = constrainMissileAcutationLimits(x_t, canardTargetInput, prevCanardInput, kins, time);

        % Update the historical command for analysis
        cmdHist(:,colNum) = [canardInput.d1; canardInput.d2; canardInput.d3; canardInput.d4];

        % Update previous canard input state for next iteration
        prevCanardInput = canardInput;
    else
        % No roll control, reset canards to 0 rad
        canardInput.d1 = deg2rad(0);
        canardInput.d2 = deg2rad(0);
        canardInput.d3 = deg2rad(0);
        canardInput.d4 = deg2rad(0);
    end

    % Define the anonymous function for ode45 that captures the inputs
    missileModelODE = @(t, x_t) MissileDynamicModel(x_t, t, canardInput, AeroModel, MotorModel, const, kins, inds);

    % Call ode45 for a small time step from current t to t + dt
    [t_out, x_out] = ode45(missileModelODE, [t, t + time.dt], x_t, options);

    % Update time and state variables with the last output from ode45
    t = t_out(end);
    x_t = x_out(end, :)';  % Transpose to maintain consistency with your original state vector format

    % Convert ECEF position to LLA for altitude check
    currLLA = ecef2lla([x_t(inds.px_ecef)', x_t(inds.py_ecef)', x_t(inds.pz_ecef)']);

    % Record the results for future analysis
    xRecord(:, colNum) = x_t;
    tRecord(1, colNum) = t;

    % Extract Acceleration Readings
    [~, accel_ecef] = MissileDynamicModel(x_out(end, :)', t, canardInput, AeroModel, MotorModel, const, kins, inds);

    sensorReading = generateIMU_Readings(x_t, accel_ecef, ImuModel, inds, const);
end

%% Plot Vehicle Trajectory
lla = ecef2lla([xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)']);

lla_target = ecef2lla([xRecord_target(1, :)', xRecord_target(2, :)', xRecord_target(3, :)']);

position_target_ECEF = [xRecord_target(1, :)', xRecord_target(2, :)', xRecord_target(3, :)'];

% Create a geoglobe
uif = uifigure('Name', 'Vehicle Trajectory');
g = geoglobe(uif);

geoplot3(g, lla(:, 1), lla(:,2), lla(:,3),"y");
hold(g,'on') % retains plot so that new plots can be added to the same plot
geoplot3(g, lla_target(:, 1), lla_target(:,2), lla_target(:,3), "r");
hold(g,'off')

%% Euler Angles
eulHist = quat2eul(xRecord(1:4, :)', 'ZYX');

yawHist   = rad2deg(eulHist(:,1));
pitchHist = rad2deg(eulHist(:,2));
rollHist  = rad2deg(eulHist(:,3));

% Plot
figure('Name', 'Orientation');
plot(tRecord(:), yawHist);
hold on;
plot(tRecord(:), pitchHist);
plot(tRecord(:), rollHist);
hold off;
title("Euler Angles");
legend('Yaw', 'Pitch', 'Roll');

figure('Name', 'Angular Velocity');
plot(tRecord(:), xRecord(inds.w_ib_x,:));
hold on;
plot(tRecord(:), xRecord(inds.w_ib_y,:));
plot(tRecord(:), xRecord(inds.w_ib_z,:));
hold off;
title("Angular Velocity");
legend('P', 'Q', 'R');

%% Vehicle State

figure('Name', 'Altitude');
plot(tRecord(:), lla(:,3))
title("Altitude Vs. Time");
ylabel("Altitude (m)");
xlabel("Time (s)");
grid on;

velocityHist = vecnorm(xRecord(inds.vel, :));

figure('Name', 'Velocity');
plot(tRecord(:), velocityHist);
title('Velocity Vs. Time');
ylabel("Velocity (m/s)");
xlabel("Time (s)");
grid on;

% Altitude Vs Downrange
downrange = getHaversine(launchLLA(1), launchLLA(2), lla(:,1), lla(:,2), const);
figure('Name', 'Conops');
plot(downrange, lla(:,3));
title("Mission Conops");
ylabel("Altitude (m)");
xlabel("Downrange (m)");
grid on;

%% Canard Command History
figure('Name', 'Canard Angles');
plot(tRecord(:), rad2deg(cmdHist));
title('Canard Actuation');
ylabel("Actuation (deg)");
xlabel("Time (s)");
grid on;
legend('Canard 1', 'Canard 2', 'Canard 3', 'Canard 4');

% figure('Name', 'Target Position');
% subplot(3,1,1);
% plot(tRecord(:), position_target_ECEF(:,1))
% title("Target Position X Vs. Time");
% ylabel("Position (m)");
% xlabel("Time (s)");
% grid on;
% subplot(3,1,2);
% plot(tRecord(:), position_target_ECEF(:,2))
% title("Target Position Y Vs. Time");
% ylabel("Position (m)");
% xlabel("Time (s)");
% grid on;
% subplot(3,1,3);
% plot(tRecord(:), position_target_ECEF(:,3))
% title("Target Position Z Vs. Time");
% ylabel("Position (m)");
% xlabel("Time (s)");
% grid on;